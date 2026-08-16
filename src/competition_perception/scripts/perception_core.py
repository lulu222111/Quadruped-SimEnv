#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
perception_core — pure perception logic, free of any ROS imports.

Split out of perception_node so that the freshness/generation state
machine, classification helpers and depth math can be exercised by
deterministic unit tests without a ROS master, Gazebo or a RealSense.

All time values in this module are float seconds on the ROS /clock
timeline.  The classes below are plain state containers with pure
methods: the owning ROS node is responsible for serializing concurrent
access (the watchdog timer runs on a different thread than the
synchronized image callback).

RGB / depth alignment assumption
--------------------------------
The competition simulator models the RealSense as a single
``gazebo_ros_openni_kinect`` sensor (see
``unitree_ros/robots/a1_description/xacro/gazebo.xacro``): one camera
block (640x480, 60 deg HFOV) shared by the RGB and depth streams, with
a single ``real_sense_optical_frame`` frame name.  The plugin renders
colour and depth from the identical projection, so both images are
registered by construction and the proportional RGB-pixel -> depth-pixel
mapping used by :func:`robust_depth` / :func:`project_to_3d` is exact
(scale 1.0 for equal resolutions).

This assumption must NOT be carried over to a real, unregistered
D435 colour/depth pair: on hardware the streams have different
resolutions, fields of view and extrinsic offsets.
"""

from __future__ import annotations

import math

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Semantic labels (module internal, not a frozen cross-module contract).
# ---------------------------------------------------------------------------

RED_SPHERE_LABEL = "red_sphere"
RED_BOX_LABEL = "red_box"
GREEN_SPHERE_LABEL = "green_sphere"

# ---------------------------------------------------------------------------
# Classification thresholds (module-internal implementation parameters,
# not part of the frozen v0.2.0 contract).
#
# A square silhouette has circularity pi/4 ~= 0.785 and a
# min-enclosing-circle fill ratio 2/pi ~= 0.637, so requiring BOTH a high
# circularity AND a high circle ratio keeps a straight-on or angled red
# box out of the sphere class while a discretized circle contour
# (circularity ~0.9+) still passes.  The 0.20..0.42 m diameter window
# brackets the 0.30 m danger sphere / 0.30 m distractor box.
# ---------------------------------------------------------------------------

SPHERE_CIRCULARITY_MIN = 0.80
SPHERE_CIRCLE_RATIO_MIN = 0.78
SPHERE_ASPECT_MIN = 0.75
SPHERE_ASPECT_MAX = 1.33
SPHERE_DIAMETER_MIN_M = 0.20
SPHERE_DIAMETER_MAX_M = 0.42

# Morphological denoising.
MORPH_KERNEL_SIZE = 5
MORPH_OPEN_ITERATIONS = 1
MORPH_CLOSE_ITERATIONS = 2

# Fraction of the contour bounding box used as the half-size of the
# median-depth window around the contour centroid.
DEPTH_WINDOW_FRACTION = 0.2

# Confidence scoring reference area in pixels.
CONFIDENCE_AREA_REFERENCE_PX = 3000.0


def clamp_confidence(value):
    """Clamp a confidence value to a finite number inside [0, 1]."""

    try:
        value = float(value)
    except (TypeError, ValueError):
        return 0.0

    if not math.isfinite(value):
        return 0.0

    return max(0.0, min(1.0, value))


class ObservationGate(object):
    """
    Freshness / generation / timeout / recovery state machine.

    Implements the semantics required by docs/team/module_contracts.md
    v0.2.0 for perception:

      * ``valid=false`` must invalidate any previously cached/latched
        output;
      * an invalidation event (input timeout, ROS /clock rollback) starts
        a new local generation; observations from an older generation
        must never be consumed again;
      * recovery requires a fresh observation that belongs to the
        current generation;
      * the arrival order of "recovered status" and "current generation
        data" is not fixed: current-generation data may arrive while the
        module is invalid and may be evaluated, but only a fresh,
        current-generation observation actually restores validity.

    Generation boundary
    -------------------
    Every generation transition records ``generation_start_stamp``, the
    ROS time at which the transition was observed.  The boundary is
    strict-after: an observation must be stamped STRICTLY AFTER it to
    belong to the current generation.

      * timeout: the watchdog tick that entered the timeout state —
        observations stamped at or before that instant were produced
        before the transition and must never recover the module, even
        when their age is still inside ``max_input_age_sec``;
      * ROS /clock rollback: the new (lower) clock value — stamps at or
        below it are pre-transition; the old timeline lies above it and
        is additionally rejected by the far-future rule.

    The transition fires exactly once: repeated watchdog ticks while
    already timed out do not create further generations, and only the
    first tick's boundary is used.

    Admission rules for an observation stamped ``stamp`` arriving at ROS
    time ``now``:

      1. zero / non-finite stamp                  -> rejected;
      2. stamp > now + future_tolerance_sec       -> rejected (far future);
      3. stamp < now - max_input_age_sec          -> rejected (stale);
      4. stamp <= generation_start_stamp          -> rejected (pre-
                                                     generation data);
      5. stamp == last accepted stamp             -> rejected (duplicate
                                                     within generation);
      6. stamp < last accepted stamp              -> rejected (out-of-order
                                                     within generation);
      7. otherwise                                -> admitted.

    Rules 2 and 3 are skipped while the ROS clock has not started yet
    (``now == 0``), mirroring simulation startup.  Rules 5 and 6 apply
    while a monotonic reference exists within the current generation.
    """

    def __init__(
        self,
        timeout_sec,
        future_tolerance_sec,
        max_input_age_sec,
    ):
        self.timeout_sec = float(timeout_sec)
        self.future_tolerance_sec = float(future_tolerance_sec)
        self.max_input_age_sec = float(max_input_age_sec)

        # Local generation counter (internal bookkeeping only; the public
        # messages intentionally carry no epoch field per the contract).
        self.generation_id = 0

        # ROS time at which the current generation started.  Observations
        # stamped before it belong to an older generation and are never
        # admitted.  None until the first generation transition.
        self.generation_start_stamp = None

        # Monotonic reference within the current generation.
        self.last_accepted_stamp = None

        # ROS time at which the last observation was accepted; drives the
        # watchdog.
        self.last_accepted_time = None

        # True while the module output must be considered invalid due to
        # timeout or clock rollback.
        self.timed_out = False

        # ROS time used for the "no data at all" startup timeout.
        self.started_at = None

        # Last ROS clock value observed; used to detect rollback.
        self.last_clock_time = None

    # -- clock ------------------------------------------------------------

    def check_clock(self, now):
        """
        Detect a ROS /clock rollback.

        Returns ``None`` when the clock is sane.  On rollback it starts a
        new generation, invalidates the module and returns a human
        readable detail string for the status message.
        """

        if self.last_clock_time is not None and now < self.last_clock_time:
            self.begin_new_generation(now)
            # Anchor to the new timeline so the rollback is detected
            # exactly once and subsequent ticks on the new clock do not
            # re-trigger it.
            self.last_clock_time = now
            return (
                "ROS clock moved backward; generation %d started, "
                "waiting for fresh current-generation RGB-D observation"
                % self.generation_id
            )

        self.last_clock_time = now
        return None

    def begin_new_generation(self, now):
        """
        Invalidate and start a new local generation.

        ``now`` becomes the generation boundary: observations stamped
        at or before it were produced before this transition and must
        never recover the module (recovery requires stamp strictly
        greater than the boundary).
        """

        self.generation_id += 1
        self.generation_start_stamp = now
        self.last_accepted_stamp = None
        self.last_accepted_time = None
        self.timed_out = True
        self.started_at = now

    # -- admission --------------------------------------------------------

    def evaluate(self, now, stamp):
        """
        Admit or reject an observation.

        Returns ``None`` when the observation may be consumed, otherwise
        a short reason string.
        """

        if stamp is None or not math.isfinite(stamp) or stamp <= 0.0:
            return "zero"

        if now > 0.0:
            if stamp > now + self.future_tolerance_sec:
                return "future"

            if stamp < now - self.max_input_age_sec:
                return "stale"

        if (
            self.generation_start_stamp is not None
            and stamp <= self.generation_start_stamp
        ):
            return "pre_generation"

        if self.last_accepted_stamp is not None:
            if stamp == self.last_accepted_stamp:
                return "duplicate"

            if stamp < self.last_accepted_stamp:
                return "out_of_order"

        return None

    def accept(self, now, stamp):
        """
        Record a successfully processed current-generation observation.

        Updates freshness and the monotonic reference, clears the timeout
        flag and returns ``True`` when this observation recovered the
        module from an invalid state.
        """

        self.last_accepted_time = now
        self.last_accepted_stamp = stamp

        if self.started_at is None:
            self.started_at = now

        recovered = self.timed_out
        self.timed_out = False
        return recovered

    # -- watchdog ---------------------------------------------------------

    def watchdog(self, now):
        """
        Run one watchdog tick.

        Returns ``None`` when nothing needs publishing, otherwise a
        detail string for a TIMEOUT status.

        Entering the timeout state is a generation transition: it bumps
        ``generation_id`` exactly once, records ``now`` as the new
        generation boundary (observations stamped before it are
        pre-transition data and must never recover the module) and
        invalidates the module.  Repeated ticks while already timed out
        return ``None`` and never create further generations.
        """

        if now <= 0.0:
            return None

        if self.started_at is None:
            self.started_at = now

        if self.timed_out:
            return None

        if self.last_accepted_time is None:
            age_sec = now - self.started_at
            detail = (
                "synchronized RGB-D input timeout; "
                "waiting for first valid observation"
            )
        else:
            age_sec = now - self.last_accepted_time
            detail = (
                "synchronized RGB-D input timeout; waiting for fresh "
                "current-generation observation"
            )

        if age_sec > self.timeout_sec:
            self.begin_new_generation(now)
            return detail

        return None

    # -- derived status ---------------------------------------------------

    @property
    def valid(self):
        """
        Contract gate: the module output is usable only when an
        observation has actually been accepted in the current generation
        and the module is not timed out.
        """

        return self.last_accepted_stamp is not None and not self.timed_out


# ---------------------------------------------------------------------------
# Depth helpers
# ---------------------------------------------------------------------------

def robust_depth(
    depth_image_m,
    center_u,
    center_v,
    bbox_width,
    bbox_height,
    rgb_shape,
    min_depth,
    max_depth,
    window_fraction=DEPTH_WINDOW_FRACTION,
):
    """
    Median of the valid depths in a small window around the contour
    centroid.  Returns ``None`` when no valid depth exists.

    RGB pixels are mapped onto the depth image by resolution ratio; the
    competition simulator guarantees aligned RGB/depth streams (see the
    module docstring).
    """

    if depth_image_m is None or len(depth_image_m.shape) != 2:
        return None

    depth_height, depth_width = depth_image_m.shape[:2]
    rgb_height, rgb_width = rgb_shape[:2]

    if (
        depth_width <= 0
        or depth_height <= 0
        or rgb_width <= 0
        or rgb_height <= 0
    ):
        return None

    scale_x = float(depth_width) / float(rgb_width)
    scale_y = float(depth_height) / float(rgb_height)

    u = int(round(center_u * scale_x))
    v = int(round(center_v * scale_y))

    u = max(0, min(depth_width - 1, u))
    v = max(0, min(depth_height - 1, v))

    half_w = max(2, int(round(bbox_width * scale_x * window_fraction)))
    half_h = max(2, int(round(bbox_height * scale_y * window_fraction)))

    u0 = max(0, u - half_w)
    u1 = min(depth_width, u + half_w + 1)
    v0 = max(0, v - half_h)
    v1 = min(depth_height, v + half_h + 1)

    if u0 >= u1 or v0 >= v1:
        return None

    roi = depth_image_m[v0:v1, u0:u1]
    roi = roi[np.isfinite(roi)]

    if roi.size == 0:
        return None

    valid = roi[(roi >= min_depth) & (roi <= max_depth)]

    if valid.size == 0:
        return None

    return float(np.median(valid))


def project_to_3d(
    center_u,
    center_v,
    depth_value,
    depth_shape,
    rgb_shape,
    fx,
    fy,
    cx,
    cy,
    min_depth,
    max_depth,
):
    """
    Back-project an RGB pixel onto the depth camera coordinate frame
    (ROS optical convention: X right, Y down, Z forward).

    Returns an ``(x, y, z)`` tuple in metres or ``None`` when the input
    is unusable.  RGB pixels are mapped onto the depth image by
    resolution ratio (aligned by construction in the competition
    simulator; see the module docstring).
    """

    if (
        depth_shape is None
        or rgb_shape is None
        or len(depth_shape) < 2
        or len(rgb_shape) < 2
    ):
        return None

    depth_height, depth_width = depth_shape[:2]
    rgb_height, rgb_width = rgb_shape[:2]

    if (
        depth_width <= 0
        or depth_height <= 0
        or rgb_width <= 0
        or rgb_height <= 0
    ):
        return None

    if (
        depth_value is None
        or not math.isfinite(depth_value)
        or depth_value < min_depth
        or depth_value > max_depth
    ):
        return None

    if not all(
        math.isfinite(value)
        for value in (fx, fy, cx, cy)
    ):
        return None

    if fx <= 0.0 or fy <= 0.0:
        return None

    if not math.isfinite(center_u) or not math.isfinite(center_v):
        return None

    depth_u = float(center_u) * float(depth_width) / float(rgb_width)
    depth_v = float(center_v) * float(depth_height) / float(rgb_height)

    if not math.isfinite(depth_u) or not math.isfinite(depth_v):
        return None

    x = (depth_u - cx) * depth_value / fx
    y = (depth_v - cy) * depth_value / fy
    z = depth_value

    if not all(math.isfinite(value) for value in (x, y, z)):
        return None

    return (float(x), float(y), float(z))


def estimate_diameter_m(
    bbox_width,
    bbox_height,
    depth_value,
    fx,
    fy,
    rgb_shape,
    depth_shape,
):
    """
    Metric diameter of the contour bounding box at ``depth_value``.
    Returns ``None`` when depth or intrinsics are unusable.
    """

    if (
        depth_value is None
        or not math.isfinite(depth_value)
        or depth_value <= 0.0
        or not math.isfinite(fx)
        or not math.isfinite(fy)
        or fx <= 0.0
        or fy <= 0.0
    ):
        return None

    depth_height, depth_width = depth_shape[:2]
    rgb_height, rgb_width = rgb_shape[:2]

    if (
        rgb_width <= 0
        or rgb_height <= 0
        or depth_width <= 0
        or depth_height <= 0
    ):
        return None

    width_depth_px = float(bbox_width) * depth_width / rgb_width
    height_depth_px = float(bbox_height) * depth_height / rgb_height

    diameter_x = width_depth_px * depth_value / fx
    diameter_y = height_depth_px * depth_value / fy

    if not math.isfinite(diameter_x) or not math.isfinite(diameter_y):
        return None

    return 0.5 * (diameter_x + diameter_y)


# ---------------------------------------------------------------------------
# Classification and confidence
# ---------------------------------------------------------------------------

def classify_object(
    color_name,
    circularity,
    circle_ratio,
    aspect_ratio,
    diameter_m=None,
):
    """
    Produce the semantic label of a detected object.

    ``color_name`` is ``"red"`` or ``"green"``.  Green objects are always
    the green sphere distractor.  Red objects are classified as the
    danger sphere only when the 2D silhouette is convincingly circular
    (circularity AND min-enclosing-circle fill ratio AND aspect ratio)
    and — when a depth estimate exists — the metric diameter lies in the
    expected 0.20..0.42 m window; otherwise they fall back to ``red_box``.
    """

    if color_name == "green":
        return GREEN_SPHERE_LABEL

    is_circular_2d = (
        circularity >= SPHERE_CIRCULARITY_MIN
        and circle_ratio >= SPHERE_CIRCLE_RATIO_MIN
        and SPHERE_ASPECT_MIN <= aspect_ratio <= SPHERE_ASPECT_MAX
    )

    if diameter_m is None:
        return RED_SPHERE_LABEL if is_circular_2d else RED_BOX_LABEL

    if (
        is_circular_2d
        and SPHERE_DIAMETER_MIN_M <= diameter_m <= SPHERE_DIAMETER_MAX_M
    ):
        return RED_SPHERE_LABEL

    return RED_BOX_LABEL


def calculate_confidence(
    area,
    circularity,
    aspect_ratio,
    position_valid,
    label,
):
    """Heuristic detection confidence in [0, 1]."""

    area_score = min(1.0, float(area) / CONFIDENCE_AREA_REFERENCE_PX)
    circularity_score = clamp_confidence(circularity)

    aspect_score = max(
        0.0,
        1.0 - min(1.0, abs(1.0 - aspect_ratio)),
    )

    depth_score = 1.0 if position_valid else 0.25

    if label in (RED_SPHERE_LABEL, GREEN_SPHERE_LABEL):
        shape_score = (
            0.45 * circularity_score
            + 0.30 * aspect_score
            + 0.25 * depth_score
        )
    else:
        shape_score = (
            0.35 * (1.0 - circularity_score)
            + 0.35 * aspect_score
            + 0.30 * depth_score
        )

    confidence = (
        0.20 * area_score
        + 0.65 * shape_score
        + 0.15 * depth_score
    )
    return clamp_confidence(confidence)


# ---------------------------------------------------------------------------
# Detection pipeline
# ---------------------------------------------------------------------------

def clean_mask(mask):
    """Morphological denoising of a colour mask."""

    kernel = np.ones((MORPH_KERNEL_SIZE, MORPH_KERNEL_SIZE), np.uint8)

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
        iterations=MORPH_OPEN_ITERATIONS,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=MORPH_CLOSE_ITERATIONS,
    )

    return mask


def detect_from_mask(
    mask,
    color_name,
    depth_image_m,
    depth_shape,
    rgb_shape,
    fx,
    fy,
    cx,
    cy,
    min_contour_area,
    min_depth,
    max_depth,
):
    """
    Extract contours from one colour mask and produce detection dicts:

      {"u", "v", "label", "confidence", "position_valid", "position"}

    ``position`` is an ``(x, y, z)`` tuple in the depth camera frame or
    ``None`` when it could not be validated.
    """

    contours_result = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    # OpenCV >= 3: (contours, hierarchy)
    contours = contours_result[-2]
    detections = []

    for contour in contours:
        area = cv2.contourArea(contour)

        if area < min_contour_area:
            continue

        _, _, width, height = cv2.boundingRect(contour)

        if width <= 0 or height <= 0:
            continue

        perimeter = cv2.arcLength(contour, True)

        if perimeter <= 0.0:
            continue

        circularity = 4.0 * math.pi * area / (perimeter * perimeter)
        aspect_ratio = float(width) / float(height)

        (_, _), radius = cv2.minEnclosingCircle(contour)
        circle_area = math.pi * radius * radius
        circle_ratio = area / circle_area if circle_area > 0.0 else 0.0

        moments = cv2.moments(contour)

        if abs(moments["m00"]) < 1e-6:
            continue

        center_u = moments["m10"] / moments["m00"]
        center_v = moments["m01"] / moments["m00"]

        depth_value = robust_depth(
            depth_image_m=depth_image_m,
            center_u=center_u,
            center_v=center_v,
            bbox_width=width,
            bbox_height=height,
            rgb_shape=rgb_shape,
            min_depth=min_depth,
            max_depth=max_depth,
        )

        depth_valid = (
            depth_value is not None
            and math.isfinite(depth_value)
            and min_depth <= depth_value <= max_depth
        )

        point = None
        if depth_valid:
            point = project_to_3d(
                center_u=center_u,
                center_v=center_v,
                depth_value=depth_value,
                depth_shape=depth_shape,
                rgb_shape=rgb_shape,
                fx=fx,
                fy=fy,
                cx=cx,
                cy=cy,
                min_depth=min_depth,
                max_depth=max_depth,
            )

        position_valid = (
            point is not None
            and all(math.isfinite(value) for value in point)
        )

        diameter_m = estimate_diameter_m(
            bbox_width=width,
            bbox_height=height,
            depth_value=depth_value,
            fx=fx,
            fy=fy,
            rgb_shape=rgb_shape,
            depth_shape=depth_shape,
        )

        label = classify_object(
            color_name=color_name,
            circularity=circularity,
            circle_ratio=circle_ratio,
            aspect_ratio=aspect_ratio,
            diameter_m=diameter_m,
        )

        confidence = calculate_confidence(
            area=area,
            circularity=circularity,
            aspect_ratio=aspect_ratio,
            position_valid=position_valid,
            label=label,
        )

        detections.append(
            {
                "u": float(center_u),
                "v": float(center_v),
                "label": label,
                "confidence": clamp_confidence(confidence),
                "position_valid": position_valid,
                "position": point if position_valid else None,
            }
        )

    return detections


def detect_objects(
    bgr_image,
    depth_image_m,
    fx,
    fy,
    cx,
    cy,
    min_contour_area,
    min_depth,
    max_depth,
    red_lower_1,
    red_upper_1,
    red_lower_2,
    red_upper_2,
    green_lower,
    green_upper,
):
    """
    Full detection pipeline on one aligned RGB-D pair.

    ``bgr_image``: BGR uint8 array; ``depth_image_m``: float32 metres.
    Returns a list of detection dicts (possibly empty when no hazard or
    distractor object is visible — an empty list does NOT mean the
    module is invalid).
    """

    rgb_shape = bgr_image.shape
    depth_shape = depth_image_m.shape

    hsv_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2HSV)

    red_mask_1 = cv2.inRange(hsv_image, red_lower_1, red_upper_1)
    red_mask_2 = cv2.inRange(hsv_image, red_lower_2, red_upper_2)
    red_mask = cv2.bitwise_or(red_mask_1, red_mask_2)

    green_mask = cv2.inRange(hsv_image, green_lower, green_upper)

    red_mask = clean_mask(red_mask)
    green_mask = clean_mask(green_mask)

    detections = []

    detections.extend(
        detect_from_mask(
            mask=red_mask,
            color_name="red",
            depth_image_m=depth_image_m,
            depth_shape=depth_shape,
            rgb_shape=rgb_shape,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            min_contour_area=min_contour_area,
            min_depth=min_depth,
            max_depth=max_depth,
        )
    )

    detections.extend(
        detect_from_mask(
            mask=green_mask,
            color_name="green",
            depth_image_m=depth_image_m,
            depth_shape=depth_shape,
            rgb_shape=rgb_shape,
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            min_contour_area=min_contour_area,
            min_depth=min_depth,
            max_depth=max_depth,
        )
    )

    return detections
