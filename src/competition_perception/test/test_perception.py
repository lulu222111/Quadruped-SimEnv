#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Deterministic unit tests for competition_perception.

Pure Python / numpy / OpenCV tests — no ROS master, no Gazebo, no
RealSense.  They cover:

  A. fresh RGB-D observation            -> accepted, module valid
  B. valid input without candidates     -> pipeline returns [] (module
                                           validity is owned by the gate,
                                           not by the detection count)
  C. candidate without reliable depth   -> position_valid=False, module
                                           validity unaffected
  D. observation timeout                -> gate invalidated (TIMEOUT)
  E. delayed / slightly-newer-but-stale observation after a timeout
                                        -> must NOT recover the module
  F. fresh current-generation observation after a timeout
                                        -> recovers the module
  G. ROS /clock rollback                -> new generation, invalidation,
                                           old-generation observation
                                           rejected, fresh new-generation
                                           observation recovers
  H. duplicate / out-of-order stamps    -> rejected without refreshing
                                           freshness
  I. synthetic classification           -> red sphere, red box, green
                                           sphere
  J. no valid depth                     -> position_valid=False

Runnable with plain unittest:

    python3 -m unittest discover -s test -v

or through catkin (catkin_add_nosetests in CMakeLists.txt):

    catkin run_tests competition_perception
"""

from __future__ import annotations

import math
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(
    0,
    os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "scripts"),
)

import perception_core


def make_gate(timeout_sec=1.0, future_tolerance_sec=0.5,
              max_input_age_sec=1.0):
    return perception_core.ObservationGate(
        timeout_sec=timeout_sec,
        future_tolerance_sec=future_tolerance_sec,
        max_input_age_sec=max_input_age_sec,
    )


# Synthetic camera: 640x480, 60 deg HFOV.
FX = FY = 320.0 / math.tan(math.radians(30.0))  # ~554.2563
CX = 319.5
CY = 239.5
MIN_DEPTH = 0.4
MAX_DEPTH = 8.0
MIN_AREA = 150.0

RED_BGR = (0, 0, 255)
GREEN_BGR = (0, 255, 0)

RED_LOWER_1 = np.array([0, 80, 60], dtype=np.uint8)
RED_UPPER_1 = np.array([10, 255, 255], dtype=np.uint8)
RED_LOWER_2 = np.array([170, 80, 60], dtype=np.uint8)
RED_UPPER_2 = np.array([179, 255, 255], dtype=np.uint8)
GREEN_LOWER = np.array([35, 60, 50], dtype=np.uint8)
GREEN_UPPER = np.array([90, 255, 255], dtype=np.uint8)


def detect(bgr, depth_m):
    return perception_core.detect_objects(
        bgr_image=bgr,
        depth_image_m=depth_m,
        fx=FX,
        fy=FY,
        cx=CX,
        cy=CY,
        min_contour_area=MIN_AREA,
        min_depth=MIN_DEPTH,
        max_depth=MAX_DEPTH,
        red_lower_1=RED_LOWER_1,
        red_upper_1=RED_UPPER_1,
        red_lower_2=RED_LOWER_2,
        red_upper_2=RED_UPPER_2,
        green_lower=GREEN_LOWER,
        green_upper=GREEN_UPPER,
    )


def blank_scene():
    """Gray background, uniform 2.0 m depth."""

    bgr = np.full((480, 640, 3), 128, dtype=np.uint8)
    depth_m = np.full((480, 640), 2.0, dtype=np.float32)
    return bgr, depth_m


def red_sphere_scene():
    """0.30 m red sphere at 2.0 m on the optical axis (~42 px radius)."""

    bgr, depth_m = blank_scene()
    cv2.circle(bgr, (320, 240), 42, RED_BGR, thickness=-1)
    return bgr, depth_m


def red_box_scene():
    """0.30 m red box at 2.0 m on the optical axis (~84 px square)."""

    bgr, depth_m = blank_scene()
    cv2.rectangle(bgr, (278, 198), (362, 282), RED_BGR, thickness=-1)
    return bgr, depth_m


def green_sphere_scene():
    bgr, depth_m = blank_scene()
    cv2.circle(bgr, (320, 240), 42, GREEN_BGR, thickness=-1)
    return bgr, depth_m


class ObservationGateTest(unittest.TestCase):
    """Freshness / generation / timeout / recovery state machine."""

    def test_startup_invalid(self):
        gate = make_gate()
        self.assertFalse(gate.valid)

    def test_A_fresh_observation_accepted_and_valid(self):
        gate = make_gate()
        self.assertIsNone(gate.evaluate(10.0, 9.5))
        self.assertFalse(gate.accept(10.0, 9.5))
        self.assertTrue(gate.valid)

    def test_zero_stamp_rejected(self):
        gate = make_gate()
        self.assertEqual(gate.evaluate(10.0, 0.0), "zero")
        self.assertEqual(gate.evaluate(10.0, None), "zero")
        self.assertEqual(gate.evaluate(10.0, float("nan")), "zero")

    def test_far_future_stamp_rejected(self):
        gate = make_gate()
        self.assertEqual(gate.evaluate(10.0, 10.6), "future")

    def test_within_future_tolerance_accepted(self):
        gate = make_gate()
        self.assertIsNone(gate.evaluate(10.0, 10.5))

    def test_stale_observation_rejected(self):
        gate = make_gate()
        self.assertIsNone(gate.evaluate(10.0, 9.0))
        self.assertEqual(gate.evaluate(10.0, 8.999), "stale")

    def test_stale_check_skipped_before_clock_starts(self):
        gate = make_gate()
        self.assertIsNone(gate.evaluate(0.0, 5.0))

    def test_H_duplicate_stamp_rejected(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        self.assertEqual(gate.evaluate(10.1, 9.5), "duplicate")

    def test_H_out_of_order_rejected_and_does_not_invalidate(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        self.assertEqual(gate.evaluate(10.2, 9.4), "out_of_order")
        self.assertTrue(gate.valid)

    def test_H_rejected_frames_do_not_refresh_freshness(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        # 9.4 is out-of-order (9.4 < 9.5) and exactly at the stale
        # boundary (10.4 - 1.0 == 9.4), so the out-of-order reason wins.
        self.assertEqual(gate.evaluate(10.4, 9.4), "out_of_order")
        # Freshness is still anchored at 10.0, so the watchdog fires.
        self.assertIsNone(gate.watchdog(10.9))
        self.assertIsNotNone(gate.watchdog(11.1))

    def test_D_timeout_after_silence(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        self.assertTrue(gate.valid)
        self.assertIsNone(gate.watchdog(10.9))
        self.assertIsNotNone(gate.watchdog(11.1))
        self.assertFalse(gate.valid)
        self.assertTrue(gate.timed_out)

    def test_D_timeout_starts_new_generation_exactly_once(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        self.assertEqual(gate.generation_id, 0)

        self.assertIsNone(gate.watchdog(10.9))
        self.assertIsNotNone(gate.watchdog(11.1))

        # First timeout tick: exactly one generation bump + boundary.
        self.assertEqual(gate.generation_id, 1)
        self.assertEqual(gate.generation_start_stamp, 11.1)
        self.assertFalse(gate.valid)

        # Repeated ticks neither bump the generation nor move the
        # boundary.
        self.assertIsNone(gate.watchdog(11.2))
        self.assertIsNone(gate.watchdog(12.0))
        self.assertEqual(gate.generation_id, 1)
        self.assertEqual(gate.generation_start_stamp, 11.1)

    def test_startup_timeout_starts_generation(self):
        gate = make_gate()
        gate.check_clock(10.0)
        self.assertIsNone(gate.watchdog(10.0))
        self.assertIsNotNone(gate.watchdog(11.1))
        self.assertEqual(gate.generation_id, 1)
        self.assertEqual(gate.generation_start_stamp, 11.1)

    def test_no_repeat_timeout_until_recovery(self):
        gate = make_gate()
        gate.accept(10.0, 9.5)
        self.assertIsNotNone(gate.watchdog(11.1))
        self.assertIsNone(gate.watchdog(11.2))
        self.assertIsNone(gate.watchdog(12.0))

    def test_startup_timeout_without_any_data(self):
        gate = make_gate()
        gate.check_clock(10.0)
        # The first watchdog tick anchors the "no data at all" clock.
        self.assertIsNone(gate.watchdog(10.0))
        self.assertIsNone(gate.watchdog(10.5))
        self.assertFalse(gate.valid)
        self.assertIsNotNone(gate.watchdog(11.1))
        self.assertFalse(gate.valid)

    def test_watchdog_silent_before_clock_starts(self):
        gate = make_gate()
        self.assertIsNone(gate.watchdog(0.0))

    def test_E_p01_delayed_observation_after_timeout_stays_invalid(self):
        # Exact blocker scenario: last accepted stamp 10.0, timeout at
        # now=12.0, delayed frame stamped 10.1 (newer than 10.0 but
        # 1.9 s old) must NOT restore RUNNING/valid=true.
        gate = make_gate()
        gate.accept(10.05, 10.0)
        self.assertTrue(gate.valid)
        self.assertIsNotNone(gate.watchdog(12.0))
        self.assertFalse(gate.valid)

        self.assertEqual(gate.evaluate(12.0, 10.1), "stale")
        self.assertFalse(gate.valid)

        self.assertEqual(gate.evaluate(12.0, 10.6), "stale")
        self.assertFalse(gate.valid)

    def test_E_pre_timeout_frame_rejected_by_generation_boundary(self):
        # A frame produced BEFORE the timeout transition (stamp 11.5,
        # boundary 12.0) must not recover the module even though its
        # age (0.6 s) is still inside max_input_age.
        gate = make_gate()
        gate.accept(10.05, 10.0)
        self.assertIsNotNone(gate.watchdog(12.0))
        self.assertEqual(gate.generation_start_stamp, 12.0)

        self.assertEqual(gate.evaluate(12.1, 11.5), "pre_generation")
        self.assertFalse(gate.valid)

    def test_boundary_equality_frame_rejected(self):
        # The boundary is strict-after: a frame stamped exactly at the
        # transition instant is pre-generation and must be rejected.
        gate = make_gate()
        gate.accept(10.05, 10.0)
        gate.watchdog(12.0)
        self.assertEqual(gate.evaluate(12.0, 12.0), "pre_generation")
        self.assertFalse(gate.valid)

        # The first frame strictly after the boundary is admitted.
        self.assertIsNone(gate.evaluate(12.05, 12.05))
        self.assertTrue(gate.accept(12.05, 12.05))
        self.assertTrue(gate.valid)

    def test_F_fresh_observation_recovers_after_timeout(self):
        # Recovery requires a frame produced after the transition.
        gate = make_gate()
        gate.accept(10.05, 10.0)
        self.assertIsNotNone(gate.watchdog(12.0))
        self.assertIsNone(gate.evaluate(12.1, 12.05))
        self.assertTrue(gate.accept(12.1, 12.05))
        self.assertTrue(gate.valid)
        self.assertFalse(gate.timed_out)

    def test_G_clock_rollback_starts_new_generation(self):
        gate = make_gate()
        gate.check_clock(100.0)
        self.assertIsNone(gate.evaluate(100.0, 99.9))
        gate.accept(100.0, 99.9)
        self.assertTrue(gate.valid)

        gen_before = gate.generation_id
        self.assertIsNone(gate.check_clock(100.1))

        rollback_detail = gate.check_clock(50.0)
        self.assertIsNotNone(rollback_detail)
        self.assertEqual(gate.generation_id, gen_before + 1)
        self.assertFalse(gate.valid)
        self.assertTrue(gate.timed_out)

    def test_G_old_generation_observation_rejected_after_rollback(self):
        gate = make_gate()
        gate.check_clock(100.0)
        gate.accept(100.0, 99.9)
        gate.check_clock(100.1)
        gate.check_clock(50.0)  # rollback

        # Pre-rollback observation is now far in the future.
        self.assertEqual(gate.evaluate(50.0, 99.9), "future")
        self.assertFalse(gate.valid)

    def test_G_fresh_new_generation_observation_recovers(self):
        gate = make_gate()
        gate.check_clock(100.0)
        gate.accept(100.0, 99.9)
        gate.check_clock(100.1)
        gate.check_clock(50.0)  # rollback -> boundary 50.0

        # New-generation stamps start at the rollback point; the
        # monotonic reference was cleared, so this must be admitted.
        self.assertIsNone(gate.evaluate(50.2, 50.1))
        self.assertTrue(gate.accept(50.2, 50.1))
        self.assertTrue(gate.valid)
        self.assertEqual(gate.last_accepted_stamp, 50.1)

    def test_G_rollback_boundary_rejects_pre_rollback_stamp(self):
        gate = make_gate()
        gate.check_clock(100.0)
        gate.accept(100.0, 99.9)
        gate.check_clock(100.1)
        gate.check_clock(50.0)  # rollback -> boundary 50.0

        # A stamp below the rollback point cannot exist in the new
        # timeline and must be rejected by the generation boundary.
        self.assertEqual(gate.evaluate(50.2, 49.9), "pre_generation")
        self.assertFalse(gate.valid)

    def test_accept_returns_recovered_flag_only_when_timed_out(self):
        gate = make_gate()
        self.assertFalse(gate.accept(10.0, 9.5))
        self.assertIsNotNone(gate.watchdog(11.1))
        # Realistic path: evaluate (admission) then accept.
        self.assertIsNone(gate.evaluate(11.2, 11.2))
        self.assertTrue(gate.accept(11.2, 11.2))

    def test_check_clock_monotonic_sequence_never_rolls_back(self):
        gate = make_gate()
        for now in (10.0, 10.1, 10.2, 10.3, 10.4, 10.5):
            self.assertIsNone(gate.check_clock(now))
        self.assertEqual(gate.last_clock_time, 10.5)

    def test_check_clock_strictly_decreasing_detects_rollback(self):
        gate = make_gate()
        gate.check_clock(10.5)
        self.assertIsNotNone(gate.check_clock(10.4))

    def test_check_clock_rollback_detected_exactly_once(self):
        # Regression: after the rollback the gate must anchor to the new
        # timeline, so later ticks on the new clock are NOT detected as
        # further rollbacks.
        gate = make_gate()
        gate.check_clock(10.0)
        self.assertIsNotNone(gate.check_clock(1.0))
        self.assertIsNone(gate.check_clock(1.1))
        self.assertIsNone(gate.check_clock(1.2))
        self.assertEqual(gate.last_clock_time, 1.2)

    def test_check_clock_ok(self):
        gate = make_gate()
        self.assertIsNone(gate.check_clock(10.0))
        self.assertIsNone(gate.check_clock(10.5))
        self.assertEqual(gate.last_clock_time, 10.5)


class ClassificationTest(unittest.TestCase):
    """Classification helpers and confidence scoring."""

    def test_I_green_always_green_sphere(self):
        self.assertEqual(
            perception_core.classify_object("green", 0.5, 0.5, 1.0, None),
            "green_sphere",
        )
        self.assertEqual(
            perception_core.classify_object("green", 0.9, 0.9, 1.0, 0.30),
            "green_sphere",
        )

    def test_I_red_circle_features_are_sphere(self):
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.95, 0.90, 1.0, 0.30),
            "red_sphere",
        )

    def test_I_red_circle_without_depth_uses_2d_shape(self):
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.95, 0.90, 1.0, None),
            "red_sphere",
        )

    def test_I_red_square_features_are_box(self):
        # Regression: a straight-on red box (circularity pi/4, circle
        # ratio 2/pi) must never be classified as a sphere even though
        # its metric "diameter" (~0.30 m) matches the sphere window.
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.785, 0.637, 1.0, 0.30),
            "red_box",
        )

    def test_I_red_rotated_box_is_box(self):
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.76, 0.63, 0.80, 0.30),
            "red_box",
        )

    def test_I_red_circle_outside_diameter_window_is_box(self):
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.95, 0.90, 1.0, 0.50),
            "red_box",
        )

    def test_I_red_square_without_depth_is_box(self):
        self.assertEqual(
            perception_core.classify_object(
                "red", 0.785, 0.637, 1.0, None),
            "red_box",
        )

    def test_confidence_in_range(self):
        conf = perception_core.calculate_confidence(
            area=3000.0,
            circularity=0.9,
            aspect_ratio=1.0,
            position_valid=True,
            label="red_sphere",
        )
        self.assertTrue(0.0 <= conf <= 1.0)

    def test_confidence_survives_nan_features(self):
        conf = perception_core.calculate_confidence(
            area=float("nan"),
            circularity=float("nan"),
            aspect_ratio=float("nan"),
            position_valid=True,
            label="red_sphere",
        )
        self.assertTrue(math.isfinite(conf))
        self.assertTrue(0.0 <= conf <= 1.0)

    def test_confidence_penalizes_invalid_position(self):
        conf_valid = perception_core.calculate_confidence(
            area=3000.0,
            circularity=0.9,
            aspect_ratio=1.0,
            position_valid=True,
            label="red_sphere",
        )
        conf_invalid = perception_core.calculate_confidence(
            area=3000.0,
            circularity=0.9,
            aspect_ratio=1.0,
            position_valid=False,
            label="red_sphere",
        )
        self.assertLess(conf_invalid, conf_valid)


class DepthMathTest(unittest.TestCase):
    """Depth median window and back-projection."""

    def test_project_center_is_optical_axis(self):
        x, y, z = perception_core.project_to_3d(
            center_u=CX,
            center_v=CY,
            depth_value=2.0,
            depth_shape=(480, 640),
            rgb_shape=(480, 640),
            fx=FX,
            fy=FY,
            cx=CX,
            cy=CY,
            min_depth=MIN_DEPTH,
            max_depth=MAX_DEPTH,
        )
        self.assertAlmostEqual(x, 0.0, delta=0.005)
        self.assertAlmostEqual(y, 0.0, delta=0.005)
        self.assertAlmostEqual(z, 2.0, delta=1e-6)

    def test_project_offsets(self):
        x, y, z = perception_core.project_to_3d(
            center_u=CX + FX,
            center_v=CY + FY,
            depth_value=2.0,
            depth_shape=(480, 640),
            rgb_shape=(480, 640),
            fx=FX,
            fy=FY,
            cx=CX,
            cy=CY,
            min_depth=MIN_DEPTH,
            max_depth=MAX_DEPTH,
        )
        self.assertAlmostEqual(x, 2.0, delta=0.005)
        self.assertAlmostEqual(y, 2.0, delta=0.005)
        self.assertAlmostEqual(z, 2.0, delta=1e-6)

    def test_project_scaled_resolution_mapping(self):
        # RGB 1280x960, depth 640x480: pixel mapping is proportional.
        x, y, z = perception_core.project_to_3d(
            center_u=639.5,
            center_v=479.5,
            depth_value=2.0,
            depth_shape=(480, 640),
            rgb_shape=(960, 1280),
            fx=FX,
            fy=FY,
            cx=CX,
            cy=CY,
            min_depth=MIN_DEPTH,
            max_depth=MAX_DEPTH,
        )
        self.assertAlmostEqual(x, 0.0, delta=0.01)
        self.assertAlmostEqual(y, 0.0, delta=0.01)

    def test_project_rejects_invalid_depth(self):
        self.assertIsNone(
            perception_core.project_to_3d(
                center_u=CX,
                center_v=CY,
                depth_value=0.0,
                depth_shape=(480, 640),
                rgb_shape=(480, 640),
                fx=FX,
                fy=FY,
                cx=CX,
                cy=CY,
                min_depth=MIN_DEPTH,
                max_depth=MAX_DEPTH,
            )
        )
        self.assertIsNone(
            perception_core.project_to_3d(
                center_u=CX,
                center_v=CY,
                depth_value=9.0,
                depth_shape=(480, 640),
                rgb_shape=(480, 640),
                fx=FX,
                fy=FY,
                cx=CX,
                cy=CY,
                min_depth=MIN_DEPTH,
                max_depth=MAX_DEPTH,
            )
        )

    def test_robust_depth_median_ignores_invalid_pixels(self):
        depth = np.full((480, 640), 0.0, dtype=np.float32)  # invalid
        depth[235:245, 315:325] = 1.0
        depth[238:242, 318:322] = 3.0
        depth[240, 320] = float("nan")

        value = perception_core.robust_depth(
            depth_image_m=depth,
            center_u=320.0,
            center_v=240.0,
            bbox_width=84.0,
            bbox_height=84.0,
            rgb_shape=(480, 640),
            min_depth=MIN_DEPTH,
            max_depth=MAX_DEPTH,
        )
        self.assertAlmostEqual(value, 1.0, delta=0.01)

    def test_robust_depth_none_when_no_valid_pixels(self):
        depth = np.zeros((480, 640), dtype=np.float32)
        value = perception_core.robust_depth(
            depth_image_m=depth,
            center_u=320.0,
            center_v=240.0,
            bbox_width=84.0,
            bbox_height=84.0,
            rgb_shape=(480, 640),
            min_depth=MIN_DEPTH,
            max_depth=MAX_DEPTH,
        )
        self.assertIsNone(value)

    def test_estimate_diameter_of_known_box(self):
        # 0.30 m box at 2.0 m: ~83 px wide with fx ~= 554.26.
        diameter = perception_core.estimate_diameter_m(
            bbox_width=83.0,
            bbox_height=83.0,
            depth_value=2.0,
            fx=FX,
            fy=FY,
            rgb_shape=(480, 640),
            depth_shape=(480, 640),
        )
        self.assertAlmostEqual(diameter, 0.30, delta=0.02)

    def test_estimate_diameter_none_without_depth(self):
        self.assertIsNone(
            perception_core.estimate_diameter_m(
                bbox_width=83.0,
                bbox_height=83.0,
                depth_value=None,
                fx=FX,
                fy=FY,
                rgb_shape=(480, 640),
                depth_shape=(480, 640),
            )
        )


class DetectionPipelineTest(unittest.TestCase):
    """Synthetic-image detection pipeline."""

    def test_I_red_sphere_detected_with_position(self):
        bgr, depth_m = red_sphere_scene()
        detections = detect(bgr, depth_m)

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection["label"], "red_sphere")
        self.assertTrue(detection["position_valid"])
        self.assertIsNotNone(detection["position"])
        x, y, z = detection["position"]
        self.assertAlmostEqual(x, 0.0, delta=0.01)
        self.assertAlmostEqual(y, 0.0, delta=0.01)
        self.assertAlmostEqual(z, 2.0, delta=0.01)
        self.assertTrue(0.0 < detection["confidence"] <= 1.0)

    def test_I_red_box_detected(self):
        bgr, depth_m = red_box_scene()
        detections = detect(bgr, depth_m)

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection["label"], "red_box")
        self.assertTrue(detection["position_valid"])
        x, y, z = detection["position"]
        self.assertAlmostEqual(z, 2.0, delta=0.01)

    def test_I_green_sphere_detected(self):
        bgr, depth_m = green_sphere_scene()
        detections = detect(bgr, depth_m)

        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertEqual(detection["label"], "green_sphere")
        self.assertTrue(detection["position_valid"])

    def test_B_empty_scene_no_candidates(self):
        # No candidates does not make the module invalid: the node still
        # publishes RUNNING/valid=true with an empty candidate array.
        bgr, depth_m = blank_scene()
        detections = detect(bgr, depth_m)
        self.assertEqual(detections, [])

    def test_J_no_valid_depth_position_invalid(self):
        bgr, _ = red_sphere_scene()
        depth_m = np.zeros((480, 640), dtype=np.float32)

        detections = detect(bgr, depth_m)
        self.assertEqual(len(detections), 1)
        detection = detections[0]
        self.assertFalse(detection["position_valid"])
        self.assertIsNone(detection["position"])
        # Classification still runs on 2D shape alone.
        self.assertEqual(detection["label"], "red_sphere")

    def test_tiny_blob_filtered_by_area(self):
        bgr, depth_m = blank_scene()
        cv2.circle(bgr, (320, 240), 6, RED_BGR, thickness=-1)

        detections = detect(bgr, depth_m)
        self.assertEqual(detections, [])

    def test_detections_do_not_change_gate_validity(self):
        # Validity is owned by the observation gate, not by the number
        # of detections.
        gate = make_gate()
        bgr, depth_m = blank_scene()
        self.assertEqual(detect(bgr, depth_m), [])
        self.assertIsNone(gate.evaluate(10.0, 9.5))
        gate.accept(10.0, 9.5)
        self.assertTrue(gate.valid)


if __name__ == "__main__":
    unittest.main()
