"""Pure state machine for the C2R7 test-only straight segment."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Tuple


class RuntimeGateError(RuntimeError):
    """The process was not explicitly authorized as a safe test-only source."""


class SegmentState(Enum):
    WAITING = "waiting"
    ACTIVE = "active"
    SUCCEEDED = "succeeded"
    BLOCKED = "blocked"
    TIMEOUT = "timeout"
    FAILED = "failed"


@dataclass(frozen=True)
class SegmentConfig:
    waypoint_distances: Tuple[float, ...] = (0.25, 0.50, 0.75)
    arrival_distance: float = 0.55
    timeout: float = 8.0
    max_yaw_deviation: float = 0.15
    max_cross_track: float = 0.15
    localization_timeout: float = 1.0
    max_future_stamp_skew: float = 0.2

    def __post_init__(self):
        numeric = (
            self.arrival_distance,
            self.timeout,
            self.max_yaw_deviation,
            self.max_cross_track,
            self.localization_timeout,
            self.max_future_stamp_skew,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError("segment configuration must be finite")
        if any(value <= 0.0 for value in numeric[:-1]):
            raise ValueError("segment limits must be positive")
        if self.max_future_stamp_skew < 0.0:
            raise ValueError("max_future_stamp_skew must be non-negative")
        if not self.waypoint_distances:
            raise ValueError("at least one waypoint distance is required")
        previous = 0.0
        for distance in self.waypoint_distances:
            if not math.isfinite(distance) or distance <= previous:
                raise ValueError(
                    "waypoint distances must be finite, positive and increasing"
                )
            previous = distance
        if self.arrival_distance > self.waypoint_distances[-1]:
            raise ValueError("arrival distance exceeds the final waypoint")


@dataclass(frozen=True)
class LocalizationSample:
    x: float
    y: float
    yaw: float
    stamp: float
    frame_id: str = "world"
    child_frame_id: str = "base"


@dataclass(frozen=True)
class Waypoint:
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class Transition:
    state: SegmentState
    detail: str
    publish_path: bool = False
    cancel_path: bool = False


def normalize_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def build_waypoints(x, y, yaw, distances=(0.25, 0.50, 0.75)):
    values = (x, y, yaw) + tuple(distances)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("waypoint inputs must be finite")
    direction_x = math.cos(yaw)
    direction_y = math.sin(yaw)
    return tuple(
        Waypoint(
            x=x + distance * direction_x,
            y=y + distance * direction_y,
            yaw=yaw,
        )
        for distance in distances
    )


def validate_runtime_gate(
        competition_mode, test_only_acknowledged, enable_ground_truth):
    if competition_mode is not False:
        raise RuntimeGateError("competition_mode must be explicitly false")
    if test_only_acknowledged is not True:
        raise RuntimeGateError("test_only_acknowledged must be explicitly true")
    if not isinstance(enable_ground_truth, str):
        raise RuntimeGateError("ENABLE_GROUND_TRUTH must be explicitly false")
    if enable_ground_truth.strip().lower() != "false":
        raise RuntimeGateError("ENABLE_GROUND_TRUTH must be explicitly false")


class SegmentController:
    """Freeze one legal localization pose and bound a straight segment."""

    def __init__(self, config=None):
        self.config = config or SegmentConfig()
        self.state = SegmentState.WAITING
        self.detail = "waiting for fresh valid localization"
        self.origin: Optional[Tuple[float, float, float]] = None
        self.waypoints: Tuple[Waypoint, ...] = ()
        self._start_time: Optional[float] = None
        self._last_now: Optional[float] = None
        self._last_sample_stamp: Optional[float] = None

    def observe(
            self, sample, now, status_valid=True, status_stamp=None):
        if self._terminal:
            return self._current()
        time_failure = self._validate_time(now)
        if time_failure is not None:
            return time_failure
        status_failure = self._validate_status(
            now, status_valid, status_stamp)
        if status_failure is not None:
            return status_failure
        sample_failure = self._validate_sample(sample, now)
        if sample_failure is not None:
            return sample_failure

        self._last_sample_stamp = sample.stamp
        if self.state is SegmentState.WAITING:
            self.origin = (sample.x, sample.y, sample.yaw)
            self.waypoints = build_waypoints(
                sample.x,
                sample.y,
                sample.yaw,
                self.config.waypoint_distances,
            )
            self._start_time = now
            self.state = SegmentState.ACTIVE
            self.detail = "straight segment active"
            return Transition(
                self.state,
                self.detail,
                publish_path=True,
            )

        timeout = self._motion_timeout(now)
        if timeout is not None:
            return timeout
        origin_x, origin_y, origin_yaw = self.origin
        delta_x = sample.x - origin_x
        delta_y = sample.y - origin_y
        along_track = (
            math.cos(origin_yaw) * delta_x + math.sin(origin_yaw) * delta_y
        )
        cross_track = (
            -math.sin(origin_yaw) * delta_x + math.cos(origin_yaw) * delta_y
        )
        yaw_deviation = abs(normalize_angle(sample.yaw - origin_yaw))

        if yaw_deviation > self.config.max_yaw_deviation:
            return self._finish(
                SegmentState.BLOCKED,
                "yaw deviation {:.6f} exceeds {:.6f}".format(
                    yaw_deviation, self.config.max_yaw_deviation),
            )
        if abs(cross_track) > self.config.max_cross_track:
            return self._finish(
                SegmentState.BLOCKED,
                "cross-track {:.6f} exceeds {:.6f}".format(
                    cross_track, self.config.max_cross_track),
            )
        if along_track >= self.config.arrival_distance:
            return self._finish(
                SegmentState.SUCCEEDED,
                "distance reached: along-track {:.6f} m".format(along_track),
            )
        return self._current()

    def tick(self, now, status_valid=True, status_stamp=None):
        if self._terminal:
            return self._current()
        time_failure = self._validate_time(now)
        if time_failure is not None:
            return time_failure
        status_failure = self._validate_status(
            now, status_valid, status_stamp)
        if status_failure is not None:
            return status_failure
        if self.state is SegmentState.ACTIVE:
            if (
                    self._last_sample_stamp is None
                    or now - self._last_sample_stamp
                    > self.config.localization_timeout):
                return self._finish(
                    SegmentState.TIMEOUT,
                    "localization odometry stale",
                )
            timeout = self._motion_timeout(now)
            if timeout is not None:
                return timeout
        return self._current()

    @property
    def _terminal(self):
        return self.state in (
            SegmentState.SUCCEEDED,
            SegmentState.BLOCKED,
            SegmentState.TIMEOUT,
            SegmentState.FAILED,
        )

    def _validate_time(self, now):
        if not math.isfinite(now) or now <= 0.0:
            return self._finish(SegmentState.FAILED, "ROS time is invalid")
        if self._last_now is not None and now < self._last_now:
            return self._finish(SegmentState.TIMEOUT, "ROS time moved backwards")
        self._last_now = now
        return None

    def _validate_status(self, now, status_valid, status_stamp):
        if status_valid is not True:
            return self._finish(
                SegmentState.BLOCKED,
                "localization status is invalid",
            )
        if status_stamp is None or not math.isfinite(status_stamp):
            return self._finish(
                SegmentState.FAILED,
                "localization status stamp is invalid",
            )
        return self._stamp_failure(now, status_stamp, "localization status")

    def _validate_sample(self, sample, now):
        if not isinstance(sample, LocalizationSample):
            return self._finish(
                SegmentState.FAILED,
                "localization sample has the wrong type",
            )
        if sample.frame_id != "world" or sample.child_frame_id != "base":
            return self._finish(
                SegmentState.FAILED,
                "localization odometry must use world/base",
            )
        if not all(math.isfinite(value) for value in (
                sample.x, sample.y, sample.yaw, sample.stamp)):
            return self._finish(
                SegmentState.FAILED,
                "localization sample contains NaN or Inf",
            )
        return self._stamp_failure(now, sample.stamp, "localization odometry")

    def _stamp_failure(self, now, stamp, label):
        if stamp <= 0.0:
            return self._finish(SegmentState.FAILED, label + " stamp is zero")
        if stamp - now > self.config.max_future_stamp_skew:
            return self._finish(SegmentState.FAILED, label + " stamp is in the future")
        if now - stamp > self.config.localization_timeout:
            return self._finish(SegmentState.TIMEOUT, label + " is stale")
        return None

    def _motion_timeout(self, now):
        if self._start_time is not None and now - self._start_time >= self.config.timeout:
            return self._finish(
                SegmentState.TIMEOUT,
                "segment motion timeout",
            )
        return None

    def _finish(self, state, detail):
        was_active = self.state is SegmentState.ACTIVE
        self.state = state
        self.detail = detail
        return Transition(
            state,
            detail,
            cancel_path=was_active,
        )

    def _current(self):
        return Transition(self.state, self.detail)
