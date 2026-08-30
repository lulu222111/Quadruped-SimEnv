#!/usr/bin/env python3
"""ROS-independent bookkeeping for bounded active visual coverage scans."""

from __future__ import annotations

from dataclasses import dataclass
from math import ceil, hypot, isfinite, pi


SCAN_INACTIVE = "inactive"
SCAN_STARTED = "started"
SCAN_SKIPPED = "skipped"
SCAN_HEADING_REACHED = "heading_reached"
SCAN_DWELLING = "dwelling"
SCAN_NEXT_HEADING = "next_heading"
SCAN_RETRY_HEADING = "retry_heading"
SCAN_HEADING_TIMEOUT = "heading_timeout"
SCAN_COMPLETE = "complete"
SCAN_TIMEOUT = "timeout"


def normalize_angle(angle):
    """Normalize an angle to [-pi, pi), rejecting non-finite input."""
    if not isfinite(angle):
        raise ValueError("angle must be finite")
    # This form avoids accumulating an unbounded heading over repeated scans.
    from math import atan2, cos, sin

    return atan2(sin(angle), cos(angle))


def angular_distance(target, current):
    """Return the absolute shortest angular distance in radians."""
    return abs(normalize_angle(target - current))


@dataclass(frozen=True)
class ScanEvent:
    """A state transition or observation from :class:`VisualCoverageTracker`."""

    kind: str
    heading_index: int = -1
    heading_count: int = 0
    retry_count: int = 0
    target_yaw: float = 0.0


class VisualCoverageTracker:
    """Track one finite scan episode and the positions already scanned.

    The class does not publish commands or sleep.  The caller drives it from
    a timer/callback and publishes a rotation-only path when an event asks for
    a new heading.  A scan timeout always terminates the episode and records
    the position, so a bad heading cannot deadlock frontier exploration.
    """

    def __init__(
        self,
        enabled=True,
        yaw_step=pi / 4.0,
        yaw_tolerance=0.15,
        dwell=0.6,
        heading_timeout=4.0,
        scan_timeout=45.0,
        position_dedup_radius=1.0,
        max_heading_retries=1,
    ):
        self.enabled = bool(enabled)
        self.yaw_step = float(yaw_step)
        self.yaw_tolerance = float(yaw_tolerance)
        self.dwell = float(dwell)
        self.heading_timeout = float(heading_timeout)
        self.scan_timeout = float(scan_timeout)
        self.position_dedup_radius = float(position_dedup_radius)
        self.max_heading_retries = int(max_heading_retries)
        self._validate_parameters()
        self.reset()

    def _validate_parameters(self):
        finite = (
            self.yaw_step,
            self.yaw_tolerance,
            self.dwell,
            self.heading_timeout,
            self.scan_timeout,
            self.position_dedup_radius,
        )
        if not all(isfinite(value) for value in finite):
            raise ValueError("visual coverage parameters must be finite")
        if not 0.0 < self.yaw_step <= pi / 2.0:
            raise ValueError("yaw_step must be in (0, pi/2]")
        if not 0.0 < self.yaw_tolerance < pi:
            raise ValueError("yaw_tolerance must be in (0, pi)")
        if self.dwell < 0.0:
            raise ValueError("dwell must not be negative")
        if self.heading_timeout <= 0.0:
            raise ValueError("heading_timeout must be positive")
        if self.scan_timeout <= 0.0:
            raise ValueError("scan_timeout must be positive")
        if self.position_dedup_radius < 0.0:
            raise ValueError("position_dedup_radius must not be negative")
        if self.max_heading_retries < 0:
            raise ValueError("max_heading_retries must not be negative")

    def reset(self):
        """Reset active scan state and all position coverage for a new epoch."""
        self.scanned_positions = []
        self.active = False
        self.origin = None
        self.headings = ()
        self.heading_index = -1
        self.started_at = None
        self.commanded_at = None
        self.reached_at = None
        self.retry_count = 0

    @property
    def heading_count(self):
        return len(self.headings)

    @property
    def current_heading(self):
        if not self.active or self.heading_index < 0:
            return None
        return self.headings[self.heading_index]

    def position_is_covered(self, position):
        x, y = self._position(position)
        return any(
            hypot(x - scanned_x, y - scanned_y)
            <= self.position_dedup_radius
            for scanned_x, scanned_y in self.scanned_positions
        )

    def start(self, position, initial_yaw, now):
        """Start a scan unless disabled or this position is already covered."""
        x, y = self._position(position)
        initial_yaw = normalize_angle(float(initial_yaw))
        now = self._time(now)

        if not self.enabled:
            return ScanEvent(SCAN_SKIPPED)
        if self.active:
            raise RuntimeError("visual scan is already active")
        if self.position_is_covered((x, y)):
            return ScanEvent(SCAN_SKIPPED)

        heading_count = max(1, int(ceil((2.0 * pi) / self.yaw_step)))
        self.active = True
        self.origin = (x, y)
        self.headings = tuple(
            normalize_angle(initial_yaw + index * self.yaw_step)
            for index in range(heading_count)
        )
        self.heading_index = 0
        self.started_at = now
        self.commanded_at = now
        self.reached_at = None
        self.retry_count = 0
        return self._event(SCAN_STARTED)

    def update(self, current_yaw, now):
        """Advance the scan using the latest measured yaw and ROS time."""
        if not self.active:
            return ScanEvent(SCAN_INACTIVE)

        current_yaw = normalize_angle(float(current_yaw))
        now = self._time(now)

        if now - self.started_at > self.scan_timeout:
            event = self._event(SCAN_TIMEOUT)
            self._finish_episode(record_coverage=False)
            return event

        target_yaw = self.current_heading
        if angular_distance(target_yaw, current_yaw) <= self.yaw_tolerance:
            if self.reached_at is None:
                self.reached_at = now
                return self._event(SCAN_HEADING_REACHED)
            if now - self.reached_at < self.dwell:
                return self._event(SCAN_DWELLING)
            if self.heading_index + 1 >= self.heading_count:
                event = self._event(SCAN_COMPLETE)
                self._finish_episode(record_coverage=True)
                return event

            self.heading_index += 1
            self.commanded_at = now
            self.reached_at = None
            self.retry_count = 0
            return self._event(SCAN_NEXT_HEADING)

        self.reached_at = None
        if now - self.commanded_at > self.heading_timeout:
            if self.retry_count < self.max_heading_retries:
                self.retry_count += 1
                self.commanded_at = now
                return self._event(SCAN_RETRY_HEADING)

            # Missing any required heading makes the whole scan
            # incomplete.  Do not skip the failed direction and later record
            # the position as fully covered.
            event = self._event(SCAN_HEADING_TIMEOUT)
            self._finish_episode(record_coverage=False)
            return event

        return self._event(SCAN_DWELLING)

    def _finish_episode(self, record_coverage=False):
        if (
            record_coverage
            and self.origin is not None
            and not self.position_is_covered(self.origin)
        ):
            self.scanned_positions.append(self.origin)
        self.active = False
        self.heading_index = -1
        self.headings = ()
        self.started_at = None
        self.commanded_at = None
        self.reached_at = None
        self.retry_count = 0

    def _event(self, kind):
        return ScanEvent(
            kind=kind,
            heading_index=self.heading_index,
            heading_count=self.heading_count,
            retry_count=self.retry_count,
            target_yaw=(
                self.current_heading
                if self.current_heading is not None
                else 0.0
            ),
        )

    @staticmethod
    def _position(position):
        try:
            x, y = float(position[0]), float(position[1])
        except (IndexError, TypeError, ValueError, OverflowError) as error:
            raise ValueError("position must contain two finite values") from error
        if not isfinite(x) or not isfinite(y):
            raise ValueError("position must contain two finite values")
        return x, y

    @staticmethod
    def _time(now):
        try:
            value = float(now)
        except (TypeError, ValueError, OverflowError) as error:
            raise ValueError("time must be finite") from error
        if not isfinite(value):
            raise ValueError("time must be finite")
        return value
