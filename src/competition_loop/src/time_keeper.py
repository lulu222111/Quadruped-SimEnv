#!/usr/bin/env python3
"""time_keeper.py — competition timer using ROS time (/clock when use_sim_time=true).

Detects ROS time regression (sim clock jumping backwards) and exposes it
so the caller can enter a safe state instead of silently continuing with
invalid timing.
"""

from __future__ import annotations

import rospy
from typing import Optional


class TimeKeeper:
    """Monotonic competition timer.

    Uses rospy.Time.now() so that when /use_sim_time is true the timer
    follows Gazebo /clock rather than wall-clock time.
    """

    def __init__(self, max_time: float = 600.0):
        self._max_time = rospy.Duration.from_sec(float(max_time))
        self._start_time: Optional[rospy.Time] = None
        self._end_time: Optional[rospy.Time] = None
        self._running = False
        self._last_time: Optional[rospy.Time] = None
        self._regression_detected: bool = False

    @property
    def max_time(self) -> rospy.Duration:
        return self._max_time

    @property
    def elapsed(self) -> rospy.Duration:
        if self._start_time is None:
            return rospy.Duration(0)
        if self._end_time is not None:
            return self._end_time - self._start_time
        now = rospy.Time.now()
        if now < self._start_time:
            # Time regression — return zero elapsed rather than negative
            return rospy.Duration(0)
        return now - self._start_time

    @property
    def elapsed_seconds(self) -> float:
        return self.elapsed.to_sec()

    @property
    def remaining(self) -> rospy.Duration:
        remaining = self._max_time - self.elapsed
        if remaining.to_sec() < 0.0:
            return rospy.Duration(0)
        return remaining

    @property
    def remaining_seconds(self) -> float:
        return self.remaining.to_sec()

    @property
    def is_running(self) -> bool:
        return self._running

    @property
    def is_expired(self) -> bool:
        return self.elapsed >= self._max_time

    @property
    def regression_detected(self) -> bool:
        """True if a time regression was detected in this cycle."""
        return self._regression_detected

    def check_time_regression(self) -> bool:
        """Check if ROS time moved backwards since last call.

        Updates internal tracking. Returns True if regression detected.
        The caller should enter a safe state.
        """
        now = rospy.Time.now()
        if self._last_time is not None and now < self._last_time:
            rospy.logerr(
                "TimeKeeper: ROS time regression detected: "
                "%.3f -> %.3f (delta=%.3f s)",
                self._last_time.to_sec(),
                now.to_sec(),
                (now - self._last_time).to_sec(),
            )
            self._regression_detected = True
            return True
        self._last_time = now
        self._regression_detected = False
        return False

    def start(self) -> None:
        if self._running:
            rospy.logwarn("TimeKeeper already running; ignoring duplicate start.")
            return
        self._start_time = rospy.Time.now()
        self._last_time = self._start_time
        self._end_time = None
        self._running = True
        self._regression_detected = False
        rospy.loginfo("TimeKeeper started at t=%.3f s", self._start_time.to_sec())

    def stop(self) -> None:
        if not self._running:
            rospy.logwarn("TimeKeeper not running; ignoring stop.")
            return
        self._end_time = rospy.Time.now()
        self._running = False
        rospy.loginfo(
            "TimeKeeper stopped at t=%.3f s (elapsed %.3f s)",
            self._end_time.to_sec(),
            self.elapsed_seconds,
        )

    def reset(self) -> None:
        self._start_time = None
        self._end_time = None
        self._last_time = None
        self._running = False
        self._regression_detected = False

    def set_max_time(self, seconds: float) -> None:
        if self._running:
            rospy.logwarn("Changing max_time while timer is running.")
        self._max_time = rospy.Duration.from_sec(float(seconds))
