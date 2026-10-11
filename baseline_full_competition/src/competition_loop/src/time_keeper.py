#!/usr/bin/env python3
# =============================================================================
# 中文说明（模块级）：time_keeper.py —— 竞赛计时器（跟随 ROS 时间）
#
# 职责：以 rospy.Time.now()（use_sim_time=true 时为仿真 /clock）计算已用/剩余
#       时间，并检测 ROS 时间回退。
# 为什么要专门处理回退：多楼层流程会在电梯过渡/楼层切换时改动仿真状态，时钟
#       重置（回放或重启）会让直接相减得到负数 elapsed，使超时判定与日志失真。
#       因此 elapsed 在检测到回退时返回 0（安全归零），且回退检测"始终更新比较
#       基线"，避免用陈旧历史时间反复误判。
# 注意：max_time 由 launch arg 传入（MODE B 默认 1800 s；MODE A 正式预算 600 s）。
# =============================================================================
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

        ALWAYS updates _last_time to the current time, even when regression
        is detected.  This ensures the next call compares against the most
        recent time, not a stale historical baseline.

        Sequence 100→101→0.1→0.2:
          tick 100: no regression, _last = 100
          tick 101: no regression, _last = 101
          tick 0.1: regression detected, _last = 0.1  (UPDATED)
          tick 0.2: no regression (0.2 > 0.1), _last = 0.2
        """
        now = rospy.Time.now()
        regression = False
        if self._last_time is not None and now < self._last_time:
            rospy.logerr(
                "TimeKeeper: ROS time regression detected: "
                "%.3f -> %.3f (delta=%.3f s)",
                self._last_time.to_sec(),
                now.to_sec(),
                (now - self._last_time).to_sec(),
            )
            regression = True
            self._regression_detected = True
        else:
            self._regression_detected = False
        # ALWAYS update baseline — even on regression
        self._last_time = now
        return regression

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
