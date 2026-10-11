#!/usr/bin/env python3
"""Pure ALIGN -> CROSS -> SETTLE threshold controller.

All threshold geometry must be supplied explicitly by an authorized external
source.  This module contains no scene-specific crossing distance.

The default crossing speed is deliberately conservative and aligned with the
current precision-mode envelope.  It is a default, not a hard maximum:
higher speeds may be supplied explicitly after dynamic qualification.

中文说明：
    纯逻辑的“门槛通过”控制器：ALIGN（对准）-> CROSS（穿越）-> SETTLE（稳定）。
    用于电梯门/门槛这类受约束的局部运动 —— 与普通全局路径跟随不同，这里要求
    严格的对准容差与低速穿越，且几何量必须由授权外部来源显式提供，
    本模块不含任何场景相关的穿越距离常量。

    默认穿越速度偏保守，属于生产默认值而非物理上限；如需更高速度，
    必须经过动力学验证后显式传入。
"""

from __future__ import annotations

from dataclasses import dataclass
import math


def clamp(value, low, high):
    return max(low, min(high, value))


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class ThresholdStage:
    # 中文：门槛通过阶段。IDLE/COMPLETE/FAILED 为终态或静止态，
    # ALIGN -> CROSS -> SETTLE 为不可跳过的运动序列。
    IDLE = "IDLE"
    ALIGN = "ALIGN"
    CROSS = "CROSS"
    SETTLE = "SETTLE"
    COMPLETE = "COMPLETE"
    FAILED = "FAILED"


@dataclass
class ThresholdConfig:
    alignment_tolerance: float = math.radians(3.0)
    alignment_kp: float = 1.8
    alignment_max_angular: float = 0.55
    alignment_timeout: float = 4.0

    # Conservative production default only.  Do not interpret this as a
    # permanent physical limit; higher values require dynamic qualification.
    # 中文：保守的生产默认穿越速度；不是永久物理上限，提速需动力学验证。
    crossing_speed: float = 0.20

    yaw_lock_kp: float = 1.3
    yaw_lock_max_angular: float = 0.35
    crossing_timeout: float = 8.0

    settle_speed: float = 0.20
    settle_kp: float = 1.5
    settle_max_angular: float = 0.35
    goal_tolerance: float = 0.10
    settle_timeout: float = 8.0

    def validate(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be positive and finite"
                )


@dataclass(frozen=True)
class ThresholdCommand:
    linear_x: float
    angular_z: float
    stage: str
    done: bool
    failed: bool
    detail: str


class ThresholdCrossingController:
    def __init__(self, config=None):
        self.config = config or ThresholdConfig()
        self.config.validate()

        self.stage = ThresholdStage.IDLE
        self.heading = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Geometry is mission/transit-specific and must never have a hidden
        # scene default.
        self.crossing_distance = None

        self.stage_entered_at = 0.0
        self.cross_start_x = 0.0
        self.cross_start_y = 0.0
        self.failure_reason = ""

    def begin(
        self,
        heading,
        goal_x,
        goal_y,
        now,
        crossing_distance=None,
    ):
        values = (heading, goal_x, goal_y, now)
        if not all(
            math.isfinite(float(value))
            for value in values
        ):
            raise ValueError(
                "threshold begin inputs must be finite"
            )

        # Never reuse geometry from an earlier transit.
        self.crossing_distance = None
        self.failure_reason = ""

        if crossing_distance is None:
            return self._fail(
                (
                    "authorized crossing distance is missing; "
                    "threshold transit cannot arm"
                ),
                float(now),
            )

        try:
            crossing_distance = float(crossing_distance)
        except (TypeError, ValueError):
            return self._fail(
                (
                    "authorized crossing distance must be "
                    "positive and finite"
                ),
                float(now),
            )

        if (
            not math.isfinite(crossing_distance)
            or crossing_distance <= 0.0
        ):
            return self._fail(
                (
                    "authorized crossing distance must be "
                    "positive and finite"
                ),
                float(now),
            )

        self.heading = normalize_angle(float(heading))
        self.goal_x = float(goal_x)
        self.goal_y = float(goal_y)
        self.crossing_distance = crossing_distance

        self.stage = ThresholdStage.ALIGN
        self.stage_entered_at = float(now)
        self.failure_reason = ""
        return None

    def update(self, x, y, yaw, now):
        values = (x, y, yaw, now)
        if not all(
            math.isfinite(float(value))
            for value in values
        ):
            return self._fail(
                "non-finite localization input",
                float(now),
            )

        x = float(x)
        y = float(y)
        yaw = normalize_angle(float(yaw))
        now = float(now)

        if self.stage == ThresholdStage.IDLE:
            return self._cmd(
                0.0,
                0.0,
                "controller not armed",
            )

        if self.stage == ThresholdStage.COMPLETE:
            return ThresholdCommand(
                0.0,
                0.0,
                self.stage,
                True,
                False,
                "threshold transit complete",
            )

        if self.stage == ThresholdStage.FAILED:
            return ThresholdCommand(
                0.0,
                0.0,
                self.stage,
                False,
                True,
                self.failure_reason,
            )

        # Defense in depth: even if state is corrupted or changed by a future
        # caller, CROSS must never become reachable without explicit geometry.
        if (
            self.stage in (
                ThresholdStage.ALIGN,
                ThresholdStage.CROSS,
            )
            and self.crossing_distance is None
        ):
            return self._fail(
                "authorized crossing geometry missing",
                now,
            )

        heading_error = normalize_angle(
            self.heading - yaw
        )

        if self.stage == ThresholdStage.ALIGN:
            if (
                now - self.stage_entered_at
                > self.config.alignment_timeout
            ):
                return self._fail(
                    "alignment timeout",
                    now,
                )

            if (
                abs(heading_error)
                <= self.config.alignment_tolerance
            ):
                self.stage = ThresholdStage.CROSS
                self.stage_entered_at = now
                self.cross_start_x = x
                self.cross_start_y = y
                return self._cmd(
                    0.0,
                    0.0,
                    "alignment complete; crossing armed",
                )

            angular = clamp(
                self.config.alignment_kp
                * heading_error,
                -self.config.alignment_max_angular,
                self.config.alignment_max_angular,
            )

            return self._cmd(
                0.0,
                angular,
                "aligning to authorized transit heading",
            )

        if self.stage == ThresholdStage.CROSS:
            if (
                now - self.stage_entered_at
                > self.config.crossing_timeout
            ):
                return self._fail(
                    "threshold crossing timeout",
                    now,
                )

            progress = (
                (x - self.cross_start_x)
                * math.cos(self.heading)
                + (y - self.cross_start_y)
                * math.sin(self.heading)
            )

            if progress >= self.crossing_distance:
                self.stage = ThresholdStage.SETTLE
                self.stage_entered_at = now
                return self._cmd(
                    0.0,
                    0.0,
                    "threshold cleared; entering settle",
                )

            angular = clamp(
                self.config.yaw_lock_kp
                * heading_error,
                -self.config.yaw_lock_max_angular,
                self.config.yaw_lock_max_angular,
            )

            return self._cmd(
                self.config.crossing_speed,
                angular,
                (
                    "crossing threshold; "
                    f"progress={progress:.3f}m"
                ),
            )

        if self.stage == ThresholdStage.SETTLE:
            if (
                now - self.stage_entered_at
                > self.config.settle_timeout
            ):
                return self._fail(
                    "post-threshold settle timeout",
                    now,
                )

            distance = math.hypot(
                self.goal_x - x,
                self.goal_y - y,
            )

            if distance <= self.config.goal_tolerance:
                self.stage = ThresholdStage.COMPLETE
                self.stage_entered_at = now
                return ThresholdCommand(
                    0.0,
                    0.0,
                    self.stage,
                    True,
                    False,
                    "settle goal reached",
                )

            angular = clamp(
                self.config.settle_kp
                * heading_error,
                -self.config.settle_max_angular,
                self.config.settle_max_angular,
            )

            linear = (
                0.0
                if abs(heading_error) > 0.8
                else self.config.settle_speed
            )

            return self._cmd(
                linear,
                angular,
                "low-speed post-threshold settle",
            )

        return self._fail(
            "unhandled threshold stage",
            now,
        )

    def _fail(self, reason, now):
        self.stage = ThresholdStage.FAILED
        self.stage_entered_at = float(now)
        self.failure_reason = str(reason)

        return ThresholdCommand(
            0.0,
            0.0,
            self.stage,
            False,
            True,
            self.failure_reason,
        )

    def _cmd(self, linear, angular, detail):
        return ThresholdCommand(
            float(linear),
            float(angular),
            self.stage,
            False,
            False,
            str(detail),
        )
