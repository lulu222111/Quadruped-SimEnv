#!/usr/bin/env python3
# =============================================================================
# 模块说明（中文）：competition_state_machine.py —— 竞赛有限状态机
#
# 职责：以线程安全的方式维护 5 个相位（WAITING / RUNNING / RETURNING /
#       FINISHED / ERROR），集中裁决相位转移是否合法，并导出 ROS 消息。
# 为什么集中：相位决定运动使能与急停，若允许各处随意赋值，安全性无法审计。
#       因此合法转移写成显式转移表（_allowed，行 153-167），非法转移只告警拒绝；
#       ERROR 恒为合法目标，用于安全兜底强制进入错误态。
# 关键约定：detail 字段用于补充说明文本，可以随时更新，但它不能改变 phase；
#       should_control_be_enabled / should_emergency_stop 是网关判定运动许可的
#       唯一依据，任何"凭相位猜运动"的写法都应改为调用这两个属性。
# =============================================================================
"""competition_state_machine.py — competition FSM.

States:
  WAITING   — environment validation, waiting for dependencies
  RUNNING   — exploration active, timer counting
  RETURNING — return path is active
  FINISHED  — max_time reached or finish service called
  ERROR     — unrecoverable fault (e.g. forbidden topic detected in comp mode)
"""

from __future__ import annotations

import threading
from typing import Optional

import rospy
from competition_interfaces.msg import CompetitionState as CompetitionStateMsg


class CompetitionPhase:
    WAITING = CompetitionStateMsg.WAITING     # 0
    RUNNING = CompetitionStateMsg.RUNNING     # 1
    RETURNING = CompetitionStateMsg.RETURNING # 2
    FINISHED = CompetitionStateMsg.FINISHED   # 3
    ERROR = CompetitionStateMsg.ERROR         # 4

    _LABELS = {
        WAITING: "WAITING",
        RUNNING: "RUNNING",
        RETURNING: "RETURNING",
        FINISHED: "FINISHED",
        ERROR: "ERROR",
    }

    @classmethod
    def label(cls, phase: int) -> str:
        return cls._LABELS.get(phase, f"UNKNOWN({phase})")


class CompetitionStateMachine:
    """Thread-safe competition finite state machine."""

    def __init__(self):
        self._lock = threading.Lock()
        self._phase = CompetitionPhase.WAITING
        self._detail = "Initialised — waiting for environment validation."

    # ------------------------------------------------------------------
    # Phase access
    # ------------------------------------------------------------------

    @property
    def phase(self) -> int:
        with self._lock:
            return self._phase

    @property
    def detail(self) -> str:
        with self._lock:
            return self._detail

    @property
    def is_terminal(self) -> bool:
        with self._lock:
            return self._phase in (CompetitionPhase.FINISHED, CompetitionPhase.ERROR)

    @property
    def is_running(self) -> bool:
        with self._lock:
            return self._phase == CompetitionPhase.RUNNING

    def should_control_be_enabled(self) -> bool:
        """Enable motion control while exploring or returning."""
        with self._lock:
            return self._phase in (
                CompetitionPhase.RUNNING,
                CompetitionPhase.RETURNING,
            )

    def should_emergency_stop(self) -> bool:
        """Emergency stop is active outside active motion phases."""
        with self._lock:
            return self._phase not in (
                CompetitionPhase.RUNNING,
                CompetitionPhase.RETURNING,
            )

    # ------------------------------------------------------------------
    # Transitions
    # ------------------------------------------------------------------

    def transition_to(self, target_phase: int, detail: str = "") -> bool:
        """Attempt a phase transition. Returns True on success.

        Allowed transitions:
          WAITING  -> RUNNING
          WAITING  -> ERROR
          RUNNING  -> FINISHED
          RUNNING  -> ERROR
          RUNNING  -> RETURNING
          RETURNING -> FINISHED
          Any      -> ERROR       (force)
        """
        with self._lock:
            current = self._phase
            allowed = self._allowed(current, target_phase)
            if not allowed:
                rospy.logwarn(
                    "Blocked transition: %s -> %s",
                    CompetitionPhase.label(current),
                    CompetitionPhase.label(target_phase),
                )
                return False

            old_label = CompetitionPhase.label(current)
            new_label = CompetitionPhase.label(target_phase)
            self._phase = target_phase
            self._detail = detail or self._detail
            rospy.loginfo("FSM: %s -> %s (%s)", old_label, new_label, self._detail)
            return True

    def force_error(self, detail: str = "") -> None:
        with self._lock:
            self._phase = CompetitionPhase.ERROR
            self._detail = detail or "Forced to ERROR"
            rospy.logerr("FSM forced to ERROR: %s", self._detail)

    def set_detail(self, detail: str) -> None:
        """Update the mission detail without changing the public phase enum."""
        with self._lock:
            self._detail = str(detail)
            rospy.loginfo("FSM detail: %s", self._detail)

    # ------------------------------------------------------------------
    # ROS message
    # ------------------------------------------------------------------

    def to_msg(self, elapsed_seconds: float = 0.0, detected_count: int = 0) -> CompetitionStateMsg:
        msg = CompetitionStateMsg()
        msg.header.stamp = rospy.Time.now()
        with self._lock:
            msg.state = self._phase
            msg.detail = self._detail
        msg.elapsed_time = rospy.Duration.from_sec(elapsed_seconds)
        msg.detected_count = detected_count
        return msg

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    @staticmethod
    def _allowed(current: int, target: int) -> bool:
        # ERROR is always allowed as target (force)
        if target == CompetitionPhase.ERROR:
            return True
        mapping = {
            CompetitionPhase.WAITING: (CompetitionPhase.RUNNING,),
            CompetitionPhase.RUNNING: (
                CompetitionPhase.FINISHED,
                CompetitionPhase.RETURNING,
            ),
            CompetitionPhase.RETURNING: (CompetitionPhase.FINISHED,),
            CompetitionPhase.FINISHED: (),
        }
        return target in mapping.get(current, ())
