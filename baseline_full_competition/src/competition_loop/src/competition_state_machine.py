#!/usr/bin/env python3
"""competition_state_machine.py — competition FSM.

States:
  WAITING   — environment validation, waiting for dependencies
  RUNNING   — exploration active, timer counting
  RETURNING — return path is active
  FINISHED  — max_time reached or finish service called
  ERROR     — unrecoverable fault (e.g. forbidden topic detected in comp mode)

中文说明：
    顶层比赛状态机，负责“什么时候允许运动”。RUNNING（探索）与
    RETURNING（返航）之外的任何阶段都会触发急停，因此本状态机是运动使能的
    唯一权威来源。

    多楼层流程位于 RUNNING 内部（见 floor_mission_fsm.py）：
    RUNNING 内先完成各楼层探索，全部完成后进入 RETURNING 返航，
    最后 FINISHED 或 ERROR。

    转移规则是白名单式的（见 _allowed）：非法转移被拒绝并记 warning，
    只有 ERROR 允许从任意状态强制进入。
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
    """Thread-safe competition finite state machine.

    中文：带锁的线程安全 FSM。所有读写都在锁内完成；ROS timer、service
    回调与 worker 线程可能并发访问，因此不暴露未加锁的可变字段。
    """

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
        """Enable motion control while exploring or returning.

        中文：只有 RUNNING / RETURNING 允许下发运动指令。
        """
        with self._lock:
            return self._phase in (
                CompetitionPhase.RUNNING,
                CompetitionPhase.RETURNING,
            )

    def should_emergency_stop(self) -> bool:
        """Emergency stop is active outside active motion phases.

        中文：其余阶段（WAITING / FINISHED / ERROR）一律急停，防止任务
        结束或校验失败后机器人仍在移动。
        """
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

        中文：白名单外的转移一律拒绝（返回 False 并记 warning），
        避免误调用把状态机推进到非法阶段。
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
        # 中文：ERROR 是唯一的“强制”目标；FINISHED 为终态，不允许再转出。
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
