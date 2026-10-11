#!/usr/bin/env python3
"""Pure fail-closed FSM for the frozen multi-floor elevator V1 contract.

中文说明：
    电梯执行器的纯逻辑状态机（ElevatorStatus V1 契约），无 ROS、无仿真数据依赖。
    负责校验 mission 身份（mission_id/elevator_id/来源与目标楼层）、维护阶段
    迁移与各阶段超时。ROS 适配层（door_elevator_executor_node）把官方服务证据
    喂给本状态机，本状态机只做“是否可推进”的判定。

    fail-closed 原则：
        任何身份不匹配、超时或证据矛盾都进入 FAILED（valid=false），
        不静默继续、不猜测楼层。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Iterable, Optional, Tuple


class ElevatorPhase:
    # 中文：电梯执行阶段。1..9 为 ACTIVE（任务进行中），
    # MOTION 子集（APPROACHING/ENTERING/EXITING）表示机器人正在受约束局部运动，
    # 此时由执行器独占运动控制权。10/11 为终态。
    IDLE = 0
    APPROACHING = 1
    AT_APPROACH = 2
    WAITING_ELEVATOR = 3
    READY_TO_ENTER = 4
    ENTERING = 5
    IN_CABIN = 6
    RIDING = 7
    READY_TO_EXIT = 8
    EXITING = 9
    COMPLETE = 10
    FAILED = 11

    ACTIVE = {
        APPROACHING,
        AT_APPROACH,
        WAITING_ELEVATOR,
        READY_TO_ENTER,
        ENTERING,
        IN_CABIN,
        RIDING,
        READY_TO_EXIT,
        EXITING,
    }
    MOTION = {APPROACHING, ENTERING, EXITING}


@dataclass(frozen=True)
class Mission:
    mission_id: int
    elevator_id: str
    source_floor: int
    target_floor: int
    stamp: float = 0.0

    # 中文：mission 身份指纹，用于拒绝重复/过期的 ElevatorMission。
    @property
    def identity(self) -> Tuple[int, str, int, int]:
        return (
            int(self.mission_id),
            str(self.elevator_id),
            int(self.source_floor),
            int(self.target_floor),
        )


@dataclass
class ExecutorTimeouts:
    # 中文：各阶段超时（秒）。默认值面向仿真；真实门动作/乘梯时间更长时，
    # 由 launch/param 显式上调（不改这里的默认语义）。
    approach: float = 90.0
    ready_wait: float = 60.0
    enter: float = 20.0
    in_cabin: float = 30.0
    ride: float = 90.0
    exit: float = 20.0

    def validate(self) -> None:
        for name, value in vars(self).items():
            if (
                not isinstance(value, (int, float))
                or not math.isfinite(float(value))
                or float(value) <= 0.0
            ):
                raise ValueError(
                    "{} timeout must be positive and finite".format(name)
                )


class ElevatorExecutorCore:
    """Mission-identity and phase FSM independent of ROS and simulator data.

    ``elevator_catalog`` is optional for pure reuse, but the production ROS
    adapter supplies it from the authorized semantic fields of
    team_scene_info.json:
        elevator id -> served floor indices

    No pose, heading, world/SDF, or simulator-truth field is consumed here.

    中文：elevator_catalog 是可选的“电梯 id -> 服务楼层集合”语义信息，
    由生产 ROS 适配层从公开 team_scene_info 提供，用于校验 mission 合法性。
    本类不消费任何位姿、朝向、world/SDF 或仿真真值字段。
    """

    def __init__(
        self,
        timeouts: Optional[ExecutorTimeouts] = None,
        elevator_catalog: Optional[Dict[str, Iterable[int]]] = None,
    ):
        self.timeouts = timeouts or ExecutorTimeouts()
        self.timeouts.validate()
        self.elevator_catalog = self._normalize_catalog(elevator_catalog)

        self.phase = ElevatorPhase.IDLE
        self.valid = True
        self.detail = "idle; no active elevator mission"
        self.mission: Optional[Mission] = None
        self.phase_entered_at = 0.0
        self._last_mission_id: Optional[int] = None

    @staticmethod
    def _normalize_catalog(catalog):
        if catalog is None:
            return None
        if not isinstance(catalog, dict):
            raise ValueError("elevator catalog must be a dictionary")

        normalized = {}
        for raw_id, raw_floors in catalog.items():
            elevator_id = str(raw_id).strip()
            if not elevator_id:
                raise ValueError("elevator catalog contains an empty id")
            try:
                floor_values = list(raw_floors)
            except TypeError as exc:
                raise ValueError(
                    "served_floors for {} must be iterable".format(elevator_id)
                ) from exc

            floors = set()
            for floor in floor_values:
                if isinstance(floor, bool) or not isinstance(floor, int):
                    raise ValueError(
                        "served_floors for {} must contain integers".format(
                            elevator_id
                        )
                    )
                if floor < 0:
                    raise ValueError(
                        "served_floors for {} must be non-negative".format(
                            elevator_id
                        )
                    )
                floors.add(int(floor))

            if not floors:
                raise ValueError(
                    "elevator {} has no served floors".format(elevator_id)
                )
            normalized[elevator_id] = frozenset(floors)
        return normalized

    def validate_mission(self, mission: Mission) -> Optional[str]:
        if isinstance(mission.mission_id, bool) or not isinstance(
            mission.mission_id, int
        ):
            return "mission_id must be an integer"
        if mission.mission_id < 0 or mission.mission_id > 0xFFFFFFFF:
            return "mission_id outside uint32 range"

        elevator_id = str(mission.elevator_id).strip()
        if not elevator_id:
            return "elevator_id must be non-empty"

        for label, value in (
            ("source_floor", mission.source_floor),
            ("target_floor", mission.target_floor),
        ):
            if isinstance(value, bool) or not isinstance(value, int):
                return "{} must be an integer".format(label)
            if value < 0:
                return "{} must be non-negative".format(label)

        if mission.source_floor == mission.target_floor:
            return "source_floor and target_floor must differ"

        if self.elevator_catalog is not None:
            served = self.elevator_catalog.get(elevator_id)
            if served is None:
                return "unknown elevator_id '{}'".format(elevator_id)
            if mission.source_floor not in served:
                return (
                    "source_floor {} is not served by elevator '{}'".format(
                        mission.source_floor,
                        elevator_id,
                    )
                )
            if mission.target_floor not in served:
                return (
                    "target_floor {} is not served by elevator '{}'".format(
                        mission.target_floor,
                        elevator_id,
                    )
                )

        return None

    def accept_mission(self, mission: Mission, now: float):
        error = self.validate_mission(mission)
        if error:
            if self.phase in ElevatorPhase.ACTIVE:
                self.fail("invalid elevator mission: " + error, now)
            return False, error

        if self._last_mission_id is not None:
            if mission.mission_id < self._last_mission_id:
                return False, "stale mission_id {}".format(mission.mission_id)

            if mission.mission_id == self._last_mission_id:
                if (
                    self.mission is not None
                    and mission.identity == self.mission.identity
                ):
                    return False, "duplicate mission"
                self.fail(
                    "mission_id {} payload changed".format(mission.mission_id),
                    now,
                )
                return False, "mission identity conflict"

            if self.phase in ElevatorPhase.ACTIVE:
                self.fail(
                    "new mission_id {} arrived while mission_id {} is active".format(
                        mission.mission_id,
                        self._last_mission_id,
                    ),
                    now,
                )
                return False, "active mission superseded"

        self.mission = Mission(
            int(mission.mission_id),
            str(mission.elevator_id).strip(),
            int(mission.source_floor),
            int(mission.target_floor),
            float(mission.stamp),
        )
        self._last_mission_id = self.mission.mission_id
        self._transition(
            ElevatorPhase.APPROACHING,
            now,
            "mission accepted; waiting for a fresh Planning approach path",
            True,
        )
        return True, "accepted"

    def mark_approach_complete(self, now: float) -> None:
        self._require(ElevatorPhase.APPROACHING)
        self._transition(
            ElevatorPhase.AT_APPROACH,
            now,
            (
                "approach path completed; holding for formal "
                "elevator/door readiness evidence"
            ),
            True,
        )

    def mark_waiting_elevator(
        self,
        evidence_valid: bool,
        detail: str,
        now: float,
    ) -> None:
        self._require(ElevatorPhase.AT_APPROACH)
        self._require_evidence(
            evidence_valid,
            detail,
            now,
            "WAITING_ELEVATOR",
        )
        if self.phase != ElevatorPhase.FAILED:
            self._transition(
                ElevatorPhase.WAITING_ELEVATOR,
                now,
                detail,
                True,
            )

    def mark_ready_to_enter(
        self,
        evidence_valid: bool,
        detail: str,
        now: float,
    ) -> None:
        if self.phase not in (
            ElevatorPhase.AT_APPROACH,
            ElevatorPhase.WAITING_ELEVATOR,
        ):
            raise RuntimeError(
                "READY_TO_ENTER invalid in phase {}".format(self.phase)
            )
        self._require_evidence(
            evidence_valid,
            detail,
            now,
            "READY_TO_ENTER",
        )
        if self.phase != ElevatorPhase.FAILED:
            self._transition(
                ElevatorPhase.READY_TO_ENTER,
                now,
                detail,
                True,
            )

    def start_entering(self, now: float) -> None:
        self._require(ElevatorPhase.READY_TO_ENTER)
        self._transition(
            ElevatorPhase.ENTERING,
            now,
            (
                "executing authorized enter path with "
                "precision transit control"
            ),
            True,
        )

    def mark_in_cabin(self, now: float) -> None:
        self._require(ElevatorPhase.ENTERING)
        self._transition(
            ElevatorPhase.IN_CABIN,
            now,
            "enter path completed; robot commanded to hold in cabin",
            True,
        )

    def start_riding(
        self,
        evidence_valid: bool,
        detail: str,
        now: float,
    ) -> None:
        self._require(ElevatorPhase.IN_CABIN)
        self._require_evidence(
            evidence_valid,
            detail,
            now,
            "RIDING",
        )
        if self.phase != ElevatorPhase.FAILED:
            self._transition(
                ElevatorPhase.RIDING,
                now,
                detail,
                True,
            )

    def mark_ready_to_exit(
        self,
        evidence_valid: bool,
        detail: str,
        now: float,
    ) -> None:
        self._require(ElevatorPhase.RIDING)
        self._require_evidence(
            evidence_valid,
            detail,
            now,
            "READY_TO_EXIT",
        )
        if self.phase != ElevatorPhase.FAILED:
            self._transition(
                ElevatorPhase.READY_TO_EXIT,
                now,
                detail,
                True,
            )

    def start_exiting(self, now: float) -> None:
        self._require(ElevatorPhase.READY_TO_EXIT)
        self._transition(
            ElevatorPhase.EXITING,
            now,
            (
                "executing authorized exit path with "
                "precision transit control"
            ),
            True,
        )

    def mark_complete(self, now: float) -> None:
        self._require(ElevatorPhase.EXITING)
        self._transition(
            ElevatorPhase.COMPLETE,
            now,
            "elevator transit execution complete",
            True,
        )

    def cancel(self, reason: str, now: float) -> None:
        if self.phase in ElevatorPhase.ACTIVE:
            self.fail(
                "cancelled: {}".format(reason or "unspecified"),
                now,
            )

    def fail(self, reason: str, now: float) -> None:
        self._transition(
            ElevatorPhase.FAILED,
            now,
            str(reason) if reason else "elevator executor failed closed",
            False,
        )

    def tick(self, now: float) -> None:
        timeout = self._timeout_for_phase()
        if (
            timeout is not None
            and self.phase in ElevatorPhase.ACTIVE
            and float(now) - self.phase_entered_at > timeout
        ):
            self.fail(
                "phase {} timed out after {:.1f}s".format(
                    self.phase,
                    timeout,
                ),
                now,
            )

    def status_identity(self):
        if self.mission is None:
            return 0, "", 0, 0
        return self.mission.identity

    def _timeout_for_phase(self):
        if self.phase == ElevatorPhase.APPROACHING:
            return self.timeouts.approach
        if self.phase in (
            ElevatorPhase.AT_APPROACH,
            ElevatorPhase.WAITING_ELEVATOR,
            ElevatorPhase.READY_TO_ENTER,
            ElevatorPhase.READY_TO_EXIT,
        ):
            return self.timeouts.ready_wait
        if self.phase == ElevatorPhase.ENTERING:
            return self.timeouts.enter
        if self.phase == ElevatorPhase.IN_CABIN:
            return self.timeouts.in_cabin
        if self.phase == ElevatorPhase.RIDING:
            return self.timeouts.ride
        if self.phase == ElevatorPhase.EXITING:
            return self.timeouts.exit
        return None

    def _require(self, expected: int) -> None:
        if self.phase != expected:
            raise RuntimeError(
                "transition requires phase {}, current phase {}".format(
                    expected,
                    self.phase,
                )
            )

    def _require_evidence(
        self,
        evidence_valid: bool,
        detail: str,
        now: float,
        name: str,
    ) -> None:
        if not evidence_valid or not str(detail).strip():
            self.fail(
                (
                    "{} requires formal current evidence; "
                    "none was supplied"
                ).format(name),
                now,
            )

    def _transition(
        self,
        phase: int,
        now: float,
        detail: str,
        valid: bool,
    ) -> None:
        self.phase = int(phase)
        self.phase_entered_at = float(now)
        self.detail = str(detail)
        self.valid = bool(valid)
