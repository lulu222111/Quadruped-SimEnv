#!/usr/bin/env python3
"""elevator_service_sequence.py — official-service evidence for the executor.

Bridges the frozen fail-closed elevator FSM to the only authorised building
APIs:

  /call_elevator  (building_generator_interfaces/CallElevator)
  /set_door_state (building_generator_interfaces/SetDoorState)

The ElevatorStatus V1 contract requires *formal, observable evidence* before
READY_TO_ENTER / RIDING / READY_TO_EXIT.  The authorised evidence sources are
exactly the responses of the official services above:

  * CallElevator succeeds and reports ``current_floor == target``:
    the cabin is observably at that floor.
  * SetDoorState succeeds and reports the door open:
    the hall door is observably open.

No simulator truth, world/SDF pose, timing inference, or hidden layout
metadata is used.  Door identifiers come from the public
team_scene_info ``public_scene.door_ids`` (kind == "elevator") mapping.

中文说明：
    把 fail-closed 电梯状态机与“唯一被授权的建筑官方 API”桥接起来。

    证据规则（ElevatorStatus V1 契约要求正式可观测证据）：
        - 轿厢到位证据：/call_elevator 成功且返回的 current_floor 等于目标楼层；
        - 门开启证据：  /set_door_state 成功且返回状态为 open/opened/opening。
        不使用时间推断、仿真真值、world/SDF 位姿或隐藏布局；
        门 id 只来自公开 team_scene_info 的 public_scene.door_ids。

    阶段推进：
        AT_APPROACH -> WAITING_ELEVATOR -> READY_TO_ENTER -> ENTERING
        IN_CABIN    -> RIDING -> READY_TO_EXIT -> EXITING
    任一服务返回与预期矛盾即抛 EvidenceError，由调用方把执行器 fail-closed。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


class EvidenceError(RuntimeError):
    """Raised when official-service evidence contradicts the transit."""


class TransitStep:
    """One official-service call required to advance a transit phase.

    中文：推进一个 transit 阶段所需的一次官方服务调用（调用电梯或设置门状态）。
    纯数据对象，便于在无 ROS 环境下单测服务序列。
    """

    CALL_ELEVATOR = "call_elevator"
    SET_DOOR_STATE = "set_door_state"

    __slots__ = ("kind", "elevator_id", "target_floor", "door_id", "open")

    def __init__(
        self,
        kind: str,
        elevator_id: str = "",
        target_floor: int = -1,
        door_id: str = "",
        open: Optional[bool] = None,
    ) -> None:
        self.kind = kind
        self.elevator_id = str(elevator_id)
        self.target_floor = int(target_floor)
        self.door_id = str(door_id)
        self.open = open

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        if self.kind == TransitStep.CALL_ELEVATOR:
            return "call_elevator({}, {})".format(
                self.elevator_id, self.target_floor
            )
        return "set_door_state({}, open={})".format(self.door_id, self.open)

    def __eq__(self, other) -> bool:
        return (
            isinstance(other, TransitStep)
            and self.kind == other.kind
            and self.elevator_id == other.elevator_id
            and self.target_floor == other.target_floor
            and self.door_id == other.door_id
            and self.open == other.open
        )


def _door_for(door_for_floor: Dict[int, str], floor: int) -> str:
    door_id = (door_for_floor or {}).get(int(floor))
    if not door_id or not str(door_id).strip():
        raise EvidenceError(
            "no public elevator hall door id for floor {}".format(floor)
        )
    return str(door_id).strip()


def plan_pre_enter_steps(
    mission,
    door_for_floor: Dict[int, str],
) -> List[TransitStep]:
    """Steps to bring the cabin to the source floor and open its hall door.

    Valid while the executor is in AT_APPROACH (approach path completed).

    中文：AT_APPROACH 阶段的服务序列 —— 把轿厢叫到源楼层，并打开源层厅门。
    """
    return [
        TransitStep(
            TransitStep.CALL_ELEVATOR,
            elevator_id=mission.elevator_id,
            target_floor=int(mission.source_floor),
        ),
        TransitStep(
            TransitStep.SET_DOOR_STATE,
            door_id=_door_for(door_for_floor, mission.source_floor),
            open=True,
        ),
    ]


def plan_ride_steps(
    mission,
    door_for_floor: Dict[int, str],
) -> List[TransitStep]:
    """Steps to close the source door, ride to the target floor, and open
    the target hall door.

    Valid while the executor is in IN_CABIN (enter path completed).

    中文：IN_CABIN 阶段的服务序列 —— 关源层厅门、乘梯到目标层、开目标层厅门。
    顺序不可交换：先关门再叫梯，最后开目标层门，保证乘梯过程中门是关闭的。
    """
    return [
        TransitStep(
            TransitStep.SET_DOOR_STATE,
            door_id=_door_for(door_for_floor, mission.source_floor),
            open=False,
        ),
        TransitStep(
            TransitStep.CALL_ELEVATOR,
            elevator_id=mission.elevator_id,
            target_floor=int(mission.target_floor),
        ),
        TransitStep(
            TransitStep.SET_DOOR_STATE,
            door_id=_door_for(door_for_floor, mission.target_floor),
            open=True,
        ),
    ]


def _require_accepted(response: dict, service: str) -> None:
    if not isinstance(response, dict) or not bool(
        response.get("accepted", False)
    ):
        detail = "?"
        if isinstance(response, dict):
            detail = response.get("message", "?")
        raise EvidenceError(
            "{} did not accept the request: {}".format(service, detail)
        )


def apply_pre_enter_evidence(
    core,
    call_response: dict,
    door_response: dict,
    now: float,
) -> None:
    """Advance AT_APPROACH -> WAITING_ELEVATOR -> READY_TO_ENTER -> ENTERING.

    Raises EvidenceError when the official services contradict the transit;
    the caller must then fail the executor closed.

    中文：校验“轿厢在源楼层 + 源门已开”两项官方证据后，依次推进
    WAITING_ELEVATOR / READY_TO_ENTER 并开始 ENTERING。
    """
    _require_accepted(call_response, "/call_elevator")
    reported_floor = call_response.get("current_floor")
    if (
        isinstance(reported_floor, bool)
        or not isinstance(reported_floor, int)
        or reported_floor != core.mission.source_floor
    ):
        raise EvidenceError(
            "cabin is not at source floor {}: /call_elevator reports {}".format(
                core.mission.source_floor, reported_floor
            )
        )

    _require_accepted(door_response, "/set_door_state")
    state = str(door_response.get("state", "")).lower()
    if state not in ("open", "opened", "opening"):
        raise EvidenceError(
            "source hall door is not open: state={!r}".format(state)
        )

    core.mark_waiting_elevator(
        True,
        "cabin at source floor per /call_elevator",
        now,
    )
    core.mark_ready_to_enter(
        True,
        "source hall door open per /set_door_state",
        now,
    )
    core.start_entering(now)


def apply_ride_evidence(
    core,
    close_response: dict,
    call_response: dict,
    door_response: dict,
    now: float,
) -> None:
    """Advance IN_CABIN -> RIDING -> READY_TO_EXIT -> EXITING.

    Raises EvidenceError when the official services contradict the transit;
    the caller must then fail the executor closed.

    中文：校验“源门已关 + 轿厢到目标楼层 + 目标门已开”三项官方证据后，
    依次推进 RIDING / READY_TO_EXIT 并开始 EXITING。
    """
    _require_accepted(close_response, "/set_door_state")
    state = str(close_response.get("state", "")).lower()
    if state not in ("closed", "close", "closing"):
        raise EvidenceError(
            "source hall door did not close: state={!r}".format(state)
        )

    _require_accepted(call_response, "/call_elevator")
    reported_floor = call_response.get("current_floor")
    if (
        isinstance(reported_floor, bool)
        or not isinstance(reported_floor, int)
        or reported_floor != core.mission.target_floor
    ):
        raise EvidenceError(
            "cabin is not at target floor {}: /call_elevator reports {}".format(
                core.mission.target_floor, reported_floor
            )
        )

    _require_accepted(door_response, "/set_door_state")
    state = str(door_response.get("state", "")).lower()
    if state not in ("open", "opened", "opening"):
        raise EvidenceError(
            "target hall door is not open: state={!r}".format(state)
        )

    core.start_riding(
        True,
        "cabin arrived at target floor per /call_elevator",
        now,
    )
    core.mark_ready_to_exit(
        True,
        "target hall door open per /set_door_state",
        now,
    )
    core.start_exiting(now)
