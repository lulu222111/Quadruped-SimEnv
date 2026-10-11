#!/usr/bin/env python3
"""floor_mission_fsm.py — pure multi-floor mission sub-state machine.

Drives the mission-level flow that lives *inside* the main competition
RUNNING phase:

  FLOOR_EXPLORATION
    -> ELEVATOR_LOCATE        (planning locates the hall door on this floor)
    -> ELEVATOR_REQUEST       (publish ElevatorMission)
    -> ELEVATOR_TRANSIT       (executor: APPROACH..COMPLETE / FAILED)
    -> FLOOR_SWITCH           (mapping sentinel + new floor + module resets)
    -> ... next floor ...
    -> ALL_FLOORS_COMPLETE
    -> RETURN_TRANSIT         (elevator back to the start floor, if needed)
    -> RETURN_HOME            (delegate to the main RETURNING flow)
    -> MISSION_COMPLETE

The FSM is ROS-free and returns immutable Command tuples that the ROS
adapter (competition_loop_node) executes.  It never reads simulator truth
or hidden layout data; the floor plan comes from mission_plan.MissionPlan
built over the public team_scene_info fields.

中文说明：
    本模块是“主状态机 RUNNING 阶段内部”的楼层子状态机，纯 Python、无 ROS 依赖，
    只根据外部事件返回 Command 元组，由 competition_loop_node 翻译成 ROS 动作。

    子状态机与主 FSM 的关系：
        CompetitionStateMachine 负责 INITIALIZE / WAITING / RUNNING /
        RETURNING / FINISHED 等顶层阶段；
        FloorMissionStateMachine 只在 RUNNING 内推进多楼层流程，
        当它进入 RETURN_HOME 时通过 BEGIN_RETURNING 把控制权交还主 FSM。

    状态推进（跨楼层）：
        FLOOR_EXPLORATION  当前楼层自主探索
          -> ELEVATOR_LOCATE   请求 planning 用合法观测发现本层电梯厅门口
          -> ELEVATOR_REQUEST  发布 ElevatorMission 给电梯执行器
          -> ELEVATOR_TRANSIT  执行器完成 APPROACH..COMPLETE / FAILED
          -> FLOOR_SWITCH      mapping 过渡 + 新楼层 + planning/localization 复位
          -> 下一层 / ALL_FLOORS_COMPLETE

    为什么“全部楼层探索完”之后还要 RETURN_TO_START_FLOOR -> RETURN_HOME：
        比赛要求在探索结束后让机器人自主返回到出发位置附近（返航）。若结束时
        停在非起始楼层，必须先乘电梯回到起始楼层（purpose="return"），
        再交给主 FSM 的 RETURNING 流程走回家路径；只有“已回到起始楼层”
        才允许直接发起返航，避免在错误楼层上规划回家目标。层级为：
        ALL_FLOORS_COMPLETE -> (需要则) RETURN_TRANSIT -> RETURN_HOME
        -> 主 FSM RETURNING -> ARRIVED_START -> FINISHED。

    合规约束：
        不读取仿真真值、不读隐藏布局；门/电梯只通过公开 id 与官方服务交互。
"""

from __future__ import annotations

import math
from typing import Optional, Tuple


class FloorMissionState:
    # 中文：楼层子状态机状态集合；取值同时作为日志/status detail 的文本。
    FLOOR_EXPLORATION = "FLOOR_EXPLORATION"   # 本层自主探索中
    ELEVATOR_LOCATE = "ELEVATOR_LOCATE"       # 定位本层电梯厅门口（合法观测）
    ELEVATOR_REQUEST = "ELEVATOR_REQUEST"     # 已发布 ElevatorMission，等待执行器受理
    ELEVATOR_TRANSIT = "ELEVATOR_TRANSIT"     # 执行器运行中（进梯/乘梯/出梯）
    FLOOR_SWITCH = "FLOOR_SWITCH"             # 切换地图层并复位 planning/localization
    ALL_FLOORS_COMPLETE = "ALL_FLOORS_COMPLETE"  # 所有目标楼层已探索完
    RETURN_HOME = "RETURN_HOME"               # 交还主 FSM 执行返航
    MISSION_COMPLETE = "MISSION_COMPLETE"
    MISSION_FAILED = "MISSION_FAILED"

    TERMINAL = {MISSION_COMPLETE, MISSION_FAILED}


class CommandKind:
    # 中文：FSM 输出的“意图”，由 competition_loop_node 解释为具体 ROS 操作。
    REQUEST_ELEVATOR_LOCATE = "request_elevator_locate"    # 通知 planning 开始 locate
    OPEN_ELEVATOR_DOOR = "open_elevator_door"              # 用官方服务打开公开厅门 id
    PUBLISH_ELEVATOR_MISSION = "publish_elevator_mission"  # 发布 ElevatorMission
    BEGIN_FLOOR_TRANSITION = "begin_floor_transition"      # 通知 mapping 进入 sentinel 过渡
    ACTIVATE_FLOOR = "activate_floor"                      # 正式激活目标楼层
    PLANNING_FLOOR_RESET = "planning_floor_reset"          # 复位 planning 楼层相关状态
    LOCALIZATION_FLOOR_RESET = "localization_floor_reset"  # 复位定位校正/对齐状态
    BEGIN_RETURNING = "begin_returning"                    # 交还主 FSM 执行返航
    SET_DETAIL = "set_detail"                              # 仅更新状态文本（可观测性）


class Command:
    """Immutable command emitted by the FSM for the ROS adapter."""

    __slots__ = ("kind", "payload")

    def __init__(self, kind: str, payload=None) -> None:
        self.kind = kind
        self.payload = payload

    def __repr__(self) -> str:  # pragma: no cover - debug helper
        return "Command({}, {})".format(self.kind, self.payload)

    def __eq__(self, other) -> bool:
        return (
            isinstance(other, Command)
            and self.kind == other.kind
            and self.payload == other.payload
        )


class FloorMissionStateMachine:
    """Event-driven multi-floor mission FSM.

    Args:
        plan:               mission_plan.MissionPlan (public scene info).
        locate_timeout:     seconds ELEVATOR_LOCATE may run before failing.
        request_timeout:    seconds ELEVATOR_REQUEST may wait for executor
                            acceptance (status must leave IDLE).
        floor_switch_timeout: seconds FLOOR_SWITCH may wait for the mapping
                            floor acknowledgement and fresh planning status.
        max_elevator_retries: transits per floor before MISSION_FAILED.

    中文：事件驱动、无 ROS 依赖；每个 on_* 方法接收一次外部事实并返回
    Command 元组。所有超时都收敛到 MISSION_FAILED，不做隐式重试。
    """

    def __init__(
        self,
        plan,
        locate_timeout: float = 180.0,
        request_timeout: float = 30.0,
        floor_switch_timeout: float = 60.0,
        max_elevator_retries: int = 1,
    ) -> None:
        for name, value in (
            ("locate_timeout", locate_timeout),
            ("request_timeout", request_timeout),
            ("floor_switch_timeout", floor_switch_timeout),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if isinstance(max_elevator_retries, bool) or max_elevator_retries < 1:
            raise ValueError("max_elevator_retries must be an integer >= 1")

        self.plan = plan
        self.locate_timeout = float(locate_timeout)
        self.request_timeout = float(request_timeout)
        self.floor_switch_timeout = float(floor_switch_timeout)
        self.max_elevator_retries = int(max_elevator_retries)

        self.state = FloorMissionState.FLOOR_EXPLORATION
        self.detail = "exploring floor {}".format(plan.start_floor)
        self.current_floor = int(plan.start_floor)

        self._mission_counter = 0
        self._active_mission_id = None
        self._transit_purpose = None  # "explore" | "return"
        self._transit_target_floor = None
        self._sentinel_emitted = False
        self._mission_accepted = False
        self._retry_count = 0
        self._state_entered_at: Optional[float] = None
        self._exploration_complete_latched = False
        self._mapping_floor_ack = False
        self._planning_alive = False

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @property
    def is_terminal(self) -> bool:
        return self.state in FloorMissionState.TERMINAL

    @property
    def is_returning_home(self) -> bool:
        return self.state == FloorMissionState.RETURN_HOME

    def _elapsed(self, now: float) -> float:
        if self._state_entered_at is None:
            return 0.0
        return float(now) - self._state_entered_at

    def _enter(self, state: str, detail: str, now: float, commands) -> None:
        self.state = state
        self.detail = str(detail)
        self._state_entered_at = float(now)
        commands.append(
            Command(CommandKind.SET_DETAIL, "FLOOR_MISSION:{}".format(state))
        )

    def _fail(self, reason: str, now: float, commands) -> None:
        self._enter(
            FloorMissionState.MISSION_FAILED,
            reason,
            now,
            commands,
        )

    def _next_transit_floor(self) -> Optional[int]:
        if self._transit_purpose == "return":
            return self.plan.start_floor
        return self.plan.next_floor_after(self.current_floor)

    # ------------------------------------------------------------------
    # Events
    # ------------------------------------------------------------------

    def start(self, now: float) -> Tuple[Command, ...]:
        """Activate the FSM (called when the main competition enters RUNNING)."""
        commands: list = []
        if self._state_entered_at is None:
            self._state_entered_at = float(now)
            commands.append(
                Command(
                    CommandKind.SET_DETAIL,
                    "FLOOR_MISSION:{}".format(self.state),
                )
            )
        return tuple(commands)

    def on_exploration_complete(
        self, value: bool, now: float
    ) -> Tuple[Command, ...]:
        # 中文：探索完成信号来自 planning 的 exploration_complete topic。
        # 采用“去抖 + 锁存”：value 为 False 时解除锁存（允许本层之后再次完成），
        # 为 True 且尚未锁存时只推进一次，避免同一层的重复 True 触发多次切层。
        commands: list = []
        if self.state != FloorMissionState.FLOOR_EXPLORATION:
            return tuple(commands)
        if not value:
            self._exploration_complete_latched = False
            return tuple(commands)
        if self._exploration_complete_latched:
            return tuple(commands)
        self._exploration_complete_latched = True

        self._retry_count = 0
        if self.plan.next_floor_after(self.current_floor) is None:
            self._enter(
                FloorMissionState.ALL_FLOORS_COMPLETE,
                "all floors explored on floor {}".format(self.current_floor),
                now,
                commands,
            )
            self._resolve_all_floors_complete(now, commands)
        else:
            self._begin_locate(now, commands, purpose="explore")
        return tuple(commands)

    def _resolve_all_floors_complete(self, now: float, commands) -> None:
        # 中文：所有目标楼层探索完成后统一在此分流：
        #   仍在非起始楼层 -> 以 purpose="return" 再乘一次电梯回起始层；
        #   已在起始楼层   -> 直接进入 RETURN_HOME，交给主 FSM 返航。
        self._retry_count = 0
        if self.current_floor != self.plan.start_floor:
            self._begin_locate(now, commands, purpose="return")
        else:
            self._begin_return_home(now, commands)

    def _begin_return_home(self, now: float, commands) -> None:
        self._enter(
            FloorMissionState.RETURN_HOME,
            "all floors complete; returning home on start floor",
            now,
            commands,
        )
        commands.append(Command(CommandKind.BEGIN_RETURNING, None))

    def _begin_locate(self, now: float, commands, purpose: str) -> None:
        # 中文：先按公开 door id 打开本层厅门（官方 /set_door_state），再请
        # planning 做有界重观测以发现门口袋。“开门 -> 观测 -> 定位”是唯一
        # 合法的电梯位置获取途径，不读任何门/电梯位置配置。
        self._transit_purpose = purpose
        self._enter(
            FloorMissionState.ELEVATOR_LOCATE,
            "locating elevator approach on floor {}".format(self.current_floor),
            now,
            commands,
        )
        door_id = self.plan.door_id_for(self.current_floor)
        if door_id is not None:
            commands.append(Command(CommandKind.OPEN_ELEVATOR_DOOR, door_id))
        commands.append(Command(CommandKind.REQUEST_ELEVATOR_LOCATE, None))

    def on_elevator_approach(self, now: float) -> Tuple[Command, ...]:
        """Planning reported an approach/cabin pose for this floor's elevator."""
        commands: list = []
        if self.state != FloorMissionState.ELEVATOR_LOCATE:
            return tuple(commands)
        target = self._next_transit_floor()
        if target is None:
            self._fail("no target floor for transit", now, commands)
            return tuple(commands)
        self._enter(
            FloorMissionState.ELEVATOR_REQUEST,
            "requesting elevator transit {} -> {}".format(
                self.current_floor, target
            ),
            now,
            commands,
        )
        mission = self.build_elevator_mission(now)
        if mission is None:  # defensive; target was checked above
            self._fail("elevator mission could not be built", now, commands)
            return tuple(commands)
        commands.append(
            Command(CommandKind.PUBLISH_ELEVATOR_MISSION, mission)
        )
        return tuple(commands)

    def tick(self, now: float) -> Tuple[Command, ...]:
        commands: list = []
        if self.is_terminal:
            return tuple(commands)

        if self.state == FloorMissionState.ELEVATOR_LOCATE:
            if self._elapsed(now) > self.locate_timeout:
                self._fail(
                    "elevator locate timed out on floor {}".format(
                        self.current_floor
                    ),
                    now,
                    commands,
                )
        elif self.state == FloorMissionState.ELEVATOR_REQUEST:
            if self._elapsed(now) > self.request_timeout:
                self._fail(
                    "elevator executor never accepted mission {}->{}".format(
                        self.current_floor,
                        self._next_transit_floor(),
                    ),
                    now,
                    commands,
                )
        elif self.state == FloorMissionState.FLOOR_SWITCH:
            if self._elapsed(now) > self.floor_switch_timeout:
                self._fail(
                    "floor switch to {} timed out".format(
                        self._transit_target_floor
                    ),
                    now,
                    commands,
                )
        return tuple(commands)

    def on_elevator_status(
        self,
        mission_id: int,
        phase: int,
        valid: bool,
        now: float,
    ) -> Tuple[Command, ...]:
        commands: list = []

        if self.state not in (
            FloorMissionState.ELEVATOR_REQUEST,
            FloorMissionState.ELEVATOR_TRANSIT,
        ):
            return tuple(commands)

        if self._active_mission_id is None or int(
            mission_id
        ) != self._active_mission_id:
            return tuple(commands)

        # 中文：执行器报告失败（valid=false 或 FAILED）时按 max_elevator_retries
        # 重新走一遍 locate -> request；超限才终止任务。重试会清空 mission id /
        # sentinel 状态，因为新的 transit 需要重新发布 mission 并重新过渡地图。
        if not valid or phase == 11:  # ElevatorStatus.FAILED == 11
            self._retry_count += 1
            if self._retry_count <= self.max_elevator_retries:
                rospy_like_logwarn(
                    "elevator transit failed; retry {}/{}".format(
                        self._retry_count, self.max_elevator_retries
                    )
                )
                self._active_mission_id = None
                self._mission_accepted = False
                self._sentinel_emitted = False
                self._begin_locate(
                    now, commands, purpose=self._transit_purpose
                )
            else:
                self._fail(
                    "elevator transit failed after {} attempt(s)".format(
                        self._retry_count
                    ),
                    now,
                    commands,
                )
            return tuple(commands)

        if self.state == FloorMissionState.ELEVATOR_REQUEST:
            # Any active phase means the executor accepted the mission.
            self._mission_accepted = True
            self._enter(
                FloorMissionState.ELEVATOR_TRANSIT,
                "elevator transit {}->{} in progress".format(
                    self.current_floor,
                    self._transit_target_floor,
                ),
                now,
                commands,
            )
            return tuple(commands)

        # Mapping contract: the transition sentinel must be published before
        # elevator movement begins (robot holding inside the cabin).
        # 中文：与 mapping 的契约 —— 必须在轿厢开始运动前先发楼层过渡 sentinel，
        # 此时机器人静止在轿厢内，避免垂直运动点云污染旧楼层栅格。
        if phase in (6, 7) and not self._sentinel_emitted:  # IN_CABIN, RIDING
            self._sentinel_emitted = True
            commands.append(
                Command(CommandKind.BEGIN_FLOOR_TRANSITION, None)
            )

        if phase == 10:  # ElevatorStatus.COMPLETE == 10
            # 中文：执行器报 COMPLETE 后才切换楼层：先复位 planning /
            # localization 的楼层相关状态，再激活新楼层栅格。
            self._enter(
                FloorMissionState.FLOOR_SWITCH,
                "transit complete; activating floor {}".format(
                    self._transit_target_floor
                ),
                now,
                commands,
            )
            new_floor = int(self._transit_target_floor)
            commands.append(
                Command(CommandKind.PLANNING_FLOOR_RESET, new_floor)
            )
            commands.append(
                Command(
                    CommandKind.LOCALIZATION_FLOOR_RESET, new_floor
                )
            )
            commands.append(Command(CommandKind.ACTIVATE_FLOOR, new_floor))
        return tuple(commands)

    def on_mapping_current_floor(
        self, floor: int, now: float
    ) -> Tuple[Command, ...]:
        commands: list = []
        if self.state != FloorMissionState.FLOOR_SWITCH:
            return tuple(commands)
        if int(floor) != int(self._transit_target_floor):
            return tuple(commands)
        self._mapping_floor_ack = True
        self._maybe_finish_floor_switch(now, commands)
        return tuple(commands)

    def on_planning_status_alive(self, now: float) -> Tuple[Command, ...]:
        commands: list = []
        if self.state != FloorMissionState.FLOOR_SWITCH:
            return tuple(commands)
        self._planning_alive = True
        self._maybe_finish_floor_switch(now, commands)
        return tuple(commands)

    def _maybe_finish_floor_switch(self, now: float, commands) -> None:
        # 中文：楼层切换必须同时收到两个独立证据才算完成：
        #   1) mapping 回报 current_floor == 目标楼层（栅格层已真正激活）；
        #   2) planning 状态仍存活（已按新楼层重启规划）。
        # 只等其中一个会让 FSM 在模块尚未就绪时就下发新任务。
        if not (getattr(self, "_mapping_floor_ack", False)
                and getattr(self, "_planning_alive", False)):
            return
        previous_floor = self.current_floor
        self.current_floor = int(self._transit_target_floor)
        self._mapping_floor_ack = False
        self._planning_alive = False
        self._sentinel_emitted = False
        self._active_mission_id = None
        self._mission_accepted = False
        self._exploration_complete_latched = False

        if self._transit_purpose == "return":
            self._begin_return_home(now, commands)
            return

        if self.plan.next_floor_after(self.current_floor) is None:
            self._enter(
                FloorMissionState.ALL_FLOORS_COMPLETE,
                "all floors explored (arrived floor {} from {})".format(
                    self.current_floor, previous_floor
                ),
                now,
                commands,
            )
            self._resolve_all_floors_complete(now, commands)
        else:
            self._enter(
                FloorMissionState.FLOOR_EXPLORATION,
                "exploring floor {}".format(self.current_floor),
                now,
                commands,
            )

    # ------------------------------------------------------------------
    # Mission publication (called by the adapter after emitting the request)
    # ------------------------------------------------------------------

    def build_elevator_mission(self, now: float) -> Optional[dict]:
        """Return the ElevatorMission payload for the pending transit."""
        if self.state != FloorMissionState.ELEVATOR_REQUEST:
            return None
        target = self._next_transit_floor()
        if target is None:
            return None
        self._mission_counter += 1
        self._active_mission_id = self._mission_counter
        self._transit_target_floor = int(target)
        return {
            "mission_id": self._mission_counter,
            "elevator_id": self.plan.elevator_id,
            "source_floor": self.current_floor,
            "target_floor": int(target),
            "stamp": float(now),
        }

    def on_return_arrived(self, now: float) -> Tuple[Command, ...]:
        commands: list = []
        if self.state == FloorMissionState.RETURN_HOME:
            self._enter(
                FloorMissionState.MISSION_COMPLETE,
                "return home complete",
                now,
                commands,
            )
        return tuple(commands)

    def force_failed(self, reason: str, now: float) -> Tuple[Command, ...]:
        commands: list = []
        if not self.is_terminal:
            self._fail(reason, now, commands)
        return tuple(commands)


def rospy_like_logwarn(message: str) -> None:  # pragma: no cover
    """Isolated logging seam so pure tests never need rospy."""
    try:  # pragma: no cover - exercised only under ROS
        import rospy

        rospy.logwarn(message)
    except Exception:
        pass
