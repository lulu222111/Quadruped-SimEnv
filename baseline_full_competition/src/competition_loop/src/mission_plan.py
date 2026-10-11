#!/usr/bin/env python3
"""mission_plan.py — multi-floor mission sequencing from public scene info.

Builds the deterministic floor visit order and elevator/door bindings used by
the multi-floor mission FSM.  Every input field comes from the authorized
public section of ``generated_building/team_scene_info.json``:

  public_scene.elevators[] : {id, served_floors, current_floor}
  public_scene.door_ids[]  : {floor_index, id, kind, initial_open}

No pose, no layout metadata, and no simulator truth is read here.

中文说明：
    本模块把“公开场景信息”翻译成一份确定性的多楼层任务计划，供
    floor_mission_fsm 消费。它只负责规划“去哪些楼层、坐哪部电梯、每层用
    哪个厅门”，并不关心机器人当前在哪、如何运动。

    数据流：
        team_scene_info.json -> public_scene -> 楼层集合 / 电梯 id / 厅门 id
        -> MissionPlan（不可变） -> floor_mission_fsm

    合规约束：
        只读取 public_scene 下的电梯与门 id、served_floors 列表；
        楼层顺序由“所选电梯的服务楼层升序”唯一确定，不含随机性；
        不读取任何位姿、布局元数据（layout_metadata）或仿真真值。
"""

from __future__ import annotations

from typing import Dict, List, Optional, Tuple


class MissionPlanError(ValueError):
    """Raised when the public scene info cannot support a multi-floor plan.

    中文：公开场景信息不足以构造多楼层计划时抛出（fail-closed，不猜测默认值）。
    """


class MissionPlan:
    """Immutable multi-floor mission plan.

    中文：不可变的多楼层任务计划；构造后所有字段只读，FSM 每次查询结果一致。

    Attributes:
        elevator_id:        public elevator identifier used for all transits.
        visit_floors:       ordered floors to explore (starts on start floor).
        start_floor:        floor the robot starts on (return destination).
        door_for_floor:     floor index -> public elevator hall door id.

    中文属性说明：
        elevator_id      本次任务唯一使用的公开电梯 id；
        visit_floors     探索楼层序列（升序，首位必须是 start_floor）；
        start_floor      出发楼层，也是任务结束时需要返回的楼层；
        door_for_floor   楼层号 -> 该层电梯厅门 id（开厅门用官方服务）。
    """

    def __init__(
        self,
        elevator_id: str,
        visit_floors: Tuple[int, ...],
        start_floor: int,
        door_for_floor: Dict[int, str],
    ) -> None:
        if not elevator_id or not str(elevator_id).strip():
            raise MissionPlanError("elevator_id must be a non-empty string")
        self.elevator_id = str(elevator_id).strip()

        floors = tuple(int(f) for f in visit_floors)
        if not floors:
            raise MissionPlanError("visit_floors must not be empty")
        if len(set(floors)) != len(floors):
            raise MissionPlanError("visit_floors must not contain duplicates")
        if any(f < 0 for f in floors):
            raise MissionPlanError("visit_floors must be non-negative")
        if int(start_floor) not in floors:
            raise MissionPlanError("start_floor must appear in visit_floors")
        if floors[0] != int(start_floor):
            raise MissionPlanError("visit_floors must begin with start_floor")
        self.visit_floors = floors
        self.start_floor = int(start_floor)

        doors = {int(f): str(d) for f, d in (door_for_floor or {}).items()}
        self.door_for_floor = doors

    # ------------------------------------------------------------------
    # Queries
    # ------------------------------------------------------------------

    @property
    def has_multiple_floors(self) -> bool:
        # 中文：仅单层时主 FSM 可以跳过跨楼层阶段（MODE A 兼容路径）。
        return len(self.visit_floors) > 1

    def next_floor_after(self, floor: int) -> Optional[int]:
        """Return the next exploration floor after ``floor``, else None.

        中文：返回探索序列中 ``floor`` 的下一层；已是最后一层或楼层未知时
        返回 None —— FSM 用 None 判定进入 ALL_FLOORS_COMPLETE。
        """
        floor = int(floor)
        if floor not in self.visit_floors:
            return None
        index = self.visit_floors.index(floor)
        if index + 1 >= len(self.visit_floors):
            return None
        return self.visit_floors[index + 1]

    def floors_remaining_after(self, floor: int) -> Tuple[int, ...]:
        # 中文：``floor`` 之后仍需探索的楼层子序列（含下一层），供日志/状态展示。
        nxt = self.next_floor_after(floor)
        if nxt is None:
            return ()
        index = self.visit_floors.index(nxt)
        return self.visit_floors[index:]

    def door_id_for(self, floor: int) -> Optional[str]:
        # 中文：查询该层电梯厅门 id；无绑定返回 None（不猜门名）。
        return self.door_for_floor.get(int(floor))

    def describe(self) -> str:
        return "elevator={} floors={} start={} doors={}".format(
            self.elevator_id,
            list(self.visit_floors),
            self.start_floor,
            self.door_for_floor,
        )


def _public_elevators(scene_info: dict) -> List[dict]:
    # 中文：取出 public_scene.elevators；缺失或为空直接 fail-closed。
    scene = (scene_info or {}).get("public_scene")
    if not isinstance(scene, dict):
        raise MissionPlanError("team_scene_info has no public_scene section")
    elevators = scene.get("elevators")
    if not isinstance(elevators, list) or not elevators:
        raise MissionPlanError("public_scene.elevators is missing or empty")
    return elevators


def _select_elevator(scene_info: dict, start_floor: int) -> Tuple[str, set]:
    """Select the elevator serving the start floor with the most floors.

    中文：在“服务起点楼层”的公开电梯中选择服务楼层最多的一部，平票时按
    elevator_id 字典序取小者，保证同一份 scene_info 得到同一计划（可复现）。
    """
    best = None  # (served_count, elevator_id, served)
    for entry in _public_elevators(scene_info):
        elevator_id = str(entry.get("id", "")).strip()
        if not elevator_id:
            raise MissionPlanError("public elevator entry has an empty id")
        served_raw = entry.get("served_floors")
        if not isinstance(served_raw, list) or not served_raw:
            raise MissionPlanError(
                "elevator '{}' has no served_floors list".format(elevator_id)
            )
        served = set()
        for floor in served_raw:
            if isinstance(floor, bool) or not isinstance(floor, int):
                raise MissionPlanError(
                    "elevator '{}' served_floors must contain integers".format(
                        elevator_id
                    )
                )
            if floor < 0:
                raise MissionPlanError(
                    "elevator '{}' served_floors must be non-negative".format(
                        elevator_id
                    )
                )
            served.add(int(floor))
        if int(start_floor) not in served:
            continue
        key = (len(served), elevator_id)
        if best is None or key > (best[0], best[1]):
            best = (len(served), elevator_id, served)
    if best is None:
        raise MissionPlanError(
            "no public elevator serves the start floor {}".format(start_floor)
        )
    return best[1], best[2]


def _door_bindings(scene_info: dict, floors) -> Dict[int, str]:
    """Map each visit floor to its public elevator-kind hall door id.

    中文：只取 kind == "elevator" 的公开门记录做楼层绑定；任何计划访问的
    楼层缺门 id 都视为不可执行，直接抛错而不是运行中才发现。
    """
    scene = (scene_info or {}).get("public_scene", {})
    door_entries = scene.get("door_ids", [])
    if not isinstance(door_entries, list):
        raise MissionPlanError("public_scene.door_ids must be a list")

    bindings: Dict[int, str] = {}
    for entry in door_entries:
        if not isinstance(entry, dict):
            continue
        if str(entry.get("kind", "")) != "elevator":
            continue
        floor = entry.get("floor_index")
        door_id = str(entry.get("id", "")).strip()
        if isinstance(floor, bool) or not isinstance(floor, int):
            continue
        if not door_id:
            continue
        bindings[int(floor)] = door_id

    missing = [f for f in floors if f not in bindings]
    if missing:
        raise MissionPlanError(
            "no public elevator hall door id for floor(s) {}".format(missing)
        )
    return bindings


def build_mission_plan(
    scene_info: dict,
    start_floor: int = 0,
) -> MissionPlan:
    """Build the multi-floor mission plan from public scene info.

    Floor policy: explore every floor served by the selected elevator in
    ascending order, starting on the start floor.  The start floor is always
    visited first even when the robot must later return to it.

    中文：楼层策略 = 探索所选电梯服务的全部楼层，按升序排列，从 start_floor
    开始。起点楼层排在最前，即使任务末尾还需要乘电梯返回该层
    （RETURN_TO_START_FLOOR 由 FSM 负责，不在本函数体现）。
    """
    if isinstance(start_floor, bool) or not isinstance(start_floor, int):
        raise MissionPlanError("start_floor must be an integer")
    if start_floor < 0:
        raise MissionPlanError("start_floor must be non-negative")

    elevator_id, served = _select_elevator(scene_info, start_floor)
    ordered = tuple(sorted(served))
    doors = _door_bindings(scene_info, ordered)
    return MissionPlan(
        elevator_id=elevator_id,
        visit_floors=ordered,
        start_floor=start_floor,
        door_for_floor=doors,
    )
