#!/usr/bin/env python3
"""elevator_locator.py — legal elevator-hall discovery from the robot's own map.

The public scene info exposes elevator and door *identifiers* only; positions
are referee information.  The only authorised observation channel for the
elevator location is therefore the robot's own occupancy map plus the
behavioural effect of the official door service:

  1. The mission controller opens the public hall door id via
     /set_door_state (official API).
  2. The robot re-observes the floor (bounded observation route).
  3. Cells that were unknown/occupied and became free localise the opened
     door pocket; the mouth line and pocket interior then yield the
     approach/cabin transit goals.

This module is pure grid geometry: no ROS, no scene files, no truth.

中文说明：
    用机器人自己的占据栅格“合法地发现”电梯厅门口。公开场景信息只给电梯/厅门
    的 id，位置属于裁判信息，因此唯一被授权的观测通道是：
        官方 /set_door_state 开门 -> 机器人有界重观测 -> 栅格 delta 定位门口袋。

    数据流与算法：
        before/after 两份栅格快照 -> DoorOpenDeltaEstimator.estimate 得到
        “由非自由变为自由”的单元（开门后新暴露的空间）
        -> largest_cluster 取最大 4 连通簇 = 门口袋
        -> mouth_line 找与开门前自由空间相邻的袋口单元
        -> select_transit_goals 由袋口质心沿“口 -> 袋”轴生成：
             approach 点：袋外 approach_standoff 米，朝向袋内；
             cabin 点：袋内 cabin_depth 米，朝向袋内。

    合规约束：
        不读任何隐藏 door/elevator 配置，不使用仿真真值；
        本模块是纯栅格几何，无 ROS 依赖，便于单测。
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Sequence, Tuple

UNKNOWN = -1
FREE = 0


class LocatorError(ValueError):
    """Raised when the map evidence cannot support a transit goal."""


def _cell_bounds(cell, width, height):
    x, y = int(cell[0]), int(cell[1])
    if not (0 <= x < width and 0 <= y < height):
        raise LocatorError("cell {} outside the {}x{} grid".format(cell, width, height))
    return x, y


def _index(cell, width):
    return int(cell[1]) * width + int(cell[0])


def _xy(index, width):
    return (index % width, index // width)


def _neighbours4(x, y):
    yield x + 1, y
    yield x - 1, y
    yield x, y + 1
    yield x, y - 1


class DoorOpenDeltaEstimator:
    """Estimates the opened-pocket cells from two map snapshots.

    Snapshots are flat row-major occupancy lists (ROS convention: -1 unknown,
    0 free, >0 occupancy likelihood).

    中文：从两份栅格快照（ROS 约定：-1 未知、0 自由、>0 占据概率）中估计
    “开门后新变为自由”的单元集合。取最大连通簇即门口袋。
    """

    def __init__(self, width: int, height: int) -> None:
        if int(width) <= 0 or int(height) <= 0:
            raise LocatorError("grid dimensions must be positive")
        self.width = int(width)
        self.height = int(height)

    def estimate(
        self,
        before: Sequence[int],
        after: Sequence[int],
    ) -> List[Tuple[int, int]]:
        if len(before) != self.width * self.height:
            raise LocatorError("before snapshot has wrong length")
        if len(after) != self.width * self.height:
            raise LocatorError("after snapshot has wrong length")

        delta: List[Tuple[int, int]] = []
        for index in range(len(after)):
            old_value = int(before[index])
            new_value = int(after[index])
            if new_value == FREE and old_value != FREE:
                delta.append(_xy(index, self.width))
        return delta

    def largest_cluster(
        self,
        cells: Sequence[Tuple[int, int]],
    ) -> List[Tuple[int, int]]:
        """Return the 4-connected cluster with the most cells."""
        if not cells:
            return []
        cell_set = {(int(c[0]), int(c[1])) for c in cells}
        visited = set()
        best: List[Tuple[int, int]] = []
        for seed in sorted(cell_set):
            if seed in visited:
                continue
            stack = [seed]
            component = []
            visited.add(seed)
            while stack:
                cell = stack.pop()
                component.append(cell)
                for nxt in _neighbours4(cell[0], cell[1]):
                    if nxt in cell_set and nxt not in visited:
                        visited.add(nxt)
                        stack.append(nxt)
            if len(component) > len(best):
                best = component
        return sorted(best)


def mouth_line(
    pocket: Sequence[Tuple[int, int]],
    before: Sequence[int],
    width: int,
    height: int,
) -> List[Tuple[int, int]]:
    """Pocket cells adjacent (4-connected) to cells free *before* the door
    opened — the door mouth through which the pocket is entered.

    中文：门口袋中与“开门前已自由”的单元相邻的那些格子，即进入口袋的入口。
    用它确定“口 -> 袋”方向轴，进而推出 approach 与 cabin 目标。
    """
    pocket_set = {(int(c[0]), int(c[1])) for c in pocket}
    mouth = []
    for cell in sorted(pocket_set):
        for nxt in _neighbours4(cell[0], cell[1]):
            x, y = nxt
            if not (0 <= x < width and 0 <= y < height):
                continue
            if nxt in pocket_set:
                continue
            if int(before[_index(nxt, width)]) == FREE:
                mouth.append(cell)
                break
    return mouth


def _centroid(cells: Sequence[Tuple[int, int]]) -> Tuple[float, float]:
    if not cells:
        raise LocatorError("centroid of empty cell set")
    xs = [float(c[0]) for c in cells]
    ys = [float(c[1]) for c in cells]
    return sum(xs) / len(xs), sum(ys) / len(ys)


def _cell_to_world(cell, resolution, origin_x, origin_y, origin_yaw):
    from math import cos, sin

    local_x = (int(cell[0]) + 0.5) * resolution
    local_y = (int(cell[1]) + 0.5) * resolution
    return (
        origin_x + cos(origin_yaw) * local_x - sin(origin_yaw) * local_y,
        origin_y + sin(origin_yaw) * local_x + cos(origin_yaw) * local_y,
    )


class TransitGoals:
    """Approach and cabin goals in the world frame."""

    __slots__ = (
        "approach_x",
        "approach_y",
        "approach_yaw",
        "cabin_x",
        "cabin_y",
        "cabin_yaw",
    )

    def __init__(
        self,
        approach_x,
        approach_y,
        approach_yaw,
        cabin_x,
        cabin_y,
        cabin_yaw,
    ) -> None:
        self.approach_x = float(approach_x)
        self.approach_y = float(approach_y)
        self.approach_yaw = float(approach_yaw)
        self.cabin_x = float(cabin_x)
        self.cabin_y = float(cabin_y)
        self.cabin_yaw = float(cabin_yaw)

    def as_dict(self) -> Dict[str, Tuple[float, float, float]]:
        return {
            "approach": (
                self.approach_x,
                self.approach_y,
                self.approach_yaw,
            ),
            "cabin": (self.cabin_x, self.cabin_y, self.cabin_yaw),
        }


def select_transit_goals(
    pocket: Sequence[Tuple[int, int]],
    mouth: Sequence[Tuple[int, int]],
    resolution: float,
    origin_x: float = 0.0,
    origin_y: float = 0.0,
    origin_yaw: float = 0.0,
    approach_standoff: float = 0.8,
    cabin_depth: float = 0.6,
) -> TransitGoals:
    """Derive the approach and cabin world goals from the opened pocket.

    * approach goal: ``approach_standoff`` metres outside the mouth centroid,
      on the free side, facing the pocket.
    * cabin goal: ``cabin_depth`` metres inside the pocket from the mouth
      centroid, along the same axis, facing into the pocket.

    中文：由袋口与口袋质心确定主轴，生成两个世界坐标目标：
        approach —— 口外 approach_standoff 米，朝向袋内；
        cabin    —— 袋内 cabin_depth 米，朝向袋内。
    """
    if resolution <= 0.0 or not math.isfinite(resolution):
        raise LocatorError("resolution must be positive and finite")
    if not pocket:
        raise LocatorError("empty pocket")
    if not mouth:
        raise LocatorError("empty mouth: pocket not adjacent to known free space")

    pocket_centroid = _centroid(pocket)
    mouth_centroid = _centroid(mouth)

    # Mouth axis: from the mouth into the pocket.
    axis_x = pocket_centroid[0] - mouth_centroid[0]
    axis_y = pocket_centroid[1] - mouth_centroid[1]
    norm = math.hypot(axis_x, axis_y)
    if norm < 1.0e-6:
        raise LocatorError("pocket and mouth centroids coincide")
    axis_x /= norm
    axis_y /= norm

    standoff_cells = float(approach_standoff) / resolution
    depth_cells = float(cabin_depth) / resolution

    approach_cell = (
        int(round(mouth_centroid[0] - axis_x * standoff_cells)),
        int(round(mouth_centroid[1] - axis_y * standoff_cells)),
    )
    cabin_cell = (
        int(round(mouth_centroid[0] + axis_x * depth_cells)),
        int(round(mouth_centroid[1] + axis_y * depth_cells)),
    )

    approach_world = _cell_to_world(
        approach_cell, resolution, origin_x, origin_y, origin_yaw
    )
    cabin_world = _cell_to_world(
        cabin_cell, resolution, origin_x, origin_y, origin_yaw
    )

    # World-frame heading along the mouth axis (grid +y is world +y rotated
    # by origin_yaw; the yaw rotation preserves the heading offset).
    heading = math.atan2(axis_y, axis_x) + float(origin_yaw)

    return TransitGoals(
        approach_x=approach_world[0],
        approach_y=approach_world[1],
        approach_yaw=heading,
        cabin_x=cabin_world[0],
        cabin_y=cabin_world[1],
        cabin_yaw=heading,
    )


def observation_candidates(
    occupancy: Sequence[int],
    width: int,
    height: int,
    radius_cells: int = 2,
) -> List[Tuple[int, int]]:
    """Free cells within ``radius_cells`` of unknown space, exploration-order
    friendly (row-major).  Used to build the bounded re-observation route
    while the door is open.

    中文：挑出靠近未知区域的自由单元作为重观测路点（按行优先顺序，便于按
    探索顺序依次前往）。用于在开门后构造“有界重观测路线”，
    保证机器人在有限步内获得足够的栅格 delta 证据。
    """
    if int(width) <= 0 or int(height) <= 0:
        raise LocatorError("grid dimensions must be positive")
    width = int(width)
    height = int(height)
    if len(occupancy) != width * height:
        raise LocatorError("occupancy snapshot has wrong length")

    candidates = []
    for y in range(height):
        for x in range(width):
            if int(occupancy[_index((x, y), width)]) != FREE:
                continue
            near_unknown = False
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    nx, ny = x + dx, y + dy
                    if not (0 <= nx < width and 0 <= ny < height):
                        continue
                    if int(occupancy[_index((nx, ny), width)]) == UNKNOWN:
                        near_unknown = True
                        break
                if near_unknown:
                    break
            if near_unknown:
                candidates.append((x, y))
    return candidates
