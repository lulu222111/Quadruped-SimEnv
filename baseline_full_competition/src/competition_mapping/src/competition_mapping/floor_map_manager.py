#!/usr/bin/env python3
"""ROS-independent storage for independent per-floor occupancy maps.

中文说明：
    多楼层栅格存储的纯逻辑实现：每层一份独立的 OccupancyGridCore，
    任一时刻只有一层是“当前激活层”。

    为什么必须分层独立存储：
        不同楼层的点云在 z 方向上重叠投影到同一张 2D 图时会产生互相污染的
        假障碍/假自由空间。把每层栅格物理隔离，切层只是切换指针，是避免
        跨楼层污染的最直接手段。

    与 ROS 的边界：
        本模块不含任何 ROS message 或 topic 名，ROS 适配层（mapping_node）
        可以独立演化，只要不改变这里的分层与隔离契约。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

from .occupancy_grid_core import OccupancyGridCore


GridFactory = Callable[[], OccupancyGridCore]


@dataclass
class FloorMapState:
    """Mutable mapping state belonging to exactly one floor.

    中文：单楼层可变建图状态。grid 是该层独立栅格，其余字段描述该层的
    初始化与最近一次扫描统计，切层时随层保留/复位。
    """

    floor_id: int
    grid: OccupancyGridCore
    initialized: bool = False
    load_time: Optional[object] = None
    last_scan_stats: Optional[dict] = None


class FloorMapManager:
    """Own independent grids while exposing one active floor at a time.

    The manager deliberately contains no ROS messages or topic names.  A ROS
    adapter may therefore change later without changing the map-storage and
    isolation contract tested here.

    中文：管理所有楼层状态并暴露唯一 active 层。首次访问的未知楼层会被
    惰性创建为空图（不预分配、不带任何假设）；已访问楼层的历史栅格一直保留。
    """

    def __init__(self, grid_factory: GridFactory, initial_floor: int = 0) -> None:
        self._grid_factory = grid_factory
        self._states: Dict[int, FloorMapState] = {}
        self._current_floor = self._validate_floor(initial_floor)
        self._ensure_floor(self._current_floor)

    @staticmethod
    def _validate_floor(floor_id: int) -> int:
        if isinstance(floor_id, bool):
            raise ValueError("floor_id must be an integer >= 0")
        try:
            value = int(floor_id)
        except (TypeError, ValueError):
            raise ValueError("floor_id must be an integer >= 0")
        if value < 0 or value != floor_id:
            raise ValueError("floor_id must be an integer >= 0")
        return value

    def _ensure_floor(self, floor_id: int) -> FloorMapState:
        if floor_id not in self._states:
            self._states[floor_id] = FloorMapState(
                floor_id=floor_id,
                grid=self._grid_factory(),
            )
        return self._states[floor_id]

    @property
    def current_floor(self) -> int:
        return self._current_floor

    @property
    def current(self) -> FloorMapState:
        return self._states[self._current_floor]

    @property
    def floor_ids(self) -> Tuple[int, ...]:
        return tuple(sorted(self._states))

    def state_for(self, floor_id: int) -> FloorMapState:
        """Return an existing layer without creating it.

        中文：读取已存在楼层状态；不存在则 KeyError（与“惰性创建”区分开）。
        """
        return self._states[self._validate_floor(floor_id)]

    def switch_floor(self, floor_id: int) -> Tuple[int, int]:
        """Activate a layer, creating an unknown map on its first visit.

        中文：切换激活层，返回 (旧楼层, 新楼层)。首次到访该层时创建空图，
        已访问楼层的历史栅格原样保留。
        """
        new_floor = self._validate_floor(floor_id)
        old_floor = self._current_floor
        self._ensure_floor(new_floor)
        self._current_floor = new_floor
        return old_floor, new_floor

    def reset_current(self) -> None:
        # 中文：仅清空当前层的栅格与统计，其余楼层不受影响。
        self._reset_state(self.current)

    def reset_all(self) -> None:
        # 中文：清空所有楼层栅格（保留楼层键本身）。
        for state in self._states.values():
            self._reset_state(state)

    def clear_to_floor(self, floor_id: int) -> None:
        """Discard every historical layer and recreate one initial layer.

        中文：丢弃全部历史楼层，只重建一个空层。用于仿真时间线重置等
        “过去的地图已完全不可信”的场景。
        """
        initial_floor = self._validate_floor(floor_id)
        self._states.clear()
        self._current_floor = initial_floor
        self._ensure_floor(initial_floor)

    def rebuild_all(self, grid_factory: GridFactory) -> None:
        """Rebuild every layer after the shared map geometry/origin changes.

        中文：当共用地图几何或原点变化时，按新工厂重建每一层栅格
        （楼层键与 active 层保持不变）。
        """
        self._grid_factory = grid_factory
        for floor_id in tuple(self._states):
            self._states[floor_id] = FloorMapState(
                floor_id=floor_id,
                grid=self._grid_factory(),
            )

    @staticmethod
    def _reset_state(state: FloorMapState) -> None:
        state.grid.reset()
        state.initialized = False
        state.load_time = None
        state.last_scan_stats = None
