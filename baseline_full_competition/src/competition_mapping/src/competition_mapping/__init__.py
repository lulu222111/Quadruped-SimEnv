# =============================================================================
# 中文说明（模块级）：competition_mapping 包入口
#
# 对外导出纯算法部分：FloorMapManager（每层一张栅格的楼层地图管理）与
# OccupancyGridCore（单层栅格核心），ROS 侧封装放在 scripts/mapping_node。
# 这样算法可在没有 ROS Master 的环境下单独验证。
# =============================================================================
"""competition_mapping Python package."""

from .floor_map_manager import FloorMapManager, FloorMapState
from .occupancy_grid_core import OccupancyGridCore, transform_point_3d

__all__ = [
    "FloorMapManager",
    "FloorMapState",
    "OccupancyGridCore",
    "transform_point_3d",
]
