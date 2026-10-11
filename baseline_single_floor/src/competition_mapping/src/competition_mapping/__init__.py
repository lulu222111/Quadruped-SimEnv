# =============================================================================
# 模块说明（中文）：competition_mapping 包入口
#
# 对外只导出纯算法部分（OccupancyGridCore、transform_point_3d），
# ROS 侧封装放在 scripts/mapping_node。这样算法可以在没有 ROS Master 的环境下
# 单独测试与复用。
# =============================================================================
"""competition_mapping Python package."""

from .occupancy_grid_core import OccupancyGridCore, transform_point_3d

__all__ = ["OccupancyGridCore", "transform_point_3d"]
