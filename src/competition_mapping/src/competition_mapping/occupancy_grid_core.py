#!/usr/bin/env python3
"""ROS-independent fixed-size 2D occupancy-grid mapping core."""

from __future__ import annotations

import math
from typing import Dict, Iterable, Iterator, List, Optional, Sequence, Tuple


GridCell = Tuple[int, int]


def transform_point_3d(
    point: Sequence[float],
    translation: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> Tuple[float, float, float]:
    """Apply a normalized quaternion rotation and translation to one xyz point."""
    x, y, z = map(float, point[:3])
    tx, ty, tz = map(float, translation[:3])
    qx, qy, qz, qw = map(float, quaternion_xyzw[:4])
    values = (x, y, z, tx, ty, tz, qx, qy, qz, qw)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("point transform contains NaN or Inf")
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1.0e-9:
        raise ValueError("transform quaternion has zero norm")
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm
    rx = (1-2*(qy*qy+qz*qz))*x + 2*(qx*qy-qz*qw)*y + 2*(qx*qz+qy*qw)*z
    ry = 2*(qx*qy+qz*qw)*x + (1-2*(qx*qx+qz*qz))*y + 2*(qy*qz-qx*qw)*z
    rz = 2*(qx*qz-qy*qw)*x + 2*(qy*qz+qx*qw)*y + (1-2*(qx*qx+qy*qy))*z
    return rx + tx, ry + ty, rz + tz


class OccupancyGridCore:
    """Integrate base-frame obstacle points into a fixed map-frame grid."""

    UNKNOWN = -1
    FREE = 0
    OCCUPIED = 100

    def __init__(
        self,
        resolution: float,
        width: int,
        height: int,
        origin_x: float,
        origin_y: float,
        min_range: float = 0.3,
        max_range: float = 20.0,
        min_height: float = -0.5,
        max_height: float = 1.5,
    ) -> None:
        self.resolution = float(resolution)
        self.width = int(width)
        self.height = int(height)
        self.origin_x = float(origin_x)
        self.origin_y = float(origin_y)
        self.min_range = float(min_range)
        self.max_range = float(max_range)
        self.min_height = float(min_height)
        self.max_height = float(max_height)
        self._validate_parameters()
        self.reset()

    def _validate_parameters(self) -> None:
        finite = math.isfinite
        if not finite(self.resolution) or self.resolution <= 0.0:
            raise ValueError("resolution must be finite and > 0")
        if self.width <= 0 or self.height <= 0:
            raise ValueError("width and height must be > 0")
        if not finite(self.origin_x) or not finite(self.origin_y):
            raise ValueError("map origin must be finite")
        if not finite(self.min_range) or self.min_range < 0.0:
            raise ValueError("min_range must be finite and >= 0")
        if not finite(self.max_range) or self.max_range <= self.min_range:
            raise ValueError("max_range must be finite and > min_range")
        if not finite(self.min_height) or not finite(self.max_height):
            raise ValueError("height limits must be finite")
        if self.max_height <= self.min_height:
            raise ValueError("max_height must be > min_height")

    def reset(self) -> None:
        """Set every cell to unknown."""
        self._data: List[int] = [self.UNKNOWN] * (self.width * self.height)

    def data_copy(self) -> List[int]:
        return list(self._data)

    def in_bounds(self, grid_x: int, grid_y: int) -> bool:
        return 0 <= grid_x < self.width and 0 <= grid_y < self.height

    def grid_to_index(self, grid_x: int, grid_y: int) -> int:
        if not self.in_bounds(grid_x, grid_y):
            raise IndexError("grid cell ({}, {}) is outside {}x{} map".format(
                grid_x, grid_y, self.width, self.height
            ))
        return grid_y * self.width + grid_x

    def get_cell(self, grid_x: int, grid_y: int) -> int:
        return self._data[self.grid_to_index(grid_x, grid_y)]

    def world_to_grid(self, world_x: float, world_y: float) -> Optional[GridCell]:
        if not math.isfinite(world_x) or not math.isfinite(world_y):
            return None
        cell = self.world_to_grid_unbounded(world_x, world_y)
        return cell if self.in_bounds(*cell) else None

    def world_to_grid_unbounded(self, world_x: float, world_y: float) -> GridCell:
        return (
            int(math.floor((world_x - self.origin_x) / self.resolution)),
            int(math.floor((world_y - self.origin_y) / self.resolution)),
        )

    def integrate_scan(
        self,
        robot_x: float,
        robot_y: float,
        robot_yaw: float,
        points_in_base: Iterable[Sequence[float]],
    ) -> Dict[str, int]:
        """Filter and integrate xyz points expressed in the robot base frame."""
        stats = {
            "input_points": 0,
            "valid_points": 0,
            "rejected_points": 0,
            "occupied_endpoints": 0,
            "free_cells_updated": 0,
            "changed_cells": 0,
            "robot_outside_map": 0,
        }
        if not all(math.isfinite(v) for v in (robot_x, robot_y, robot_yaw)):
            raise ValueError("robot pose contains NaN or Inf")

        robot_cell = self.world_to_grid(robot_x, robot_y)
        if robot_cell is None:
            stats["robot_outside_map"] = 1
            return stats

        cos_yaw = math.cos(robot_yaw)
        sin_yaw = math.sin(robot_yaw)
        for raw_point in points_in_base:
            stats["input_points"] += 1
            try:
                point_x, point_y, point_z = map(float, raw_point[:3])
            except (IndexError, TypeError, ValueError):
                stats["rejected_points"] += 1
                continue
            if not all(math.isfinite(v) for v in (point_x, point_y, point_z)):
                stats["rejected_points"] += 1
                continue
            point_range = math.hypot(point_x, point_y)
            if not self.min_range <= point_range <= self.max_range:
                stats["rejected_points"] += 1
                continue
            if not self.min_height <= point_z <= self.max_height:
                stats["rejected_points"] += 1
                continue

            stats["valid_points"] += 1
            world_x = robot_x + cos_yaw * point_x - sin_yaw * point_y
            world_y = robot_y + sin_yaw * point_x + cos_yaw * point_y
            endpoint = self.world_to_grid_unbounded(world_x, world_y)
            endpoint_inside = self.in_bounds(*endpoint)
            ray = self._bresenham(robot_cell[0], robot_cell[1], endpoint[0], endpoint[1])

            for cell in ray:
                if not self.in_bounds(*cell):
                    break
                if endpoint_inside and cell == endpoint:
                    break
                index = self.grid_to_index(*cell)
                # A later clear ray is newer evidence, so it clears stale obstacles.
                if self._data[index] != self.FREE:
                    self._data[index] = self.FREE
                    stats["changed_cells"] += 1
                stats["free_cells_updated"] += 1

            if endpoint_inside:
                index = self.grid_to_index(*endpoint)
                if self._data[index] != self.OCCUPIED:
                    self._data[index] = self.OCCUPIED
                    stats["changed_cells"] += 1
                stats["occupied_endpoints"] += 1

        return stats

    @staticmethod
    def _bresenham(start_x: int, start_y: int, end_x: int, end_y: int) -> Iterator[GridCell]:
        x, y = start_x, start_y
        dx = abs(end_x - start_x)
        sx = 1 if start_x < end_x else -1
        dy = -abs(end_y - start_y)
        sy = 1 if start_y < end_y else -1
        error = dx + dy
        while True:
            yield x, y
            if x == end_x and y == end_y:
                return
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x += sx
            if doubled <= dx:
                error += dx
                y += sy
