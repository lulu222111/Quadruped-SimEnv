#!/usr/bin/env python3
"""Pure OccupancyGrid frontier extraction algorithms.

This module deliberately has no ROS dependency so that the core algorithm can
be tested with small synthetic maps before it is connected to planning_node.
"""

from typing import List, Sequence, Tuple


GridCell = Tuple[int, int]


def extract_frontier_cells(
    data: Sequence[int],
    width: int,
    height: int,
    free_value: int = 0,
    unknown_value: int = -1,
) -> List[GridCell]:
    """Return free cells adjacent to at least one unknown cell.

    Coordinates are returned as ``(x, y)`` grid indices. A four-connected
    neighborhood is used for the first implementation.

    Args:
        data: Row-major OccupancyGrid cell values.
        width: Number of grid columns.
        height: Number of grid rows.
        free_value: Value representing a traversable cell.
        unknown_value: Value representing an unexplored cell.

    Raises:
        ValueError: If the dimensions or data length are invalid.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")

    expected_size = width * height
    if len(data) != expected_size:
        raise ValueError(
            "grid data length does not match width * height: "
            f"{len(data)} != {expected_size}"
        )

    frontier_cells: List[GridCell] = []
    neighbors = ((0, -1), (1, 0), (0, 1), (-1, 0))

    for y in range(height):
        for x in range(width):
            index = y * width + x
            if data[index] != free_value:
                continue

            for dx, dy in neighbors:
                neighbor_x = x + dx
                neighbor_y = y + dy

                if not (0 <= neighbor_x < width and 0 <= neighbor_y < height):
                    continue

                neighbor_index = neighbor_y * width + neighbor_x
                if data[neighbor_index] == unknown_value:
                    frontier_cells.append((x, y))
                    break

    return frontier_cells

def cluster_frontier_cells(frontier_cells, min_size=1):
    """Group 8-connected frontier cells and discard undersized groups."""
    if min_size < 1:
        raise ValueError("min_size must be at least 1")

    remaining = set(frontier_cells)
    clusters = []

    neighbor_offsets = (
        (-1, -1), (0, -1), (1, -1),
        (-1, 0),           (1, 0),
        (-1, 1),  (0, 1),  (1, 1),
    )

    while remaining:
        start = min(remaining)
        remaining.remove(start)

        stack = [start]
        cluster = []

        while stack:
            cell = stack.pop()
            cluster.append(cell)
            x, y = cell

            for dx, dy in neighbor_offsets:
                neighbor = (x + dx, y + dy)
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    stack.append(neighbor)

        cluster.sort()
        if len(cluster) >= min_size:
            clusters.append(cluster)

    clusters.sort(key=lambda cluster: cluster[0])
    return clusters

def select_frontier_representative(cluster):
    """Return the frontier cell nearest to the cluster centroid."""
    if not cluster:
        raise ValueError("cluster must not be empty")

    center_x = sum(cell[0] for cell in cluster) / float(len(cluster))
    center_y = sum(cell[1] for cell in cluster) / float(len(cluster))

    return min(
        cluster,
        key=lambda cell: (
            (cell[0] - center_x) ** 2 + (cell[1] - center_y) ** 2,
            cell[0],
            cell[1],
        ),
    )

def grid_cell_to_world(
    cell,
    resolution,
    origin_x=0.0,
    origin_y=0.0,
    origin_yaw=0.0,
):
    """Convert a grid cell to the world position at the cell center."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")

    from math import cos, sin

    cell_x, cell_y = cell

    local_x = (cell_x + 0.5) * resolution
    local_y = (cell_y + 0.5) * resolution

    world_x = (
        origin_x
        + cos(origin_yaw) * local_x
        - sin(origin_yaw) * local_y
    )
    world_y = (
        origin_y
        + sin(origin_yaw) * local_x
        + cos(origin_yaw) * local_y
    )

    return world_x, world_y