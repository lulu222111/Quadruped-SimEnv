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