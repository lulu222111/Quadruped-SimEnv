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

def select_best_frontier_cluster(
    clusters,
    robot_cell,
    size_weight=1.0,
    distance_weight=1.0,
    heading_yaw=None,
    heading_weight=0.0,
    room_bias_weight=0.0,
    room_bias_center_cells=0.0,
    cluster_room_dist=None,
):
    """Select a frontier cluster by size, distance, heading, and room bias.

    ``heading_weight`` uses grid-cell score units.  Zero preserves the
    original selector; a positive value favours the robot's forward
    half-plane without using scene or hazard truth.

    ``room_bias_weight`` adds a bonus that is CENTRED on
    ``room_bias_center_cells``:

        bonus = room_bias_weight * (dist_from_wall - room_bias_center_cells)

    ``cluster_room_dist`` is an optional dict keyed by each cluster's first
    cell (clusters are internally sorted, so the first cell is a stable
    per-cluster key) mapping to that cluster's distance to the nearest wall
    (cells).  Frontiers whose rep sits far from walls (wide rooms / lobby
    interiors, e.g. dist ~ 40+ cells at 0.1 m) get a POSITIVE bonus, while
    frontiers in narrow false spaces (corridors / elevator shafts /
    stairwells, e.g. dist ~ 11-16 cells at 0.1 m) get a NEGATIVE bonus when
    their dist is below the centre.  With ``room_bias_center_cells = 0`` the
    term reduces to the original linear ``weight * dist`` behaviour.
    Ignored when ``room_bias_weight`` is 0 or ``cluster_room_dist`` is None.
    """
    if size_weight < 0.0:
        raise ValueError("size_weight must not be negative")
    if distance_weight < 0.0:
        raise ValueError("distance_weight must not be negative")
    if heading_weight < 0.0:
        raise ValueError("heading_weight must not be negative")
    if room_bias_weight < 0.0:
        raise ValueError("room_bias_weight must not be negative")
    if room_bias_center_cells < 0.0:
        raise ValueError("room_bias_center_cells must not be negative")
    if not clusters:
        return None

    from math import cos, hypot, sin

    robot_x, robot_y = robot_cell
    candidates = []

    for cluster in clusters:
        if not cluster:
            continue

        representative = select_frontier_representative(cluster)
        distance = hypot(
            representative[0] - robot_x,
            representative[1] - robot_y,
        )
        heading_alignment = 0.0
        if heading_yaw is not None and distance > 0.0:
            heading_alignment = (
                cos(heading_yaw) * (representative[0] - robot_x)
                + sin(heading_yaw) * (representative[1] - robot_y)
            ) / distance
        room_bonus = 0.0
        if room_bias_weight > 0.0 and cluster_room_dist is not None:
            dist_from_wall = cluster_room_dist.get(cluster[0], 0.0)
            room_bonus = room_bias_weight * (
                dist_from_wall - room_bias_center_cells
            )
        score = (
            size_weight * len(cluster)
            - distance_weight * distance
            + heading_weight * heading_alignment
            + room_bonus
        )

        candidates.append(
            (
                -score,
                distance,
                representative[0],
                representative[1],
                cluster,
            )
        )

    if not candidates:
        return None

    candidates.sort()
    return candidates[0][4]

def world_to_grid_cell(
    world_position,
    resolution,
    origin_x=0.0,
    origin_y=0.0,
    origin_yaw=0.0,
):
    """Convert a world position to its containing occupancy-grid cell."""
    if resolution <= 0.0:
        raise ValueError("resolution must be positive")

    from math import cos, floor, sin

    world_x, world_y = world_position
    delta_x = world_x - origin_x
    delta_y = world_y - origin_y

    # Apply the inverse rotation of the map origin.
    local_x = (
        cos(origin_yaw) * delta_x
        + sin(origin_yaw) * delta_y
    )
    local_y = (
        -sin(origin_yaw) * delta_x
        + cos(origin_yaw) * delta_y
    )

    cell_x = int(floor(local_x / resolution))
    cell_y = int(floor(local_y / resolution))

    return cell_x, cell_y
