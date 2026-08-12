#!/usr/bin/env python3

"""ROS-independent exploration planning pipeline."""

from collections import deque
from math import atan2

from frontier import (
    cluster_frontier_cells,
    extract_frontier_cells,
    select_best_frontier_cluster,
    select_frontier_representative,
)
from path_planner import inflate_obstacles


PLAN_SUCCESS = "success"
PLAN_NO_FRONTIER = "no_frontier"
PLAN_UNREACHABLE = "unreachable"


def path_cell_yaws(path, origin_yaw=0.0):
    """
    Return one world-frame yaw for every grid path cell.

    Each pose faces the next path cell. The final pose keeps the
    direction of the final path segment.
    """
    if not path:
        return []

    if len(path) == 1:
        return [origin_yaw]

    yaws = []

    for index in range(len(path) - 1):
        current_x, current_y = path[index]
        next_x, next_y = path[index + 1]

        yaws.append(
            origin_yaw
            + atan2(
                next_y - current_y,
                next_x - current_x,
            )
        )

    yaws.append(yaws[-1])
    return yaws


def order_frontier_candidates(cluster, robot_cell):
    """
    Order candidate cells within one Frontier cluster.

    The centroid representative remains the first candidate. Remaining
    cells are ordered by centroid distance, robot distance, and grid
    coordinates.
    """
    if not cluster:
        return []

    representative = select_frontier_representative(
        cluster
    )

    center_x = (
        sum(cell[0] for cell in cluster)
        / len(cluster)
    )
    center_y = (
        sum(cell[1] for cell in cluster)
        / len(cluster)
    )

    def candidate_key(cell):
        centroid_distance_squared = (
            (cell[0] - center_x) ** 2
            + (cell[1] - center_y) ** 2
        )
        robot_distance_squared = (
            (cell[0] - robot_cell[0]) ** 2
            + (cell[1] - robot_cell[1]) ** 2
        )

        return (
            centroid_distance_squared,
            robot_distance_squared,
            cell[1],
            cell[0],
        )

    remaining_candidates = sorted(
        (
            cell
            for cell in cluster
            if cell != representative
        ),
        key=candidate_key,
    )

    return [
        representative,
        *remaining_candidates,
    ]


def _bfs_reachability(
    grid,
    width,
    height,
    start,
    obstacle_threshold=50,
):
    """
    Single-source BFS over a 4-connected occupancy grid.

    Returns a ``parent`` dict that maps each reachable cell to the
    neighbour from which it was first discovered.  The start cell maps
    to ``None``.  Cells absent from the dict are unreachable.

    Paths reconstructed by following parent pointers are shortest paths
    under unit edge weights.
    """
    parent = {}

    sx, sy = start
    if not (0 <= sx < width and 0 <= sy < height):
        return parent

    start_value = grid[sy * width + sx]
    if not (0 <= start_value < obstacle_threshold):
        return parent

    parent[start] = None
    queue = deque([start])

    neighbour_offsets = (
        (0, -1),
        (-1, 0),
        (1, 0),
        (0, 1),
    )

    while queue:
        cx, cy = queue.popleft()

        for dx, dy in neighbour_offsets:
            nx, ny = cx + dx, cy + dy

            if not (0 <= nx < width and 0 <= ny < height):
                continue

            neighbour = (nx, ny)
            if neighbour in parent:
                continue

            value = grid[ny * width + nx]
            if not (0 <= value < obstacle_threshold):
                continue

            parent[neighbour] = (cx, cy)
            queue.append(neighbour)

    return parent


def plan_to_frontier_with_status(
    data,
    width,
    height,
    robot_cell,
    min_frontier_size=5,
    inflation_radius_cells=0,
    size_weight=1.0,
    distance_weight=1.0,
    obstacle_threshold=50,
):
    """
    Select a reachable frontier and report an explicit planning status.

    Returns:
        A dictionary whose ``status`` is one of:

        - ``PLAN_SUCCESS``: a reachable frontier and path were found.
        - ``PLAN_NO_FRONTIER``: no usable frontier cluster exists.
        - ``PLAN_UNREACHABLE``: Frontier clusters exist, but none is
          reachable.
    """
    frontier_cells = extract_frontier_cells(
        data,
        width,
        height,
    )
    clusters = cluster_frontier_cells(
        frontier_cells,
        min_size=min_frontier_size,
    )

    if not clusters:
        return {
            "status": PLAN_NO_FRONTIER,
            "goal_cell": None,
            "path": None,
            "cluster": None,
            "frontier_count": len(frontier_cells),
        }

    planning_grid = inflate_obstacles(
        data,
        width,
        height,
        radius_cells=inflation_radius_cells,
        obstacle_threshold=obstacle_threshold,
    )

    robot_x, robot_y = robot_cell

    if (
        0 <= robot_x < width
        and 0 <= robot_y < height
    ):
        robot_index = (
            robot_y * width
            + robot_x
        )
        original_robot_value = data[robot_index]

        if (
            0 <= original_robot_value
            < obstacle_threshold
        ):
            planning_grid[robot_index] = (
                original_robot_value
            )

    remaining_clusters = list(clusters)

    parent = _bfs_reachability(
        planning_grid,
        width,
        height,
        start=robot_cell,
        obstacle_threshold=obstacle_threshold,
    )

    while remaining_clusters:
        selected_cluster = select_best_frontier_cluster(
            remaining_clusters,
            robot_cell=robot_cell,
            size_weight=size_weight,
            distance_weight=distance_weight,
        )

        if selected_cluster is None:
            break

        candidates = order_frontier_candidates(
            selected_cluster,
            robot_cell,
        )

        for goal_cell in candidates:
            if goal_cell in parent:
                path = []
                cell = goal_cell
                while cell is not None:
                    path.append(cell)
                    cell = parent[cell]
                path.reverse()

                return {
                    "status": PLAN_SUCCESS,
                    "goal_cell": goal_cell,
                    "path": path,
                    "cluster": selected_cluster,
                    "frontier_count": len(
                        frontier_cells
                    ),
                }

        remaining_clusters.remove(
            selected_cluster
        )

    return {
        "status": PLAN_UNREACHABLE,
        "goal_cell": None,
        "path": None,
        "cluster": None,
        "frontier_count": len(frontier_cells),
    }


def plan_to_frontier(
    data,
    width,
    height,
    robot_cell,
    min_frontier_size=5,
    inflation_radius_cells=0,
    size_weight=1.0,
    distance_weight=1.0,
    obstacle_threshold=50,
):
    """
    Select a reachable frontier and plan a collision-free grid path.

    Returns a planning dictionary when successful and ``None`` when no
    reachable Frontier exists.
    """
    result = plan_to_frontier_with_status(
        data,
        width,
        height,
        robot_cell,
        min_frontier_size=min_frontier_size,
        inflation_radius_cells=inflation_radius_cells,
        size_weight=size_weight,
        distance_weight=distance_weight,
        obstacle_threshold=obstacle_threshold,
    )

    if result["status"] != PLAN_SUCCESS:
        return None

    return {
        "goal_cell": result["goal_cell"],
        "path": result["path"],
        "cluster": result["cluster"],
        "frontier_count": result["frontier_count"],
    }