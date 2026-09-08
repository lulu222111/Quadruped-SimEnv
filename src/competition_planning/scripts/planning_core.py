#!/usr/bin/env python3

"""ROS-independent exploration planning pipeline."""

from collections import deque
from math import atan2, cos, hypot, sin

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


def compress_collinear_path(path):
    """Remove only interior cells on straight grid runs."""
    if len(path) <= 2:
        return list(path)
    compressed = [path[0]]
    previous_direction = (
        path[1][0] - path[0][0],
        path[1][1] - path[0][1],
    )
    for index in range(1, len(path) - 1):
        next_direction = (
            path[index + 1][0] - path[index][0],
            path[index + 1][1] - path[index][1],
        )
        if next_direction != previous_direction:
            compressed.append(path[index])
        previous_direction = next_direction
    compressed.append(path[-1])
    return compressed


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


def order_frontier_candidates(
    cluster,
    robot_cell,
    heading_yaw=None,
    heading_weight=0.0,
):
    """
    Order candidate cells within one Frontier cluster.

    The centroid representative remains the first candidate. Remaining
    cells are ordered by centroid distance, robot distance, and grid
    coordinates.
    """
    if not cluster:
        return []
    if heading_weight < 0.0:
        raise ValueError("heading_weight must not be negative")

    # A frontier can coincide with the current free cell at map startup.
    # Such a goal creates a one-pose path and permanent zero command, so it
    # has no exploration value.
    eligible = (
        [cell for cell in cluster if cell != robot_cell]
        if heading_yaw is not None and heading_weight > 0.0
        else list(cluster)
    )
    if not eligible:
        return []

    representative = select_frontier_representative(eligible)

    center_x = (
        sum(cell[0] for cell in eligible)
        / len(eligible)
    )
    center_y = (
        sum(cell[1] for cell in eligible)
        / len(eligible)
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
        robot_distance = hypot(
            cell[0] - robot_cell[0],
            cell[1] - robot_cell[1],
        )
        heading_alignment = 0.0
        if heading_yaw is not None and robot_distance > 0.0:
            heading_alignment = (
                cos(heading_yaw) * (cell[0] - robot_cell[0])
                + sin(heading_yaw) * (cell[1] - robot_cell[1])
            ) / robot_distance

        return (
            robot_distance - heading_weight * heading_alignment,
            centroid_distance_squared,
            robot_distance_squared,
            cell[1],
            cell[0],
        )

    if heading_yaw is not None and heading_weight > 0.0:
        return sorted(eligible, key=candidate_key)

    remaining_candidates = sorted(
        (
            cell
            for cell in eligible
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


def _find_recovery_start(
    data,
    planning_grid,
    width,
    height,
    robot_cell,
    radius_cells,
    obstacle_threshold=50,
):
    """Return the nearest 4-connected raw/inflated free start cell."""
    if radius_cells < 1:
        return None

    robot_x, robot_y = robot_cell
    candidates = []

    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            distance = abs(dx) + abs(dy)
            if distance == 0 or distance > radius_cells:
                continue

            candidate_x = robot_x + dx
            candidate_y = robot_y + dy
            if not (
                0 <= candidate_x < width
                and 0 <= candidate_y < height
            ):
                continue

            candidate_index = candidate_y * width + candidate_x
            if (
                0 <= data[candidate_index] < obstacle_threshold
                and 0 <= planning_grid[candidate_index] < obstacle_threshold
            ):
                candidates.append(
                    (distance, candidate_y, candidate_x)
                )

    if not candidates:
        return None

    _, candidate_y, candidate_x = min(candidates)
    return candidate_x, candidate_y


def _nearest_reachable_standoff(
    parent,
    frontier_cell,
    radius_cells,
    planning_start,
    minimum_path_length_cells=0,
):
    """Return a safe reachable cell near a frontier blocked by inflation.

    A standoff can be geometrically valid while being so close to the current
    start that the path follower treats every pose as already reached.  The
    optional minimum path length lets the caller express that controller
    contract without changing the ordinary exact-frontier planner.
    """
    if radius_cells < 1:
        return None
    if minimum_path_length_cells < 0:
        raise ValueError(
            "minimum_path_length_cells must not be negative"
        )
    frontier_x, frontier_y = frontier_cell
    candidates = []
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            distance = abs(dx) + abs(dy)
            if distance < 1 or distance > radius_cells:
                continue
            candidate = (frontier_x + dx, frontier_y + dy)
            if candidate in parent and candidate != planning_start:
                candidates.append((distance, candidate[1], candidate[0]))
    for _, candidate_y, candidate_x in sorted(candidates):
        candidate = candidate_x, candidate_y
        path_length_cells = 0
        cell = candidate
        while cell is not None:
            path_length_cells += 1
            cell = parent[cell]
        path_length_cells -= 1
        if path_length_cells < minimum_path_length_cells:
            continue
        return candidate
    return None


def plan_to_goal_with_status(
    data,
    width,
    height,
    robot_cell,
    goal_cell,
    inflation_radius_cells=0,
    start_recovery_radius_cells=0,
    obstacle_threshold=50,
):
    """Plan a shortest inflated-grid path to a fixed legal goal cell.

    This is the return-mode counterpart to frontier planning.  The goal must
    be in-bounds, raw-free, and free in the inflated grid.  An unknown robot
    start may use the existing finite recovery radius; occupied and
    out-of-bounds starts remain unreachable.
    """
    if start_recovery_radius_cells < 0:
        raise ValueError(
            "start_recovery_radius_cells must not be negative"
        )

    planning_grid = inflate_obstacles(
        data,
        width,
        height,
        radius_cells=inflation_radius_cells,
        obstacle_threshold=obstacle_threshold,
    )

    goal_x, goal_y = goal_cell
    if not (0 <= goal_x < width and 0 <= goal_y < height):
        return {
            "status": PLAN_UNREACHABLE,
            "goal_cell": goal_cell,
            "path": None,
            "start_recovery_used": False,
            "start_recovery_cell": None,
            "detail": "return goal is out of bounds",
        }

    goal_index = goal_y * width + goal_x
    if (
        not (0 <= data[goal_index] < obstacle_threshold)
        or not (0 <= planning_grid[goal_index] < obstacle_threshold)
    ):
        return {
            "status": PLAN_UNREACHABLE,
            "goal_cell": goal_cell,
            "path": None,
            "start_recovery_used": False,
            "start_recovery_cell": None,
            "detail": "return goal is occupied or inflated",
        }

    robot_x, robot_y = robot_cell
    start_recovery_used = False
    start_recovery_cell = None
    planning_start = robot_cell
    robot_cell_raw_value = None
    robot_cell_inflated_value = None

    if 0 <= robot_x < width and 0 <= robot_y < height:
        robot_index = robot_y * width + robot_x
        robot_cell_raw_value = data[robot_index]
        robot_cell_inflated_value = planning_grid[robot_index]
        if 0 <= robot_cell_raw_value < obstacle_threshold:
            planning_grid[robot_index] = robot_cell_raw_value
            if robot_cell_inflated_value >= obstacle_threshold:
                start_recovery_cell = _find_recovery_start(
                    data,
                    planning_grid,
                    width,
                    height,
                    robot_cell,
                    radius_cells=start_recovery_radius_cells,
                    obstacle_threshold=obstacle_threshold,
                )
                if start_recovery_cell is not None:
                    planning_start = start_recovery_cell
                    start_recovery_used = True
        elif robot_cell_raw_value == -1:
            start_recovery_cell = _find_recovery_start(
                data,
                planning_grid,
                width,
                height,
                robot_cell,
                radius_cells=start_recovery_radius_cells,
                obstacle_threshold=obstacle_threshold,
            )
            if start_recovery_cell is not None:
                planning_start = start_recovery_cell
                start_recovery_used = True

    parent = _bfs_reachability(
        planning_grid,
        width,
        height,
        start=planning_start,
        obstacle_threshold=obstacle_threshold,
    )
    if goal_cell not in parent:
        return {
            "status": PLAN_UNREACHABLE,
            "goal_cell": goal_cell,
            "path": None,
            "start_recovery_used": start_recovery_used,
            "start_recovery_cell": start_recovery_cell,
            "robot_cell_raw_value": robot_cell_raw_value,
            "robot_cell_inflated_value": robot_cell_inflated_value,
            "detail": "return goal is unreachable",
        }

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
        "start_recovery_used": start_recovery_used,
        "start_recovery_cell": start_recovery_cell,
        "robot_cell_raw_value": robot_cell_raw_value,
        "robot_cell_inflated_value": robot_cell_inflated_value,
    }


def plan_to_goal_region_with_status(
    data,
    width,
    height,
    robot_cell,
    goal_cell,
    goal_search_radius_cells=0,
    **planning_kwargs,
):
    """Plan to an exact goal or the nearest reachable cell in its region."""
    if goal_search_radius_cells < 0:
        raise ValueError("goal_search_radius_cells must not be negative")
    candidates = [(0, goal_cell[1], goal_cell[0], goal_cell)]
    for dy in range(-goal_search_radius_cells, goal_search_radius_cells + 1):
        for dx in range(-goal_search_radius_cells, goal_search_radius_cells + 1):
            distance = abs(dx) + abs(dy)
            if distance == 0 or distance > goal_search_radius_cells:
                continue
            candidate = (goal_cell[0] + dx, goal_cell[1] + dy)
            candidates.append((distance, candidate[1], candidate[0], candidate))
    last_result = None
    for _, _, _, candidate in sorted(candidates):
        result = plan_to_goal_with_status(
            data,
            width,
            height,
            robot_cell,
            candidate,
            **planning_kwargs,
        )
        if result["status"] == PLAN_SUCCESS:
            result["requested_goal_cell"] = goal_cell
            result["goal_offset_used"] = candidate != goal_cell
            return result
        last_result = result
    last_result["requested_goal_cell"] = goal_cell
    last_result["goal_offset_used"] = False
    return last_result


def plan_to_heading_goal_with_status(
    data,
    width,
    height,
    robot_cell,
    heading_yaw,
    max_forward_cells,
    min_forward_cells=1.0,
    corridor_half_width_cells=10.0,
    inflation_radius_cells=0,
    start_recovery_radius_cells=0,
    obstacle_threshold=50,
    heading_reference_cell=None,
):
    """Plan to the furthest reachable free cell in a heading corridor.

    By default the corridor is measured from ``robot_cell``.  A caller that
    is maintaining a fixed heading anchor may provide
    ``heading_reference_cell``; the BFS still starts at ``robot_cell``, but
    candidate forward/lateral offsets are then measured from that fixed cell.
    This keeps replanning from moving the corridor itself as the robot moves.
    """
    if max_forward_cells <= 0.0:
        raise ValueError("max_forward_cells must be positive")
    if min_forward_cells <= 0.0:
        raise ValueError("min_forward_cells must be positive")
    if corridor_half_width_cells < 0.0:
        raise ValueError("corridor_half_width_cells must not be negative")

    planning_grid = inflate_obstacles(
        data,
        width,
        height,
        radius_cells=inflation_radius_cells,
        obstacle_threshold=obstacle_threshold,
    )
    robot_x, robot_y = robot_cell
    planning_start = robot_cell
    start_recovery_used = False
    start_recovery_cell = None
    robot_cell_raw_value = None
    robot_cell_inflated_value = None
    if 0 <= robot_x < width and 0 <= robot_y < height:
        robot_index = robot_y * width + robot_x
        robot_cell_raw_value = data[robot_index]
        robot_cell_inflated_value = planning_grid[robot_index]
        if 0 <= robot_cell_raw_value < obstacle_threshold:
            planning_grid[robot_index] = robot_cell_raw_value
            if robot_cell_inflated_value >= obstacle_threshold:
                start_recovery_cell = _find_recovery_start(
                    data,
                    planning_grid,
                    width,
                    height,
                    robot_cell,
                    radius_cells=start_recovery_radius_cells,
                    obstacle_threshold=obstacle_threshold,
                )
                if start_recovery_cell is not None:
                    planning_start = start_recovery_cell
                    start_recovery_used = True
        elif robot_cell_raw_value == -1:
            start_recovery_cell = _find_recovery_start(
                data,
                planning_grid,
                width,
                height,
                robot_cell,
                radius_cells=start_recovery_radius_cells,
                obstacle_threshold=obstacle_threshold,
            )
            if start_recovery_cell is not None:
                planning_start = start_recovery_cell
                start_recovery_used = True

    parent = _bfs_reachability(
        planning_grid,
        width,
        height,
        start=planning_start,
        obstacle_threshold=obstacle_threshold,
    )
    direction_x = cos(heading_yaw)
    direction_y = sin(heading_yaw)
    if heading_reference_cell is None:
        heading_reference_cell = robot_cell
    reference_x, reference_y = heading_reference_cell
    candidates = []
    for cell in parent:
        dx = cell[0] - reference_x
        dy = cell[1] - reference_y
        forward = direction_x * dx + direction_y * dy
        lateral = abs(-direction_y * dx + direction_x * dy)
        if (
            min_forward_cells <= forward <= max_forward_cells
            and lateral <= corridor_half_width_cells
        ):
            candidates.append((-forward, lateral, dx * dx + dy * dy, cell[1], cell[0], cell))

    diagnostics = {
        "robot_cell": robot_cell,
        "robot_cell_raw_value": robot_cell_raw_value,
        "robot_cell_inflated_value": robot_cell_inflated_value,
        "start_recovery_used": start_recovery_used,
        "start_recovery_cell": start_recovery_cell,
        "reachable_cell_count": len(parent),
        "inflation_radius_cells": inflation_radius_cells,
        "heading_reference_cell": heading_reference_cell,
    }
    if not candidates:
        return {
            "status": PLAN_UNREACHABLE,
            "goal_cell": None,
            "path": None,
            "detail": "no reachable free cell in heading corridor",
            **diagnostics,
        }

    goal_cell = min(candidates)[-1]
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
        **diagnostics,
    }


def plan_to_frontier_with_status(
    data,
    width,
    height,
    robot_cell,
    min_frontier_size=5,
    inflation_radius_cells=0,
    start_recovery_radius_cells=0,
    frontier_goal_search_radius_cells=0,
    size_weight=1.0,
    distance_weight=1.0,
    heading_yaw=None,
    heading_weight=0.0,
    minimum_standoff_path_length_cells=0,
    obstacle_threshold=50,
    preferred_frontier_cell=None,
    room_bias_weight=0.0,
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

    # Room-bias distance field (only when requested): distance to nearest
    # wall for every free/unknown cell.  Frontiers inside wide rooms sit far
    # from walls (high value) while corridor / elevator-shaft / stairwell
    # frontiers sit close to walls (low value).  Keyed by each cluster's
    # first cell (clusters are sorted internally) so the selector can look
    # up the room distance for whichever subset it is asked about.
    cluster_room_dist = None
    if room_bias_weight > 0.0:
        from room_access import distance_to_wall

        dist_field = distance_to_wall(
            data, width, height, occ_thresh=obstacle_threshold
        )
        cluster_room_dist = {}
        for cluster in clusters:
            if not cluster:
                continue
            rep = select_frontier_representative(cluster)
            cluster_room_dist[cluster[0]] = float(
                dist_field[rep[1], rep[0]]
            )

    planning_grid = inflate_obstacles(
        data,
        width,
        height,
        radius_cells=inflation_radius_cells,
        obstacle_threshold=obstacle_threshold,
    )

    if start_recovery_radius_cells < 0:
        raise ValueError(
            "start_recovery_radius_cells must not be negative"
        )
    if frontier_goal_search_radius_cells < 0:
        raise ValueError(
            "frontier_goal_search_radius_cells must not be negative"
        )
    if minimum_standoff_path_length_cells < 0:
        raise ValueError(
            "minimum_standoff_path_length_cells must not be negative"
        )

    robot_x, robot_y = robot_cell
    robot_cell_raw_value = None
    robot_cell_inflated_value = None
    start_recovery_used = False
    start_recovery_cell = None
    planning_start = robot_cell

    if (
        0 <= robot_x < width
        and 0 <= robot_y < height
    ):
        robot_index = (
            robot_y * width
            + robot_x
        )
        original_robot_value = data[robot_index]
        robot_cell_raw_value = original_robot_value
        robot_cell_inflated_value = planning_grid[robot_index]

        if (
            0 <= original_robot_value
            < obstacle_threshold
        ):
            planning_grid[robot_index] = (
                original_robot_value
            )
            if robot_cell_inflated_value >= obstacle_threshold:
                start_recovery_cell = _find_recovery_start(
                    data,
                    planning_grid,
                    width,
                    height,
                    robot_cell,
                    radius_cells=start_recovery_radius_cells,
                    obstacle_threshold=obstacle_threshold,
                )
                if start_recovery_cell is not None:
                    planning_start = start_recovery_cell
                    start_recovery_used = True
        elif original_robot_value == -1:
            start_recovery_cell = _find_recovery_start(
                data,
                planning_grid,
                width,
                height,
                robot_cell,
                radius_cells=start_recovery_radius_cells,
                obstacle_threshold=obstacle_threshold,
            )
            if start_recovery_cell is not None:
                planning_start = start_recovery_cell
                start_recovery_used = True

    if preferred_frontier_cell is None:
        remaining_clusters = list(clusters)
    else:
        # A visual-checkpoint continuation may need a different arrival cell
        # after the map marks the previously chosen standoff as occupied or
        # inflated.  Keep the frontier objective in that case: selecting a
        # cluster that contains the recorded frontier cell lets the existing
        # candidate/standoff logic choose another legal cell in the same
        # frontier region.  If the recorded cell is no longer a frontier,
        # return UNREACHABLE so the caller can release the stale objective
        # instead of silently switching to an unrelated cluster.
        preferred_cluster = next(
            (
                cluster for cluster in clusters
                if preferred_frontier_cell in cluster
            ),
            None,
        )
        remaining_clusters = (
            [preferred_cluster] if preferred_cluster is not None else []
        )

    parent = _bfs_reachability(
        planning_grid,
        width,
        height,
        start=planning_start,
        obstacle_threshold=obstacle_threshold,
    )
    reachable_frontier_count = sum(
        1
        for frontier_cell in frontier_cells
        if frontier_cell in parent
    )
    diagnostics = {
        "cluster_count": len(clusters),
        "reachable_cell_count": len(parent),
        "reachable_frontier_count": reachable_frontier_count,
        "robot_cell": robot_cell,
        "robot_cell_raw_value": robot_cell_raw_value,
        "robot_cell_inflated_value": robot_cell_inflated_value,
        "start_recovery_used": start_recovery_used,
        "start_recovery_cell": start_recovery_cell,
        "inflation_radius_cells": inflation_radius_cells,
        "minimum_standoff_path_length_cells": (
            minimum_standoff_path_length_cells
        ),
    }

    while remaining_clusters:
        selected_cluster = select_best_frontier_cluster(
            remaining_clusters,
            robot_cell=robot_cell,
            size_weight=size_weight,
            distance_weight=distance_weight,
            heading_yaw=heading_yaw,
            heading_weight=heading_weight,
            room_bias_weight=room_bias_weight,
            cluster_room_dist=cluster_room_dist,
        )

        if selected_cluster is None:
            break

        candidates = order_frontier_candidates(
            selected_cluster,
            robot_cell,
            heading_yaw=heading_yaw,
            heading_weight=heading_weight,
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
                    **diagnostics,
                    "frontier_goal_cell": goal_cell,
                    "frontier_goal_offset_used": False,
                }

        for frontier_cell in candidates:
            goal_cell = _nearest_reachable_standoff(
                parent,
                frontier_cell,
                frontier_goal_search_radius_cells,
                planning_start,
                minimum_path_length_cells=(
                    minimum_standoff_path_length_cells
                ),
            )
            if goal_cell is None:
                continue
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
                "frontier_count": len(frontier_cells),
                **diagnostics,
                "frontier_goal_cell": frontier_cell,
                "frontier_goal_offset_used": True,
            }

        remaining_clusters.remove(
            selected_cluster
        )

    # Even when no frontier is reachable, report the frontier cell nearest to
    # the robot.  A recovery layer (planning_node exploration nudge) uses it
    # to command a short blind drive toward unexplored space so the robot
    # re-observes and the map can correct a drifted/enclosed reachability
    # pocket (R29 planner-BLOCKED deadlock).
    nearest_frontier_cell = None
    nearest_distance_sq = None
    for frontier_cell in frontier_cells:
        d_sq = (
            (frontier_cell[0] - robot_x) ** 2
            + (frontier_cell[1] - robot_y) ** 2
        )
        if nearest_distance_sq is None or d_sq < nearest_distance_sq:
            nearest_distance_sq = d_sq
            nearest_frontier_cell = frontier_cell
    diagnostics["nearest_frontier_cell"] = nearest_frontier_cell

    return {
        "status": PLAN_UNREACHABLE,
        "goal_cell": None,
        "path": None,
        "cluster": None,
        "frontier_count": len(frontier_cells),
        **diagnostics,
    }


def plan_to_frontier(
    data,
    width,
    height,
    robot_cell,
    min_frontier_size=5,
    inflation_radius_cells=0,
    start_recovery_radius_cells=0,
    frontier_goal_search_radius_cells=0,
    size_weight=1.0,
    distance_weight=1.0,
    heading_yaw=None,
    heading_weight=0.0,
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
        start_recovery_radius_cells=start_recovery_radius_cells,
        frontier_goal_search_radius_cells=frontier_goal_search_radius_cells,
        size_weight=size_weight,
        distance_weight=distance_weight,
        heading_yaw=heading_yaw,
        heading_weight=heading_weight,
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
