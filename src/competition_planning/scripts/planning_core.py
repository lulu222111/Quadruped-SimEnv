#!/usr/bin/env python3

"""ROS-independent exploration planning pipeline."""

from frontier import (
    cluster_frontier_cells,
    extract_frontier_cells,
    select_best_frontier_cluster,
    select_frontier_representative,
)
from path_planner import astar_path, inflate_obstacles


PLAN_SUCCESS = "success"
PLAN_NO_FRONTIER = "no_frontier"
PLAN_UNREACHABLE = "unreachable"


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
        - ``PLAN_UNREACHABLE``: frontier clusters exist, but none is reachable.
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

    remaining_clusters = list(clusters)

    while remaining_clusters:
        selected_cluster = select_best_frontier_cluster(
            remaining_clusters,
            robot_cell=robot_cell,
            size_weight=size_weight,
            distance_weight=distance_weight,
        )

        if selected_cluster is None:
            break

        goal_cell = select_frontier_representative(selected_cluster)
        path = astar_path(
            planning_grid,
            width,
            height,
            start=robot_cell,
            goal=goal_cell,
            obstacle_threshold=obstacle_threshold,
        )

        if path is not None:
            return {
                "status": PLAN_SUCCESS,
                "goal_cell": goal_cell,
                "path": path,
                "cluster": selected_cluster,
                "frontier_count": len(frontier_cells),
            }

        remaining_clusters.remove(selected_cluster)

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

    This compatibility interface keeps the original behavior:

    - returns a planning dictionary when successful;
    - returns ``None`` when no reachable frontier exists.
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