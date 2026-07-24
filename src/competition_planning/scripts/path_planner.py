#!/usr/bin/env python3

"""Grid-based A* path planning without ROS dependencies."""

from heapq import heappop, heappush


def astar_path(
    data,
    width,
    height,
    start,
    goal,
    obstacle_threshold=50,
):
    """
    Find a collision-free, four-connected path on an occupancy grid.

    Occupancy values:
    - -1: unknown and not traversable
    - 0..obstacle_threshold-1: traversable
    - obstacle_threshold or greater: obstacle

    Returns a list from start to goal, or None when no path exists.
    """
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if len(data) != width * height:
        raise ValueError("grid data length does not match width * height")
    if obstacle_threshold <= 0:
        raise ValueError("obstacle_threshold must be positive")

    def is_inside(cell):
        x, y = cell
        return 0 <= x < width and 0 <= y < height

    def occupancy(cell):
        x, y = cell
        return data[y * width + x]

    def is_traversable(cell):
        if not is_inside(cell):
            return False
        value = occupancy(cell)
        return 0 <= value < obstacle_threshold

    if not is_inside(start):
        raise ValueError("start is outside the grid")
    if not is_inside(goal):
        raise ValueError("goal is outside the grid")
    if not is_traversable(start) or not is_traversable(goal):
        return None
    if start == goal:
        return [start]

    def heuristic(cell):
        return abs(cell[0] - goal[0]) + abs(cell[1] - goal[1])

    open_heap = []
    heappush(open_heap, (heuristic(start), 0, start))

    came_from = {}
    best_cost = {start: 0}
    closed = set()

    neighbor_offsets = (
        (0, -1),
        (-1, 0),
        (1, 0),
        (0, 1),
    )

    while open_heap:
        _, current_cost, current = heappop(open_heap)

        if current in closed:
            continue
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path

        closed.add(current)
        current_x, current_y = current

        for dx, dy in neighbor_offsets:
            neighbor = (current_x + dx, current_y + dy)

            if not is_traversable(neighbor):
                continue
            if neighbor in closed:
                continue

            new_cost = current_cost + 1
            if new_cost >= best_cost.get(neighbor, float("inf")):
                continue

            best_cost[neighbor] = new_cost
            came_from[neighbor] = current
            estimated_total = new_cost + heuristic(neighbor)
            heappush(
                open_heap,
                (estimated_total, new_cost, neighbor),
            )

    return None