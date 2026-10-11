#!/usr/bin/env python3

# =============================================================================
# 中文说明（模块级）：path_planner.py —— 与 ROS 无关的栅格 A* 寻路
#
# 职责：在占据栅格上做四连通 A*（含障碍膨胀），返回格路径或 None。
# 为什么先膨胀：机器人有体积，贴着障碍搜出的路径会刮墙；先用机身半径膨胀把
#       "墙 ± 半径"标为不可通行，再在膨胀图上搜索。膨胀时刻意保留 unknown。
# 实现要点：closed 集做惰性删除（堆中可能残留旧条目，弹出时判重）；启发式用
#       曼哈顿距离，与四连通代价一致，保证最优性。
# 本模块同时支持单层与"电梯 transit"目标（多楼层流程复用同一寻路核心）。
# =============================================================================

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

def inflate_obstacles(
    data,
    width,
    height,
    radius_cells,
    obstacle_threshold=50,
    inflated_value=100,
):
    """Expand occupied cells by a circular safety radius."""
    if width <= 0 or height <= 0:
        raise ValueError("width and height must be positive")
    if len(data) != width * height:
        raise ValueError("grid data length does not match width * height")
    if radius_cells < 0:
        raise ValueError("radius_cells must not be negative")
    if obstacle_threshold <= 0:
        raise ValueError("obstacle_threshold must be positive")

    inflated = list(data)

    obstacle_cells = []
    for y in range(height):
        for x in range(width):
            value = data[y * width + x]
            if value >= obstacle_threshold:
                obstacle_cells.append((x, y))

    radius_squared = radius_cells * radius_cells

    for obstacle_x, obstacle_y in obstacle_cells:
        for offset_y in range(-radius_cells, radius_cells + 1):
            for offset_x in range(-radius_cells, radius_cells + 1):
                if (
                    offset_x * offset_x + offset_y * offset_y
                    > radius_squared
                ):
                    continue

                x = obstacle_x + offset_x
                y = obstacle_y + offset_y

                if not (0 <= x < width and 0 <= y < height):
                    continue

                index = y * width + x

                # Unknown cells remain unknown; they are already non-traversable.
                if inflated[index] >= 0:
                    inflated[index] = inflated_value

    return inflated