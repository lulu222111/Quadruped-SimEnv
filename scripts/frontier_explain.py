#!/usr/bin/env python3
"""frontier_explain.py — 复现 planning 的 frontier 选择并解释每一候选簇的打分。

用途(本地诊断, 非比赛算法): 回答"机器人在这个位置为什么选中这个 frontier 簇、
其它簇输在哪、伪空间/房间簇各自到墙距离(dist)是多少"。通过订阅 /competition/map
与 /competition/localization/odom, 实时(或对齐实跑)用与 planning_core 相同的
公式对每个簇计算 score 分解, 并按簇代表点世界坐标标注所处区域
(lobby/corridor/elevator/stair/room_*), 便于核对 8 向 room_bias 等启发式是否合理。

区域标注使用 generated_building/layout_metadata.json 的公开几何(与 robot_start
同世界系), 仅用于人眼核对, 不走入规划决策。

用法(容器内, ROS_MASTER_URI 已指向 sim):
    python3 /workspace/SimEnv/scripts/frontier_explain.py [--top 8] [--interval 3.0]
        [--room-bias-weight 5.0] [--heading-weight 300.0] [--size-weight 0.1]
        [--distance-weight 1.0] [--heading-deg DEG]   # 缺省用 robot 当前 yaw
        [--out /tmp/scores.jsonl] [--duration 120.0]  # 采样一段时间后退出
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry

SCRIPTS_DIR = "/workspace/SimEnv/src/competition_planning/scripts"
LAYOUT_PATH = "/workspace/SimEnv/generated_building/layout_metadata.json"

_map = {}
_odom = None
_map_generation = -1


def on_map(msg):
    global _map, _map_generation
    gen = msg.info.map_load_time.to_sec() if hasattr(msg.info, "map_load_time") else 0.0
    _map = {
        "data": list(msg.data),
        "w": msg.info.width,
        "h": msg.info.height,
        "res": msg.info.resolution,
        "ox": msg.info.origin.position.x,
        "oy": msg.info.origin.position.y,
        "oyaw": quaternion_yaw(msg.info.origin.orientation),
    }
    _map_generation += 1


def on_odom(msg):
    global _odom
    _odom = msg


def quaternion_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def cell_to_world(cell, m):
    cx, cy = cell
    lx = (cx + 0.5) * m["res"]
    ly = (cy + 0.5) * m["res"]
    c, s = math.cos(m["oyaw"]), math.sin(m["oyaw"])
    return (m["ox"] + c * lx - s * ly, m["oy"] + s * lx + c * ly)


def world_to_cell(wx, wy, m):
    dx = wx - m["ox"]
    dy = wy - m["oy"]
    c, s = math.cos(m["oyaw"]), math.sin(m["oyaw"])
    lx = c * dx + s * dy
    ly = -s * dx + c * dy
    return (int(math.floor(lx / m["res"])), int(math.floor(ly / m["res"])))


def load_regions():
    """Return list of (label, x0,x1,y0,y1) from layout_metadata.json."""
    if not os.path.exists(LAYOUT_PATH):
        return []
    with open(LAYOUT_PATH) as fh:
        layout = json.load(fh)
    floor = layout["floors"][0]
    regions = []
    for name, bounds in (
        ("lobby", floor.get("lobby_bounds")),
        ("corridor", floor.get("corridor_bounds")),
        ("elevator_shaft", floor.get("elevator_bounds")),
        ("stairwell", floor.get("stair_bounds")),
    ):
        if bounds:
            regions.append((
                name,
                bounds["x_min"], bounds["x_max"],
                bounds["y_min"], bounds["y_max"],
            ))
    for room in floor.get("rooms", []):
        b = room["bounds"]
        regions.append((
            room["id"],
            b["x_min"], b["x_max"], b["y_min"], b["y_max"],
        ))
    return regions


def classify(wx, wy, regions):
    for label, x0, x1, y0, y1 in regions:
        if x0 <= wx <= x1 and y0 <= wy <= y1:
            return label
    return "outdoor/other"


def explain_frame(m, regions, args, dist_field):
    from frontier import (
        cluster_frontier_cells,
        extract_frontier_cells,
        select_frontier_representative,
    )

    data = m["data"]
    w, h = m["w"], m["h"]
    robot = _odom.pose.pose.position
    robot_yaw = quaternion_yaw(_odom.pose.pose.orientation)
    robot_cell = world_to_cell(robot.x, robot.y, m)
    heading_yaw = (
        math.radians(args.heading_deg)
        if args.heading_deg is not None
        else robot_yaw - m["oyaw"]
    )

    frontier_cells = extract_frontier_cells(data, w, h)
    clusters = cluster_frontier_cells(frontier_cells, min_size=5)
    # cluster_room_dist exactly like planning_core (rep dist keyed by cluster[0])
    cluster_room_dist = {}
    for cl in clusters:
        if not cl:
            continue
        rep = select_frontier_representative(cl)
        cluster_room_dist[cl[0]] = float(dist_field[rep[1], rep[0]])

    rows = []
    for cl in clusters:
        if not cl:
            continue
        rep = select_frontier_representative(cl)
        rx, ry = rep
        wx, wy = cell_to_world(rep, m)
        d_robot = math.hypot(rx - robot_cell[0], ry - robot_cell[1])
        ha = 0.0
        if d_robot > 0.0:
            ha = (
                math.cos(heading_yaw) * (rx - robot_cell[0])
                + math.sin(heading_yaw) * (ry - robot_cell[1])
            ) / d_robot
        room_dist = cluster_room_dist.get(cl[0], 0.0)
        room_bonus = args.room_bias_weight * room_dist
        score = (
            args.size_weight * len(cl)
            - args.distance_weight * d_robot
            + args.heading_weight * ha
            + room_bonus
        )
        rows.append({
            "size": len(cl),
            "rep_world": [round(wx, 2), round(wy, 2)],
            "region": classify(wx, wy, regions),
            "dist_wall_cells": round(room_dist, 1),
            "dist_robot_cells": round(d_robot, 1),
            "heading_align": round(ha, 3),
            "room_bonus": round(room_bonus, 1),
            "score": round(score, 1),
            "first_cell": list(cl[0]),
        })

    rows.sort(key=lambda r: -r["score"])
    return robot, robot_yaw, heading_yaw, rows, frontier_cells


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--top", type=int, default=10)
    parser.add_argument("--interval", type=float, default=3.0)
    parser.add_argument("--duration", type=float, default=0.0)
    parser.add_argument("--room-bias-weight", type=float, default=5.0)
    parser.add_argument("--size-weight", type=float, default=0.1)
    parser.add_argument("--distance-weight", type=float, default=1.0)
    parser.add_argument("--heading-weight", type=float, default=300.0)
    parser.add_argument("--heading-deg", type=float, default=None)
    parser.add_argument("--out", default="")
    parser.add_argument("--min-size", type=int, default=5)
    args = parser.parse_args()

    sys.path.insert(0, SCRIPTS_DIR)
    from room_access import distance_to_wall

    regions = load_regions()
    rospy.init_node("frontier_explain", anonymous=True)
    rospy.Subscriber("/competition/map", OccupancyGrid, on_map, queue_size=1)
    rospy.Subscriber("/competition/localization/odom", Odometry, on_odom, queue_size=1)

    out_fh = open(args.out, "a") if args.out else None
    t0 = time.time()
    seen_gen = -1
    while not rospy.is_shutdown():
        if _map and _odom and _map_generation != seen_gen:
            seen_gen = _map_generation
            data = _map["data"]
            dist_field = distance_to_wall(data, _map["w"], _map["h"], occ_thresh=50)
            robot, yaw, h_yaw, rows, fcells = explain_frame(_map, regions, args, dist_field)
            lines = [
                f"--- gen {_map_generation} robot=({robot.x:.2f},{robot.y:.2f}) "
                f"yaw={math.degrees(yaw):.0f} heading_ref={math.degrees(h_yaw):.0f} "
                f"frontier_cells={len(fcells)} ---"
            ]
            for rank, r in enumerate(rows[: args.top], 1):
                lines.append(
                    f"  #{rank:<2} {r['region']:<14} size={r['size']:<4} "
                    f"rep=({r['rep_world'][0]:>6.2f},{r['rep_world'][1]:>6.2f}) "
                    f"dist_wall={r['dist_wall_cells']:>5.1f} "
                    f"dist_robot={r['dist_robot_cells']:>5.1f} "
                    f"head={r['heading_align']:>+6.3f} roomB={r['room_bonus']:>7.1f} "
                    f"score={r['score']:>8.1f}"
                )
            block = "\n".join(lines)
            print(block, flush=True)
            if out_fh:
                payload = {
                    "gen": _map_generation,
                    "robot": [round(robot.x, 2), round(robot.y, 2)],
                    "yaw_deg": round(math.degrees(yaw), 1),
                    "heading_ref_deg": round(math.degrees(h_yaw), 1),
                    "frontier_cells": len(fcells),
                    "top": rows[: args.top],
                }
                out_fh.write(json.dumps(payload) + "\n")
                out_fh.flush()
        if args.duration > 0 and time.time() - t0 > args.duration:
            break
        rospy.sleep(args.interval)

    if out_fh:
        out_fh.close()


if __name__ == "__main__":
    main()
