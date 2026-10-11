#!/usr/bin/env python3
"""query_map.py — 诊断占用栅格地图中关键点位的可通行性（本地调试用，非比赛算法）。

用法:
    python3 query_map.py          # 默认采样入口/大厅/走廊关键点
    python3 query_map.py 0 0.5    # 自定义点(世界系 x y)

打印占用值: -1=未知 0=空闲 100=占据; 以及距离最近占用/未知
"""
import sys

import rospy
from nav_msgs.msg import OccupancyGrid

POINTS = [
    ("门外(-0.3)", 0.0, -0.3),
    ("门口(0.0)", 0.0, 0.0),
    ("门内(+0.3)", 0.0, 0.3),
    ("大厅y1", 0.0, 1.0),
    ("大厅y3", 0.0, 3.0),
    ("大厅左(-5,3)", -5.0, 3.0),
    ("走廊起点(0,8)", 0.0, 8.0),
    ("走廊(0,12)", 0.0, 12.0),
    ("左房间口(-2,9)", -2.0, 9.0),
    ("右房间口(2,9)", 2.0, 9.0),
]

_map = {}


def on_map(msg):
    global _map
    _map = {
        "data": msg.data,
        "width": msg.info.width,
        "height": msg.info.height,
        "res": msg.info.resolution,
        "ox": msg.info.origin.position.x,
        "oy": msg.info.origin.position.y,
    }


def val_at(x, y):
    gx = int((x - _map["ox"]) / _map["res"])
    gy = int((y - _map["oy"]) / _map["res"])
    if not (0 <= gx < _map["width"] and 0 <= gy < _map["height"]):
        return None
    return _map["data"][gy * _map["width"] + gx]


def main():
    rospy.init_node("query_map", anonymous=True)
    rospy.Subscriber("/competition/map", OccupancyGrid, on_map, queue_size=1)
    pts = POINTS
    if len(sys.argv) >= 3:
        try:
            pts = [(f"({sys.argv[1]},{sys.argv[2]})", float(sys.argv[1]), float(sys.argv[2]))]
        except ValueError:
            pass
    t0 = rospy.Time.now().to_sec()
    while not _map and rospy.Time.now().to_sec() - t0 < 15:
        rospy.sleep(0.2)
    if not _map:
        print("NO MAP within 15s")
        return
    print(f"map {_map['width']}x{_map['height']} res={_map['res']:.3f} "
          f"origin=({_map['ox']:.2f},{_map['oy']:.2f})")
    for name, x, y in pts:
        v = val_at(x, y)
        tag = {-1: "未知?", 0: "空闲", 100: "占据!"}.get(v, f"{v}")
        print(f"{name:12s} world=({x:6.2f},{y:6.2f}) -> {'(越界)' if v is None else tag}")


if __name__ == "__main__":
    main()
