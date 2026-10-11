#!/usr/bin/env python3
"""dump_map_ascii.py — 把 /competition/map 缩略为 ASCII 布局，叠加 robot 与危险源（本地调试用）。

用法（容器内, ROS_MASTER_URI 已指到 sim master）:
    python3 /workspace/SimEnv/scripts/dump_map_ascii.py [x y] [xy_per_char]
字符: '#'=占据 '.'=空闲 ':'=未知(>50) '?'=未知(<50) 'R'=robot 'D'=danger
"""
import sys

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import OccupancyGrid

_map = {}
_robot = None


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


def on_models(msg):
    global _robot
    try:
        i = msg.name.index("a1_gazebo")
        _robot = msg.pose[i]
    except (ValueError, AttributeError):
        pass


def main():
    rospy.init_node("dump_map_ascii", anonymous=True)
    rospy.Subscriber("/competition/map", OccupancyGrid, on_map, queue_size=1)
    rospy.Subscriber("/gazebo/model_states", ModelStates, on_models, queue_size=1)
    t0 = rospy.Time.now().to_sec()
    while (not _map) and rospy.Time.now().to_sec() - t0 < 15:
        rospy.sleep(0.2)
    if not _map:
        print("NO MAP")
        return
    w, h, res = _map["width"], _map["height"], _map["res"]
    ox, oy = _map["ox"], _map["oy"]
    step = float(sys.argv[1]) if len(sys.argv) > 1 else 0.5
    xs = float(sys.argv[2]) if len(sys.argv) > 2 else ox
    ys = float(sys.argv[3]) if len(sys.argv) > 3 else oy
    xe = float(sys.argv[4]) if len(sys.argv) > 4 else ox + w * res
    ye = float(sys.argv[5]) if len(sys.argv) > 5 else oy + h * res
    # downsample: print in world grid of `step` meters
    rows = []
    gy = ys
    while gy < ye:
        line = []
        gx = xs
        while gx < xe:
            ch = "."
            for dy in range(0, 5):
                for dx in range(0, 5):
                    px = gx + dx * step / 5
                    py = gy + dy * step / 5
                    c = int((px - ox) / res)
                    r = int((py - oy) / res)
                    if 0 <= c < w and 0 <= r < h:
                        v = _map["data"][r * w + c]
                        if v > 80:
                            ch = "#"
                            break
                        if v < 0:
                            ch = ":" if ch == "." else ch
                        elif 0 <= v <= 20:
                            ch = "." if ch != "#" else ch
                        elif ch == ".":
                            ch = "?"
                else:
                    continue
                break
            if _robot is not None and abs(gx - _robot.position.x) < step / 2 and abs(gy - _robot.position.y) < step / 2:
                ch = "R"
            line.append(ch)
            gx += step
        rows.append("".join(line))
        gy += step
    print(f"map {w}x{h} res={res:.3f} origin=({ox:.2f},{oy:.2f}) step={step}")
    for row in rows:
        print(row)
    if _robot is not None:
        print(f"robot at ({_robot.position.x:.2f},{_robot.position.y:.2f})")


if __name__ == "__main__":
    main()
