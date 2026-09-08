#!/usr/bin/env python3
"""zone_monitor.py — 后台盯 robot 所在区段(zone)，进房即落日志(后台监视用)。
用法: python3 -u zone_monitor.py   → 每 8s 打印/记录 truth 位置+zone
"""
import math
import time

import rospy
from gazebo_msgs.msg import ModelStates

CORR = 7.85
HALF = 1.1


def zone(x, y):
    if y < CORR:
        return "lobby"
    if -HALF <= x <= HALF:
        return "corridor"
    return "room"


def main():
    rospy.init_node("zone_monitor", anonymous=True)
    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, lambda m: None)
    last_zone = None
    t0 = time.time()
    while not rospy.is_shutdown():
        try:
            msgs = rospy.wait_for_message("/gazebo/model_states", ModelStates, timeout=5)
        except Exception:
            continue
        try:
            i = msgs.name.index("a1_gazebo")
        except ValueError:
            continue
        p = msgs.pose[i]
        x, y = p.position.x, p.position.y
        z = zone(x, y)
        if z != last_zone:
            print("CHANGE t=%.1f -> %s @(%.2f,%.2f)" % (time.time() - t0, z, x, y), flush=True)
            last_zone = z
        time.sleep(8)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
