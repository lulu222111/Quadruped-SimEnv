#!/usr/bin/env python3
"""fastlio_env_test.py — 对照实验 v2：raw FAST-LIO 在不同环境区段的真实漂移。

v2 修正：首帧用完整 2D 刚体变换(旋转+平移)把 FAST-LIO 系对齐到真值系，
消除固定 yaw 差导致的"假漂移"，只保留 FAST-LIO 自身随运动的漂移。

zone: lobby(y<7.85) / corridor(|x|<=1.1,y>=7.85) / room(|x|>1.1,y>=7.85)
每段结算: 段内漂移增量 / 段内真值位移。
"""
import math
import time

import numpy as np
import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

INTERVAL = 4.0
DUR = 1500.0
CORR = 7.85
CORR_HALF = 1.1

truth0 = None  # (x,y,yaw)
fl0 = None
T_align = None  # 3x3 homog: truth系 <- fl系
last_tr = None
last_fl = None
seg = None
t0 = time.time()
log = []


def q2yaw(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def hmat(x, y, th):
    c, s = math.cos(th), math.sin(th)
    return np.array([[c, -s, x], [s, c, y], [0.0, 0.0, 1.0]])


def zone_of(x, y):
    if y < CORR:
        return "lobby"
    if -CORR_HALF <= x <= CORR_HALF:
        return "corridor"
    return "room"


def settle():
    global seg
    if seg is None:
        return
    dx = math.hypot(seg["x2"] - seg["x1"], seg["y2"] - seg["y1"])
    dd = math.hypot(seg["e2"] - seg["e1"])
    rate = (dd / dx * 100.0) if dx > 0.05 else float("nan")
    line = "--- 段结束 [%s] 位移=%5.2fm 漂移增量=%5.3fm 漂移率=%5.1f%% ---" % (seg["zone"], dx, dd, rate)
    print(line)
    log.append(line)
    seg = None


def update_zone(nx, ny, errx, erry):
    global seg
    z = zone_of(nx, ny)
    e = math.hypot(errx, erry)
    if seg is None:
        seg = {"zone": z, "x1": nx, "y1": ny, "x2": nx, "y2": ny, "e1": e, "e2": e}
    elif seg["zone"] != z:
        settle()
        seg = {"zone": z, "x1": nx, "y1": ny, "x2": nx, "y2": ny, "e1": e, "e2": e}
    else:
        seg["x2"], seg["y2"], seg["e2"] = nx, ny, e


def on_models(msg):
    global last_tr
    try:
        i = msg.name.index("a1_gazebo")
    except ValueError:
        return
    p = msg.pose[i]
    last_tr = (p.position.x, p.position.y, q2yaw(p.orientation))


def on_fl(msg):
    global last_fl
    p = msg.pose.pose
    last_fl = (p.position.x, p.position.y, q2yaw(p.orientation))


def main():
    global truth0, fl0, T_align
    rospy.init_node("fastlio_env_test", anonymous=True)
    rospy.Subscriber("/gazebo/model_states", ModelStates, on_models)
    rospy.Subscriber("/Odometry", Odometry, on_fl)
    print("   t(s) |   zone   |   truth x,y    | seg_位移 | seg_漂移 | 累计漂移")
    print("-" * 74)
    while not rospy.is_shutdown() and time.time() - t0 < DUR:
        time.sleep(INTERVAL)
        if last_tr is None or last_fl is None:
            continue
        if truth0 is None:
            truth0 = last_tr
            fl0 = last_fl
            T_align = hmat(*truth0) @ np.linalg.inv(hmat(*fl0))
            print("%6.1f | 首帧刚体对齐 truth0=%s fl0=%s" % (time.time() - t0, truth0, fl0))
            continue
        v = T_align @ np.array([last_fl[0], last_fl[1], 1.0])
        ax, ay = v[0], v[1]
        errx = ax - last_tr[0]
        erry = ay - last_tr[1]
        update_zone(last_tr[0], last_tr[1], errx, erry)
        segdx = math.hypot(seg["x2"] - seg["x1"], seg["y2"] - seg["y1"])
        segdd = math.hypot(seg["e2"] - seg["e1"])
        cum = math.hypot(errx, erry)
        z = zone_of(*last_tr[:2])
        print("%6.1f | %8s | (%6.2f,%6.2f) | %7.2f | %7.3f | %7.3f" % (
            time.time() - t0, z, last_tr[0], last_tr[1], segdx, segdd, cum))
    settle()
    print("=== 汇总(段结算见上方 --- 行) ===")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
