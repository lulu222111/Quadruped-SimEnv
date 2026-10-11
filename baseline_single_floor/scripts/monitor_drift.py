#!/usr/bin/env python3
"""monitor_drift — 实时对比 FAST-LIO 定位 (/competition/localization/odom)
与 Gazebo 真值 (/ground_truth/base_w)，量化机器人的坐标漂移。

用法（容器内）:
    source /opt/ros/noetic/setup.bash
    source /workspace/SimEnv/devel/setup.bash
    export ROS_MASTER_URI=http://localhost:11311
    python3 /workspace/SimEnv/scripts/monitor_drift.py [--interval 2.0]

输出列:
    t(s)  : 相对启动的仿真时间
    flio  : FAST-LIO world->base 位置 (x,y) 与 yaw
    truth : Gazebo 真值位置 (x,y) 与 yaw
    err_xy: 当前误差范数 ||truth - flio||_xy
    drift : 相对首帧基线后的误差增量（真正随时间的漂移）
    mode  : robot 是否在运动 (truth 速度 > 0.02 m/s)
"""

import math
import sys
import threading
import time

import rospy
from nav_msgs.msg import Odometry

INTERVAL = float(sys.argv[sys.argv.index("--interval") + 1]) if "--interval" in sys.argv else 2.0

# 缓存
_lock = threading.Lock()
truth = None       # (t, x, y, z, yaw, vx)
flio = None        # (t, x, y, z, yaw)
baseline = None    # 首帧 (truth - flio) 常量偏移
t_ref = [None]     # 首个真值时间戳，用于相对时间显示


def yaw_from_quat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def on_truth(msg):
    global truth
    with _lock:
        vx = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        truth = (msg.header.stamp.to_sec(), msg.pose.pose.position.x,
                 msg.pose.pose.position.y, msg.pose.pose.position.z,
                 yaw_from_quat(msg.pose.pose.orientation), vx)


def on_flio(msg):
    global flio
    with _lock:
        flio = (msg.header.stamp.to_sec(), msg.pose.pose.position.x,
                msg.pose.pose.position.y, msg.pose.pose.position.z,
                yaw_from_quat(msg.pose.pose.orientation))


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rospy.init_node("monitor_drift", anonymous=True)
    rospy.Subscriber("/competition/localization/odom", Odometry, on_flio)
    rospy.Subscriber("/ground_truth/base_w", Odometry, on_truth)
    t0 = time.time()

    print(f"{'t(s)':>7} | {'flio x,y':>16} | {'truth x,y':>16} | "
          f"{'err_xy(m)':>9} | {'drift(m)':>8} | {'dYaw(°)':>7} | mode")
    print("-" * 92)

    last = [None]
    while not rospy.is_shutdown():
        time.sleep(INTERVAL)
        with _lock:
            t = truth
            f = flio
        if t is None or f is None:
            print(f"{time.time()-t0:7.1f} | 等待数据 truth={t is not None} flio={f is not None}")
            continue

        # 对齐：用最近的真值（仿真中两者频率都很高，直接取当前缓存即可）
        err_xy = math.hypot(t[1] - f[1], t[2] - f[2])
        d_yaw = math.degrees(wrap_pi(t[4] - f[4]))
        if t_ref[0] is None:
            t_ref[0] = t[0]
        rel_t = t[0] - t_ref[0]

        global baseline
        if baseline is None:
            baseline = (t[1] - f[1], t[2] - f[2], t[4] - f[4])
            print(f"{rel_t:7.1f} | 基线锁定 (truth-flio): dx={baseline[0]:+.3f} dy={baseline[1]:+.3f} "
                  f"dyaw={math.degrees(baseline[2]):+.2f}°  开始监测漂移...")
            continue

        drift_xy = math.hypot((t[1] - f[1]) - baseline[0], (t[2] - f[2]) - baseline[1])
        drift_yaw = math.degrees(wrap_pi((t[4] - f[4]) - baseline[2]))

        mode = "运动" if t[5] > 0.02 else "静止"
        print(f"{rel_t:7.1f} | ({f[1]:6.3f},{f[2]:6.3f})   | ({t[1]:6.3f},{t[2]:6.3f})   | "
              f"{err_xy:9.3f} | {drift_xy:8.3f} | {drift_yaw:7.2f} | {mode}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
