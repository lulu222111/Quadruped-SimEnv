#!/usr/bin/env python3
"""eval_localization — 本地调试评估定位精度（非比赛算法）。
对比 /competition/localization/odom 与 Gazebo 真值 (/gazebo/model_states 的 a1_gazebo)。

用法（容器内）:
    python3 /workspace/SimEnv/scripts/eval_localization.py [--interval 2.0] [--dur 40]

输出:
    err_xy : 当前真值与定位的平面距离差（含初始坐标偏移）
    drift  : 相对首帧基线后的误差增量 = 真正的随时间漂移
    dYaw   : 偏航差
    mode   : 运动/静止（按真值速度）
"""

import math
import sys
import time

import rospy
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

INTERVAL = float(sys.argv[sys.argv.index("--interval") + 1]) if "--interval" in sys.argv else 2.0
DUR = float(sys.argv[sys.argv.index("--dur") + 1]) if "--dur" in sys.argv else 45.0

truth = None   # (t, x, y, z, yaw, vx, vy)
odom = None    # (t, x, y, z, yaw)
baseline = None
t0 = time.time()


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def on_models(msg):
    global truth
    try:
        i = msg.name.index("a1_gazebo")
    except ValueError:
        return
    p = msg.pose[i]
    v = msg.twist[i]
    truth = (time.time(), p.position.x, p.position.y, p.position.z,
             yaw_from_quat(p.orientation), v.linear.x, v.linear.y)


def on_odom(msg):
    global odom
    odom = (time.time(), msg.pose.pose.position.x, msg.pose.pose.position.y,
            msg.pose.pose.position.z, yaw_from_quat(msg.pose.pose.orientation))


def wrap_pi(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def main():
    rospy.init_node("eval_localization", anonymous=True)
    rospy.Subscriber("/competition/localization/odom", Odometry, on_odom)
    rospy.Subscriber("/gazebo/model_states", ModelStates, on_models)
    print(f"{'t(s)':>6} | {'odom x,y':>15} | {'truth x,y':>15} | {'err_xy(m)':>9} | "
          f"{'drift(m)':>8} | {'dYaw(°)':>7} | {'err_z':>7} | mode")
    print("-" * 95)

    while not rospy.is_shutdown() and time.time() - t0 < DUR:
        time.sleep(INTERVAL)
        tr, od = truth, odom
        if tr is None or od is None:
            print(f"{time.time()-t0:6.1f} | 等待数据 truth={tr is not None} odom={od is not None}")
            continue
        err_xy = math.hypot(tr[1] - od[1], tr[2] - od[2])
        err_z = tr[3] - od[3]
        d_yaw = math.degrees(wrap_pi(tr[4] - od[4]))
        global baseline
        if baseline is None:
            baseline = (tr[1] - od[1], tr[2] - od[2], tr[4] - od[4])
            print(f"{time.time()-t0:6.1f} | 基线 (truth-odom): dx={baseline[0]:+.3f} "
                  f"dy={baseline[1]:+.3f} dyaw={math.degrees(baseline[2]):+.2f}° 开始监测...")
            continue
        drift = math.hypot((tr[1] - od[1]) - baseline[0], (tr[2] - od[2]) - baseline[1])
        drift_yaw = math.degrees(wrap_pi((tr[4] - od[4]) - baseline[2]))
        speed = math.hypot(tr[5], tr[6])
        mode = "运动" if speed > 0.02 else "静止"
        print(f"{time.time()-t0:6.1f} | ({od[1]:6.3f},{od[2]:6.3f}) | ({tr[1]:6.3f},{tr[2]:6.3f}) | "
              f"{err_xy:9.3f} | {drift:8.3f} | {drift_yaw:7.2f} | {err_z:+6.3f} | {mode}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
