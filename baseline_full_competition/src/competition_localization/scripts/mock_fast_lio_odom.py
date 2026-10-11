#!/usr/bin/env python3
# =============================================================================
# 中文说明（模块级）：mock_fast_lio_odom.py —— 定位链路的离线/接口自检桩
#
# 用途：在没有真实 FAST-LIO 输入时，以 10 Hz 发布"静态但时间戳持续前进"的
#       FAST-LIO 风格 /Odometry，用于校验下游定位适配器/建图/规划的消息契约
#       与新鲜度判定（`rostopic pub -r` 会复用消息导致时间戳不变，无法触发判定）。
# 注意：验证辅助节点，不属于正式比赛链路，正式启动不包含它。
# =============================================================================
"""mock_fast_lio_odom — publish a static FAST-LIO-style /Odometry at 10 Hz.

Unlike `rostopic pub -r 10` (which reuses a single message, so every stamp is
identical and tf2 emits TF_REPEATED_DATA warnings), this publisher gives each
message a FRESH rospy.Time.now() stamp, producing realistic distinct
timestamps for the localization adapter.

Used by scripts/verify_localization.sh as the mock FAST-LIO input.
"""

from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry


def main() -> None:
    rospy.init_node("mock_fast_lio_odom", anonymous=True)
    pub = rospy.Publisher("/Odometry", Odometry, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "camera_init"
        msg.child_frame_id = "body"
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        pub.publish(msg)
        rate.sleep()


if __name__ == "__main__":
    main()
