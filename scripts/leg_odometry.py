#!/usr/bin/env python3
"""leg_odometry — 基于腿关节角的 A1 腿式里程计（用于校正 FAST-LIO）。

输入:
  /a1_gazebo/joint_states  (sensor_msgs/JointState)  12 个腿关节角/角速度
  /trunk_imu               (sensor_msgs/Imu)         身体朝向(yaw) + 角速度
输出:
  /leg_odometry            (nav_msgs/Odometry)
    frame: c2r28_fastlio_local -> c2r28_base
    内容: 平面 x,y 位置（腿速度积分）+ yaw（IMU）

原理:
  - 每腿正运动学求足端位置 p_foot（躯干系）+ 雅可比 x 关节速度 = 足端速度。
  - 支撑腿（足端低于阈值 = 触地）的足端在世界系固定:
        v_body_world = -R_bw * (v_foot_body + omega_body x p_foot)
    对支撑腿取平均得到机身平面速度。
  - 用 IMU yaw 把速度转到世界系积分出 x,y；yaw 直接用 IMU。
  - 静止时（各关节几乎不动）速度 ~0，位置冻结 —— 正好压制 FAST-LIO 的静止漂移。

用法（容器内）:
    python3 -u /workspace/SimEnv/scripts/leg_odometry.py

FAST-LIO 校正启动:
    roslaunch competition_localization localization_corrected.launch
"""

import argparse

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState

# ---------------------------------------------------------------------------
# A1 运动学参数（与 unitree_guide/include/common/unitreeLeg.h 一致）
# ---------------------------------------------------------------------------
ABAD = 0.0838   # 肩宽/外展连杆长
HIP = 0.2       # 大腿长
KNEE = 0.2      # 小腿长

# 髋关节在躯干系位置（leg: 0=FR, 1=FL, 2=RR, 3=RL）
HIP2B = [
    np.array([0.1805, -0.047, 0.0]),   # FR
    np.array([0.1805,  0.047, 0.0]),   # FL
    np.array([-0.1805, -0.047, 0.0]),  # RR
    np.array([-0.1805,  0.047, 0.0]),  # RL
]
SIDE = [-1.0, 1.0, -1.0, 1.0]          # FR, FL, RR, RL

# 12 个关节在 /a1_gazebo/joint_states 中的名字 -> (leg, joint_idx)
JOINT_SPEC = {
    "FR_hip_joint": (0, 0), "FR_thigh_joint": (0, 1), "FR_calf_joint": (0, 2),
    "FL_hip_joint": (1, 0), "FL_thigh_joint": (1, 1), "FL_calf_joint": (1, 2),
    "RR_hip_joint": (2, 0), "RR_thigh_joint": (2, 1), "RR_calf_joint": (2, 2),
    "RL_hip_joint": (3, 0), "RL_thigh_joint": (3, 1), "RL_calf_joint": (3, 2),
}


def leg_fk(q, leg):
    """足端位置（髋系）正运动学；返回 3 维向量（机身系）。"""
    l1 = SIDE[leg] * ABAD
    l2 = -HIP
    l3 = -KNEE
    s1, s2, s3 = np.sin(q)
    c1, c2, c3 = np.cos(q)
    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3
    p = np.array([
        l3 * s23 + l2 * s2,
        -l3 * s1 * c23 + l1 * c1 - l2 * c2 * s1,
        l3 * c1 * c23 + l1 * s1 + l2 * c1 * c2,
    ])
    return HIP2B[leg] + p  # 机身系足端


def leg_jacobian(q, leg):
    """足端速度雅可比 (3x3): v_foot = J * qd（机身系）。"""
    l1 = SIDE[leg] * ABAD
    l2 = -HIP
    l3 = -KNEE
    s1, s2, s3 = np.sin(q)
    c1, c2, c3 = np.cos(q)
    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3
    J = np.zeros((3, 3))
    J[0, 0] = 0.0
    J[1, 0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1
    J[2, 0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1
    J[0, 1] = l3 * c23 + l2 * c2
    J[1, 1] = l3 * s1 * s23 + l2 * s1 * s2
    J[2, 1] = -l3 * c1 * s23 - l2 * c1 * s2
    J[0, 2] = l3 * c23
    J[1, 2] = l3 * s1 * s23
    J[2, 2] = -l3 * c1 * s23
    return J


def quat_yaw(q):
    """从四元数取 yaw（绕世界 z）。"""
    x, y, z, w = q
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rot(q):
    """四元数 -> 3x3 旋转矩阵（body->world）。"""
    x, y, z, w = q
    n = np.hypot(np.hypot(np.hypot(w, x), y), z)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class LegOdometry:
    def __init__(self, contact_thresh=-0.18, lp_alpha=0.8,
                 contact_vz_thresh=0.15, vel_scale=1.2):
        self.contact_thresh = contact_thresh   # 足端机身系 z 低于此值视为触地（辅助）
        self.lp_alpha = lp_alpha               # 速度低通系数
        self.contact_vz_thresh = contact_vz_thresh  # 支撑脚竖直速度阈值
        self.vel_scale = vel_scale             # 速度标定系数（补偿支撑脚打滑）
        self.frame_id = rospy.get_param("~frame_id", "c2r28_fastlio_local")
        self.child_frame_id = rospy.get_param("~child_frame_id", "c2r28_base")
        self.rate_hz = rospy.get_param("~rate_hz", 50.0)

        self.q = {i: np.zeros(3) for i in range(4)}       # 各腿关节角
        self.qd = {i: np.zeros(3) for i in range(4)}      # 各腿关节角速度
        self.pos = np.zeros(3)                            # 世界系位置
        self.yaw = 0.0
        self.vel_world = np.zeros(3)                      # 平滑后的世界系速度
        self.omega_body = np.zeros(3)
        self.R_bw = np.eye(3)                             # body->world 全旋转（IMU）
        self.last_t = None
        self.imu_ok = False

        self._sub_js = rospy.Subscriber("/a1_gazebo/joint_states", JointState,
                                        self._js_cb, queue_size=5, tcp_nodelay=True)
        self._sub_imu = rospy.Subscriber("/trunk_imu", Imu,
                                         self._imu_cb, queue_size=5, tcp_nodelay=True)
        self._pub = rospy.Publisher("/leg_odometry", Odometry, queue_size=10)
        rospy.loginfo(
            "[leg_odometry] 订阅 /a1_gazebo/joint_states + /trunk_imu, "
            "发布 /leg_odometry (%s->%s), 触地阈值 z<%.2f",
            self.frame_id, self.child_frame_id, self.contact_thresh,
        )

    def _js_cb(self, msg):
        names = list(msg.name)
        pos = list(msg.position)
        vel = list(msg.velocity) if len(msg.velocity) == len(names) else None
        stamp = msg.header.stamp
        now = stamp.to_sec() if stamp and stamp.to_sec() > 0 else rospy.Time.now().to_sec()
        for i, name in enumerate(names):
            spec = JOINT_SPEC.get(name)
            if spec is None:
                continue
            leg, jidx = spec
            self.q[leg][jidx] = pos[i]
            if vel is not None:
                self.qd[leg][jidx] = vel[i]
        self._estimate(now)

    def _imu_cb(self, msg):
        o = msg.orientation
        q = [o.x, o.y, o.z, o.w]
        self.yaw = quat_yaw(q)
        self.R_bw = quat_to_rot(q)
        self.imu_ok = True
        g = msg.angular_velocity
        self.omega_body = np.array([g.x, g.y, g.z])

    def _estimate(self, now):
        if self.last_t is None:
            self.last_t = now
            return
        dt = now - self.last_t
        self.last_t = now
        if dt <= 0 or dt > 0.5:
            return

        # 每条腿的足端位置与速度（机身系）+ 世界系足端高度/竖直速度
        foot_world_z = []
        foot_world_vz = []
        feet = []
        for leg in range(4):
            p_foot = leg_fk(self.q[leg], leg)
            feet.append(p_foot)
            foot_world_z.append((self.R_bw @ p_foot)[2])   # 足端相对躯干垂直高度
            v_foot = leg_jacobian(self.q[leg], leg) @ self.qd[leg]
            vf_w = self.R_bw @ (v_foot + np.cross(self.omega_body, p_foot))
            foot_world_vz.append(vf_w[2])                 # 足端竖直速度（世界系）

        # 触地判据（组合）：
        #   - 高度：摆动腿会抬起（辅助）
        #   - 竖直速度：支撑脚竖直速度 ~0，摆动腿明显非零（主判据）。
        #     注意：高速时支撑脚有打滑，不能用世界系三维速度判据（会误杀支撑腿）。
        min_z = min(foot_world_z)
        cand = []          # (最稳系数, 机身速度候选)
        for leg in range(4):
            if foot_world_z[leg] > min_z + 0.04:
                continue  # 摆动腿（抬高了）
            if abs(foot_world_vz[leg]) > self.contact_vz_thresh:
                continue  # 摆动腿（竖直速度大）
            p_foot = feet[leg]
            v_foot = leg_jacobian(self.q[leg], leg) @ self.qd[leg]
            # 足端世界系固定: v_body_world = -R_bw*(v_foot + omega x p_foot)
            wxp = np.cross(self.omega_body, p_foot)
            v_body = -(v_foot + wxp)
            v_fw = self.R_bw @ (v_foot + wxp)
            cand.append((np.linalg.norm(v_fw), self.R_bw @ v_body))

        if cand:
            # 用最"扎实落地"的腿（足端世界系速度最小）的速度候选，
            # 避免平均被混入的摆动腿拉低（低速时摆动腿 vz 也小，防不胜防）。
            cand.sort(key=lambda c: c[0])
            v_raw = cand[0][1]
        else:
            v_raw = self.vel_world  # 无支撑腿时沿用上次速度

        # 低通 + 静止抑制（关节几乎不动 -> 位置冻结）
        if np.all([np.allclose(self.qd[i], 0, atol=1e-3) for i in range(4)]):
            v_raw = np.zeros(3)
        self.vel_world = self.lp_alpha * v_raw + (1 - self.lp_alpha) * self.vel_world

        # 用标定系数补偿支撑脚打滑带来的速度低估
        self.pos[0] += self.vel_scale * self.vel_world[0] * dt
        self.pos[1] += self.vel_scale * self.vel_world[1] * dt
        self.pos[2] = 0.0

        self._publish(now)

    def _publish(self, now):
        odom = Odometry()
        odom.header.stamp = rospy.Time.from_sec(now)
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.pos[0]
        odom.pose.pose.position.y = self.pos[1]
        odom.pose.pose.position.z = 0.0
        yaw = self.yaw if self.imu_ok else 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vel_world[0]
        odom.twist.twist.linear.y = self.vel_world[1]
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = self.omega_body[0]
        odom.twist.twist.angular.y = self.omega_body[1]
        odom.twist.twist.angular.z = self.omega_body[2]
        self._pub.publish(odom)

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--contact-thresh", type=float, default=-0.18,
                    help="足端机身系 z 低于此值判定为触地（辅助判据）")
    ap.add_argument("--contact-vel-thresh", type=float, default=0.15,
                    help="支撑脚竖直速度阈值（主判据，默认 0.15 m/s）")
    ap.add_argument("--vel-scale", type=float, default=1.2,
                    help="速度标定系数（补偿支撑脚打滑，实测最优 1.2）")
    ap.add_argument("--lp-alpha", type=float, default=0.8,
                    help="速度低通系数（实测最优 0.8）")
    args = ap.parse_args()
    rospy.init_node("leg_odometry", anonymous=True)
    node = LegOdometry(contact_thresh=args.contact_thresh,
                       contact_vz_thresh=args.contact_vel_thresh,
                       lp_alpha=args.lp_alpha,
                       vel_scale=args.vel_scale)
    node.spin()
