#!/usr/bin/env python3
"""Experimental A1 leg odometry used as a required FAST-LIO correction.

Publishing requires a complete JointState sample paired with a fresh,
source-time-compatible IMU sample and at least one reliable support leg.
Invalid observations never create a fresh correction.
"""

import math

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState


ABAD = 0.0838
HIP = 0.2
KNEE = 0.2
HIP2B = [
    np.array([0.1805, -0.047, 0.0]),
    np.array([0.1805, 0.047, 0.0]),
    np.array([-0.1805, -0.047, 0.0]),
    np.array([-0.1805, 0.047, 0.0]),
]
SIDE = [-1.0, 1.0, -1.0, 1.0]
JOINT_SPEC = {
    "FR_hip_joint": (0, 0), "FR_thigh_joint": (0, 1), "FR_calf_joint": (0, 2),
    "FL_hip_joint": (1, 0), "FL_thigh_joint": (1, 1), "FL_calf_joint": (1, 2),
    "RR_hip_joint": (2, 0), "RR_thigh_joint": (2, 1), "RR_calf_joint": (2, 2),
    "RL_hip_joint": (3, 0), "RL_thigh_joint": (3, 1), "RL_calf_joint": (3, 2),
}
REQUIRED_JOINTS = frozenset(JOINT_SPEC)


def leg_fk(q, leg):
    """Return foot position in the body frame."""
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
    return HIP2B[leg] + p


def leg_jacobian(q, leg):
    """Return the 3x3 foot velocity Jacobian in the body frame."""
    l1 = SIDE[leg] * ABAD
    l2 = -HIP
    l3 = -KNEE
    s1, s2, s3 = np.sin(q)
    c1, c2, c3 = np.cos(q)
    c23 = c2 * c3 - s2 * s3
    s23 = s2 * c3 + c2 * s3
    jacobian = np.zeros((3, 3))
    jacobian[1, 0] = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1
    jacobian[2, 0] = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1
    jacobian[0, 1] = l3 * c23 + l2 * c2
    jacobian[1, 1] = l3 * s1 * s23 + l2 * s1 * s2
    jacobian[2, 1] = -l3 * c1 * s23 - l2 * c1 * s2
    jacobian[0, 2] = l3 * c23
    jacobian[1, 2] = l3 * s1 * s23
    jacobian[2, 2] = -l3 * c1 * s23
    return jacobian


def quat_yaw(q):
    x, y, z, w = q
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quat_to_rot(q):
    """Convert x/y/z/w quaternion to a body-to-world rotation matrix."""
    x, y, z, w = q
    norm = np.hypot(np.hypot(np.hypot(w, x), y), z)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def rot_z(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


class LegOdometry:
    """Source-stamped, fail-closed planar leg odometry."""

    def __init__(self, lp_alpha=0.8, contact_vz_thresh=0.15, vel_scale=1.2):
        # Preserve the PR candidate's algorithm parameters in this safety pass.
        self.lp_alpha = float(rospy.get_param("~lp_alpha", lp_alpha))
        self.contact_vz_thresh = float(
            rospy.get_param("~contact_vz_thresh", contact_vz_thresh)
        )
        self.vel_scale = float(rospy.get_param("~vel_scale", vel_scale))
        self.max_imu_age = float(rospy.get_param("~max_imu_age", 0.5))
        self.max_imu_joint_skew = float(
            rospy.get_param("~max_imu_joint_skew", 0.05)
        )
        if self.max_imu_age <= 0.0:
            raise ValueError("~max_imu_age must be > 0")
        if self.max_imu_joint_skew < 0.0:
            raise ValueError("~max_imu_joint_skew must be >= 0")

        self.frame_id = rospy.get_param("~frame_id", "c2r28_fastlio_local")
        self.child_frame_id = rospy.get_param("~child_frame_id", "c2r28_base")
        self.joint_state_topic = rospy.get_param(
            "~joint_state_topic", "/a1_gazebo/joint_states"
        )
        self.imu_topic = rospy.get_param("~imu_topic", "/trunk_imu")
        self.odom_topic = rospy.get_param("~odom_topic", "/leg_odometry")

        self.q = {i: np.zeros(3) for i in range(4)}
        self.qd = {i: np.zeros(3) for i in range(4)}
        self.pos = np.zeros(3)
        self.yaw = 0.0
        self.vel_world = np.zeros(3)
        self.omega_body = np.zeros(3)
        self.R_bw = np.eye(3)
        self.last_t = None

        # Explicit input readiness and source/receive timestamps.
        self.imu_ready = False
        self.joint_state_ready = False
        self.last_imu_stamp = None
        self.last_joint_stamp = None
        self.last_imu_receive_time = None

        self._sub_js = rospy.Subscriber(
            self.joint_state_topic, JointState, self._js_cb,
            queue_size=5, tcp_nodelay=True,
        )
        self._sub_imu = rospy.Subscriber(
            self.imu_topic, Imu, self._imu_cb,
            queue_size=5, tcp_nodelay=True,
        )
        self._pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        rospy.loginfo(
            "leg_odometry: EXPERIMENTAL input=%s + %s output=%s "
            "max_imu_age=%.3f max_skew=%.3f",
            self.joint_state_topic, self.imu_topic, self.odom_topic,
            self.max_imu_age, self.max_imu_joint_skew,
        )

    @staticmethod
    def _valid_stamp(stamp):
        return stamp is not None and stamp.to_sec() > 0.0

    def _skip_joint_sample(self, stamp=None):
        """Break integration continuity across an unavailable observation."""
        self.vel_world = np.zeros(3)
        self.last_t = stamp.to_sec() if self._valid_stamp(stamp) else None

    def _imu_cb(self, msg):
        stamp = msg.header.stamp
        if not self._valid_stamp(stamp):
            self.imu_ready = False
            self.last_imu_stamp = None
            rospy.logwarn_throttle(1.0, "leg_odometry: rejecting zero IMU stamp")
            return
        if self.last_imu_stamp is not None and stamp.to_sec() < self.last_imu_stamp.to_sec():
            self.imu_ready = False
            self.last_imu_stamp = None
            self.last_imu_receive_time = None
            rospy.logwarn("leg_odometry: IMU timestamp rollback; state reset")
            return

        orientation = msg.orientation
        angular = msg.angular_velocity
        quat = np.array([orientation.x, orientation.y, orientation.z, orientation.w], dtype=float)
        omega = np.array([angular.x, angular.y, angular.z], dtype=float)
        norm = float(np.linalg.norm(quat))
        if (not np.all(np.isfinite(quat)) or not np.all(np.isfinite(omega))
                or norm <= 1e-6 or abs(norm - 1.0) >= 1e-3):
            self.imu_ready = False
            rospy.logwarn_throttle(1.0, "leg_odometry: invalid IMU orientation/rate")
            return

        self.yaw = float(quat_yaw(quat))
        self.R_bw = quat_to_rot(quat)
        self.omega_body = omega
        self.last_imu_stamp = stamp
        self.last_imu_receive_time = rospy.Time.now()
        self.imu_ready = True

    def _js_cb(self, msg):
        stamp = msg.header.stamp
        if not self._valid_stamp(stamp):
            self.joint_state_ready = False
            self._skip_joint_sample()
            rospy.logwarn_throttle(1.0, "leg_odometry: rejecting zero JointState stamp")
            return
        if self.last_joint_stamp is not None and stamp.to_sec() < self.last_joint_stamp.to_sec():
            self.joint_state_ready = False
            self.last_joint_stamp = None
            self._skip_joint_sample()
            rospy.logwarn("leg_odometry: JointState timestamp rollback; state reset")
            return

        names = list(msg.name)
        index = {name: i for i, name in enumerate(names)}
        if (not REQUIRED_JOINTS.issubset(index)
                or any(index[name] >= len(msg.position) for name in REQUIRED_JOINTS)
                or any(index[name] >= len(msg.velocity) for name in REQUIRED_JOINTS)):
            self.joint_state_ready = False
            self.last_joint_stamp = stamp
            self._skip_joint_sample(stamp)
            rospy.logwarn_throttle(1.0, "leg_odometry: incomplete required JointState sample")
            return

        if not self.imu_ready or self.last_imu_stamp is None:
            self.joint_state_ready = False
            self.last_joint_stamp = stamp
            self._skip_joint_sample(stamp)
            rospy.logwarn_throttle(1.0, "leg_odometry: waiting for valid IMU")
            return

        now = rospy.Time.now()
        imu_receive_age = (now - self.last_imu_receive_time).to_sec()
        source_skew = abs(stamp.to_sec() - self.last_imu_stamp.to_sec())
        if imu_receive_age < 0.0:
            self.imu_ready = False
            self.last_imu_stamp = None
            self.last_imu_receive_time = None
            self.joint_state_ready = False
            self.last_joint_stamp = stamp
            self._skip_joint_sample(stamp)
            rospy.logwarn("leg_odometry: ROS time rollback; IMU state reset")
            return
        if imu_receive_age > self.max_imu_age or source_skew > self.max_imu_joint_skew:
            self.joint_state_ready = False
            self.last_joint_stamp = stamp
            self._skip_joint_sample(stamp)
            rospy.logwarn_throttle(
                1.0, "leg_odometry: stale/skewed IMU age=%.3f skew=%.3f; sample skipped",
                imu_receive_age, source_skew,
            )
            return

        new_q = {i: np.zeros(3) for i in range(4)}
        new_qd = {i: np.zeros(3) for i in range(4)}
        for name, (leg, joint_index) in JOINT_SPEC.items():
            sample_index = index[name]
            position = float(msg.position[sample_index])
            velocity = float(msg.velocity[sample_index])
            if not math.isfinite(position) or not math.isfinite(velocity):
                self.joint_state_ready = False
                self.last_joint_stamp = stamp
                self._skip_joint_sample(stamp)
                rospy.logwarn_throttle(1.0, "leg_odometry: non-finite required joint sample")
                return
            new_q[leg][joint_index] = position
            new_qd[leg][joint_index] = velocity

        self.q = new_q
        self.qd = new_qd
        self.last_joint_stamp = stamp
        self.joint_state_ready = True
        self._estimate(stamp)

    def _estimate(self, stamp):
        sample_time = stamp.to_sec()
        if self.last_t is None:
            self.last_t = sample_time
            return
        dt = sample_time - self.last_t
        self.last_t = sample_time
        if dt <= 0.0 or dt > 0.5:
            self.vel_world = np.zeros(3)
            return

        foot_world_z = []
        foot_world_vz = []
        feet = []
        for leg in range(4):
            p_foot = leg_fk(self.q[leg], leg)
            feet.append(p_foot)
            foot_world_z.append((self.R_bw @ p_foot)[2])
            v_foot = leg_jacobian(self.q[leg], leg) @ self.qd[leg]
            velocity_world = self.R_bw @ (v_foot + np.cross(self.omega_body, p_foot))
            foot_world_vz.append(velocity_world[2])

        min_z = min(foot_world_z)
        candidates = []
        for leg in range(4):
            # Retain only the PR's actually-used relative-height and vertical-
            # velocity criteria.  The unused absolute-Z parameter is removed.
            if foot_world_z[leg] > min_z + 0.04:
                continue
            if abs(foot_world_vz[leg]) > self.contact_vz_thresh:
                continue
            p_foot = feet[leg]
            v_foot = leg_jacobian(self.q[leg], leg) @ self.qd[leg]
            foot_motion_body = v_foot + np.cross(self.omega_body, p_foot)
            foot_motion_world = self.R_bw @ foot_motion_body
            candidates.append((np.linalg.norm(foot_motion_world), self.R_bw @ (-foot_motion_body)))

        if not candidates:
            # No coasting, position integration, or fake zero-velocity output.
            # last_t already advanced, preventing a recovery-time dt spike.
            self.vel_world = np.zeros(3)
            rospy.logwarn_throttle(1.0, "leg_odometry: no reliable support leg; sample unavailable")
            return

        candidates.sort(key=lambda candidate: candidate[0])
        velocity_raw = candidates[0][1]
        if all(np.allclose(self.qd[leg], 0.0, atol=1e-3) for leg in range(4)):
            velocity_raw = np.zeros(3)
        self.vel_world = self.lp_alpha * velocity_raw + (1.0 - self.lp_alpha) * self.vel_world
        self.pos[0] += self.vel_scale * self.vel_world[0] * dt
        self.pos[1] += self.vel_scale * self.vel_world[1] * dt
        self.pos[2] = 0.0
        self._publish(stamp)

    def _publish(self, stamp):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.pos[0]
        odom.pose.pose.position.y = self.pos[1]
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = np.sin(self.yaw / 2.0)
        odom.pose.pose.orientation.w = np.cos(self.yaw / 2.0)

        # Odometry twist is expressed in child_frame_id.  The published pose
        # is yaw-only, so use the matching world-to-base inverse rotation.
        velocity_base = rot_z(self.yaw).T @ self.vel_world
        odom.twist.twist.linear.x = velocity_base[0]
        odom.twist.twist.linear.y = velocity_base[1]
        odom.twist.twist.linear.z = velocity_base[2]
        odom.twist.twist.angular.x = self.omega_body[0]
        odom.twist.twist.angular.y = self.omega_body[1]
        odom.twist.twist.angular.z = self.omega_body[2]
        self._pub.publish(odom)


def main():
    rospy.init_node("leg_odometry")
    LegOdometry()
    rospy.spin()


if __name__ == "__main__":
    main()
