#!/usr/bin/env python3
"""Pure-Python safety tests for experimental leg_odometry."""

import math
import sys
import unittest
from pathlib import Path


class _Duration:
    def __init__(self, seconds):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds


class _Stamp:
    def __init__(self, seconds=0.0):
        self.seconds = seconds

    def to_sec(self):
        return self.seconds

    def __sub__(self, other):
        return _Duration(self.seconds - other.seconds)


class _Time:
    now_seconds = 0.0

    @classmethod
    def now(cls):
        return _Stamp(cls.now_seconds)


class _Publisher:
    def __init__(self):
        self.published = []

    def publish(self, message):
        self.published.append(message)


class _Rospy:
    Time = _Time

    def get_param(self, name, default=None):
        return default

    def Subscriber(self, *args, **kwargs):
        return None

    def Publisher(self, *args, **kwargs):
        return _Publisher()

    def __getattr__(self, name):
        return lambda *args, **kwargs: None


sys.modules["rospy"] = _Rospy()


class _Vector:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Header:
    def __init__(self, stamp=None):
        self.stamp = stamp if stamp is not None else _Stamp()
        self.frame_id = ""


class _JointState:
    def __init__(self):
        self.header = _Header()
        self.name = []
        self.position = []
        self.velocity = []


class _Imu:
    def __init__(self):
        self.header = _Header()
        self.orientation = _Quaternion()
        self.angular_velocity = _Vector()


class _Odometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.pose = type("PoseWCov", (), {
            "pose": type("Pose", (), {
                "position": _Vector(), "orientation": _Quaternion()
            })()
        })()
        self.twist = type("TwistWCov", (), {
            "twist": type("Twist", (), {
                "linear": _Vector(), "angular": _Vector()
            })()
        })()


class _SensorMsgs:
    Imu = _Imu
    JointState = _JointState


class _NavMsgs:
    Odometry = _Odometry


sys.modules["sensor_msgs"] = type("sensor_msgs", (), {"msg": _SensorMsgs})()
sys.modules["sensor_msgs.msg"] = _SensorMsgs
sys.modules["nav_msgs"] = type("nav_msgs", (), {"msg": _NavMsgs})()
sys.modules["nav_msgs.msg"] = _NavMsgs


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "leg_odometry.py"
mod = type("mod", (), {})()
with open(SCRIPT, "r", encoding="utf-8") as stream:
    source = stream.read()
exec(compile(source.split("if __name__")[0], str(SCRIPT), "exec"), mod.__dict__)


def _imu(stamp, yaw=0.0):
    message = _Imu()
    message.header.stamp = _Stamp(stamp)
    message.orientation = _Quaternion(
        0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    )
    return message


def _joints(stamp, missing=None):
    message = _JointState()
    message.header.stamp = _Stamp(stamp)
    names = [name for name in mod.JOINT_SPEC if name != missing]
    message.name = names
    message.position = [0.0] * len(names)
    message.velocity = [0.0] * len(names)
    return message


class TestLegOdometry(unittest.TestCase):
    def setUp(self):
        _Time.now_seconds = 10.0
        self.node = mod.LegOdometry()

    def test_joint_state_without_imu_does_not_publish(self):
        self.node._js_cb(_joints(10.0))
        self.assertFalse(self.node.joint_state_ready)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_zero_source_stamps_are_rejected_without_now_fallback(self):
        self.node._imu_cb(_imu(0.0))
        self.node._js_cb(_joints(0.0))
        self.assertFalse(self.node.imu_ready)
        self.assertIsNone(self.node.last_imu_stamp)
        self.assertIsNone(self.node.last_t)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_imu_joint_source_skew_blocks_publish(self):
        self.node._imu_cb(_imu(10.0))
        self.node._js_cb(_joints(10.1))
        self.assertFalse(self.node.joint_state_ready)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_stale_imu_receive_age_blocks_publish(self):
        self.node._imu_cb(_imu(10.0))
        _Time.now_seconds = 11.0
        self.node._js_cb(_joints(10.0))
        self.assertFalse(self.node.joint_state_ready)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_incomplete_required_joint_state_blocks_publish(self):
        self.node._imu_cb(_imu(10.0))
        self.node._js_cb(_joints(10.0, missing="RL_calf_joint"))
        self.assertFalse(self.node.joint_state_ready)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_no_support_leg_does_not_integrate_or_publish(self):
        self.node.last_t = 10.0
        self.node.pos[0] = 3.0
        self.node.vel_world[0] = 2.0
        self.node.contact_vz_thresh = -1.0  # makes every support candidate invalid
        self.node._estimate(_Stamp(10.1))
        self.assertAlmostEqual(self.node.pos[0], 3.0)
        self.assertAlmostEqual(self.node.vel_world[0], 0.0)
        self.assertAlmostEqual(self.node.last_t, 10.1)
        self.assertEqual(len(self.node._pub.published), 0)

    def test_twist_linear_is_expressed_in_child_frame(self):
        self.node.yaw = math.pi / 2.0
        self.node.vel_world[:] = [1.0, 0.0, 0.0]
        self.node._publish(_Stamp(12.0))
        message = self.node._pub.published[-1]
        self.assertAlmostEqual(message.twist.twist.linear.x, 0.0, places=6)
        self.assertAlmostEqual(message.twist.twist.linear.y, -1.0, places=6)
        self.assertEqual(message.child_frame_id, "c2r28_base")
        self.assertEqual(message.header.stamp.to_sec(), 12.0)

    def test_dead_contact_thresh_parameter_is_removed(self):
        self.assertNotIn("contact_thresh", source)
        self.assertNotIn("--contact-thresh", source)


if __name__ == "__main__":
    unittest.main()
