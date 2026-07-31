#!/usr/bin/env python3
"""Pure-Python safety and ownership tests for path_follower_node."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


class _MockDuration:
    def __init__(self, seconds):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds


class _MockTimeInstance:
    def __init__(self, seconds=0.0):
        self._seconds = float(seconds)

    def __sub__(self, other):
        return _MockDuration(self._seconds - other._seconds)


class _MockTime:
    _now = 0.0

    @classmethod
    def now(cls):
        return _MockTimeInstance(cls._now)

    @classmethod
    def set(cls, seconds):
        cls._now = float(seconds)


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _MockRospy:
    Time = _MockTime
    Duration = _MockDuration
    INFO = 20

    def __init__(self):
        self.publishers = []
        self.subscribers = []

    def init_node(self, *_args, **_kwargs):
        pass

    def get_param(self, _name, default):
        return default

    def Publisher(self, topic, *_args, **_kwargs):
        publisher = _Publisher(topic)
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic, *_args, **_kwargs):
        self.subscribers.append(topic)
        return object()

    def loginfo(self, *_args, **_kwargs):
        pass

    def loginfo_throttle(self, *_args, **_kwargs):
        pass


ROSPY = _MockRospy()
sys.modules["rospy"] = ROSPY


class _Vector3:
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


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Header:
    def __init__(self, frame_id=""):
        self.frame_id = frame_id
        self.stamp = _MockTime.now()


class _Pose:
    def __init__(self, x=0.0, y=0.0):
        self.position = _Vector3(x, y)
        self.orientation = _Quaternion()


class _PoseStamped:
    def __init__(self, frame_id="", x=0.0, y=0.0):
        self.header = _Header(frame_id)
        self.pose = _Pose(x, y)


class _Path:
    def __init__(self, frame_id="", poses=None):
        self.header = _Header(frame_id)
        self.poses = list(poses or [])


class _Odometry:
    def __init__(self, frame_id="", x=0.0, y=0.0):
        self.header = _Header(frame_id)
        self.pose = type("PoseWithCovariance", (), {})()
        self.pose.pose = _Pose(x, y)


class _GeometryMsgsMsg:
    Twist = _Twist


class _GeometryMsgs:
    msg = _GeometryMsgsMsg()


class _NavMsgsMsg:
    Path = _Path
    Odometry = _Odometry


class _NavMsgs:
    msg = _NavMsgsMsg()


sys.modules["geometry_msgs"] = _GeometryMsgs()
sys.modules["geometry_msgs.msg"] = _GeometryMsgsMsg()
sys.modules["nav_msgs"] = _NavMsgs()
sys.modules["nav_msgs.msg"] = _NavMsgsMsg()


class _ModuleStatus:
    UNKNOWN = 0
    STARTING = 1
    READY = 2
    RUNNING = 3
    SUCCEEDED = 4
    BLOCKED = 5
    TIMEOUT = 6
    FAILED = 7

    def __init__(self):
        self.header = _Header()
        self.state = self.UNKNOWN
        self.valid = False
        self.confidence = 0.0
        self.detail = ""


sys.modules["competition_interfaces"] = type("CompetitionInterfaces", (), {})()
sys.modules["competition_interfaces.msg"] = type(
    "CompetitionInterfacesMsg", (), {"ModuleStatus": _ModuleStatus}
)()


def _load_node():
    path = (
        Path(__file__).resolve().parents[1]
        / "scripts"
        / "path_follower_node"
    )
    namespace = {"__name__": "test_module"}
    with path.open(encoding="utf-8") as source_file:
        exec(compile(source_file.read(), str(path), "exec"), namespace)
    return namespace


NODE = _load_node()
PathFollower = NODE["PathFollower"]


class TestPathFollower(unittest.TestCase):
    def setUp(self):
        _MockTime.set(10.0)
        self.follower = PathFollower.__new__(PathFollower)
        self.follower.path = []
        self.follower.path_frame = ""
        self.follower.path_time = None
        self.follower.path_error = None
        self.follower.odom = None
        self.follower.odom_time = None
        self.follower.odom_error = None
        self.follower.target_index = 0
        self.follower.max_linear = 0.25
        self.follower.max_angular = 0.8
        self.follower.goal_tolerance = 0.2
        self.follower.path_timeout = _MockDuration(5.0)
        self.follower.odom_timeout = _MockDuration(1.0)
        self.follower.raw_vel_pub = _Publisher(
            "/competition/raw_cmd_vel"
        )
        self.follower.status_pub = _Publisher(
            "/competition/control/path_follower_status"
        )

    def _set_valid_odom(self, frame="map", x=0.0, y=0.0):
        self.follower.on_odom(_Odometry(frame, x, y))

    def _set_valid_path(self, frame="map", points=None):
        points = points or [(1.0, 0.0)]
        poses = [
            _PoseStamped(frame, x, y)
            for x, y in points
        ]
        self.follower.on_path(_Path(frame, poses))

    def _last_status(self):
        return self.follower.status_pub.messages[-1]

    def _last_cmd(self):
        return self.follower.raw_vel_pub.messages[-1]

    def test_constructor_owns_only_internal_status_and_raw_velocity(self):
        ROSPY.publishers.clear()
        ROSPY.subscribers.clear()
        PathFollower()
        publisher_topics = [publisher.topic for publisher in ROSPY.publishers]
        self.assertEqual(
            publisher_topics,
            [
                "/competition/raw_cmd_vel",
                "/competition/control/path_follower_status",
            ],
        )
        self.assertNotIn("/cmd_vel", publisher_topics)
        self.assertNotIn("/competition/control/status", publisher_topics)

    def test_normal_tracking_reports_running(self):
        self._set_valid_odom()
        self._set_valid_path()
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.RUNNING)
        self.assertTrue(self._last_status().valid)

    def test_finished_path_reports_succeeded_and_zero(self):
        self._set_valid_odom()
        self._set_valid_path(points=[(0.05, 0.0)])
        self.follower.step()
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.SUCCEEDED)
        self.assertTrue(self._last_status().valid)

    def test_empty_path_cancels_old_path_immediately(self):
        self._set_valid_odom()
        self._set_valid_path()
        self.follower.on_path(_Path("map", []))
        self.assertEqual(self.follower.path, [])
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.follower.step()
        self.assertEqual(self._last_status().state, _ModuleStatus.STARTING)
        self.assertFalse(self._last_status().valid)

    def test_frame_mismatch_reports_failed_and_zero(self):
        self._set_valid_odom(frame="odom")
        self._set_valid_path(frame="map")
        self.follower.step()
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.FAILED)
        self.assertIn("frame mismatch", self._last_status().detail)

    def test_non_finite_path_reports_failed(self):
        self._set_valid_odom()
        self._set_valid_path(points=[(math.nan, 0.0)])
        self.follower.step()
        self.assertEqual(self._last_status().state, _ModuleStatus.FAILED)
        self.assertFalse(self._last_status().valid)

    def test_invalid_quaternion_reports_failed(self):
        odom = _Odometry("map")
        odom.pose.pose.orientation = _Quaternion(0.0, 0.0, 0.0, 0.0)
        self.follower.on_odom(odom)
        self._set_valid_path()
        self.follower.step()
        self.assertEqual(self._last_status().state, _ModuleStatus.FAILED)
        self.assertIn("zero norm", self._last_status().detail)

    def test_path_time_rollback_reports_timeout(self):
        self._set_valid_odom()
        self._set_valid_path()
        self.follower.path_time = _MockTimeInstance(11.0)
        self.follower.step()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertIn("backwards", self._last_status().detail)

    def test_odom_timeout_reports_timeout(self):
        self._set_valid_odom()
        self._set_valid_path()
        self.follower.odom_time = _MockTimeInstance(8.0)
        self.follower.step()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertEqual(self._last_cmd().linear.x, 0.0)


if __name__ == "__main__":
    unittest.main()
