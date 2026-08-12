#!/usr/bin/env python3
"""Pure-Python tests for control status aggregation and velocity gating."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


class _MockDuration:
    def __init__(self, secs):
        self._seconds = float(secs)

    def to_sec(self):
        return self._seconds


class _MockTimeInstance:
    def __init__(self, seconds=0.0):
        self._seconds = float(seconds)

    def to_sec(self):
        return self._seconds

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

    @classmethod
    def advance(cls, seconds):
        cls._now += float(seconds)


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

    def init_node(self, *args, **kwargs):
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

    def Timer(self, *_args, **_kwargs):
        return object()

    def on_shutdown(self, _callback):
        pass

    def sleep(self, _duration):
        pass

    def spin(self):
        pass

    def loginfo(self, *_args, **_kwargs):
        pass


ROSPY = _MockRospy()
sys.modules["rospy"] = ROSPY


class _Vector3:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _Twist:
    def __init__(self):
        self.linear = _Vector3()
        self.angular = _Vector3()


class _Bool:
    def __init__(self, data=False):
        self.data = data


class _GeometryMsgsMsg:
    Twist = _Twist


class _GeometryMsgs:
    msg = _GeometryMsgsMsg()


class _StdMsgsMsg:
    Bool = _Bool


class _StdMsgs:
    msg = _StdMsgsMsg()


sys.modules["geometry_msgs"] = _GeometryMsgs()
sys.modules["geometry_msgs.msg"] = _GeometryMsgsMsg()
sys.modules["std_msgs"] = _StdMsgs()
sys.modules["std_msgs.msg"] = _StdMsgsMsg()


class _Header:
    def __init__(self):
        self.stamp = _MockTime.now()


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


def _load_node(filename):
    path = Path(__file__).resolve().parents[1] / "scripts" / filename
    namespace = {"__name__": "test_module"}
    with path.open(encoding="utf-8") as source_file:
        exec(compile(source_file.read(), str(path), "exec"), namespace)
    return namespace


NODE = _load_node("cmd_vel_guard_node")
CmdVelGuard = NODE["CmdVelGuard"]


class TestCmdVelGuard(unittest.TestCase):
    def setUp(self):
        _MockTime.set(10.0)
        self.guard = CmdVelGuard.__new__(CmdVelGuard)
        self.guard._timeout = 0.5
        self.guard._path_status_timeout = 1.0
        self.guard._max_linear_x = 0.5
        self.guard._max_linear_y = 0.3
        self.guard._max_angular_z = 1.0
        self.guard._last_raw_cmd_time = None
        self.guard._last_path_status_time = None
        self.guard._path_status = None
        self.guard._control_enabled = True
        self.guard._emergency_stop = False
        self.guard._raw_cmd_finite = True
        self.guard._raw_linear_x = 0.0
        self.guard._raw_linear_y = 0.0
        self.guard._raw_angular_z = 0.0
        self.guard._cmd_vel_pub = _Publisher("/cmd_vel")
        self.guard._status_pub = _Publisher("/competition/control/status")

    def _set_fresh_raw(self, x=0.25, y=0.0, angular=0.0):
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._raw_cmd_finite = True
        self.guard._raw_linear_x = x
        self.guard._raw_linear_y = y
        self.guard._raw_angular_z = angular

    def _set_path_status(
        self, state=_ModuleStatus.RUNNING, valid=True, detail="following"
    ):
        status = _ModuleStatus()
        status.state = state
        status.valid = valid
        status.confidence = 0.8 if valid else 0.0
        status.detail = detail
        self.guard._path_status = status
        self.guard._last_path_status_time = _MockTime.now()

    def _run(self):
        self.guard._control_loop(None)
        return (
            self.guard._cmd_vel_pub.messages[-1],
            self.guard._status_pub.messages[-1],
        )

    def test_constructor_declares_unique_public_topics(self):
        ROSPY.publishers.clear()
        ROSPY.subscribers.clear()
        CmdVelGuard()
        publisher_topics = [publisher.topic for publisher in ROSPY.publishers]
        self.assertEqual(publisher_topics.count("/cmd_vel"), 1)
        self.assertEqual(
            publisher_topics.count("/competition/control/status"), 1
        )
        self.assertIn(
            "/competition/control/path_follower_status", ROSPY.subscribers
        )

    def test_never_received_path_status_is_starting_and_zero(self):
        self._set_fresh_raw()
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.STARTING)
        self.assertFalse(status.valid)

    def test_expired_path_status_is_timeout_and_zero(self):
        self._set_fresh_raw()
        self._set_path_status()
        self.guard._last_path_status_time = _MockTimeInstance(8.0)
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertFalse(status.valid)

    def test_path_status_time_rollback_is_timeout(self):
        self._set_fresh_raw()
        self._set_path_status()
        self.guard._last_path_status_time = _MockTimeInstance(11.0)
        _, status = self._run()
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertIn("backwards", status.detail)

    def test_failed_path_status_stops_and_is_propagated(self):
        self._set_fresh_raw()
        self._set_path_status(_ModuleStatus.FAILED, False, "frame mismatch")
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.FAILED)
        self.assertFalse(status.valid)
        self.assertEqual(status.detail, "frame mismatch")

    def test_valid_false_running_status_stops(self):
        self._set_fresh_raw()
        self._set_path_status(_ModuleStatus.RUNNING, False, "invalid path")
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)

    def test_running_valid_status_forwards_limited_velocity(self):
        self._set_fresh_raw(x=2.0, y=-1.0, angular=3.0)
        self._set_path_status()
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.5)
        self.assertEqual(cmd.linear.y, -0.3)
        self.assertEqual(cmd.angular.z, 1.0)
        self.assertEqual(status.state, _ModuleStatus.RUNNING)
        self.assertTrue(status.valid)

    def test_succeeded_status_is_public_and_zero(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status(
            _ModuleStatus.SUCCEEDED, True, "path finished"
        )
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.SUCCEEDED)
        self.assertTrue(status.valid)

    def test_estop_overrides_running_status(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()
        self.guard._emergency_stop = True
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.BLOCKED)
        self.assertFalse(status.valid)
        self.assertIn("emergency_stop", status.detail)

    def test_disabled_overrides_running_status(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()
        self.guard._control_enabled = False
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.READY)
        self.assertFalse(status.valid)

    def test_non_finite_raw_velocity_is_failed_and_zero(self):
        self._set_path_status()
        message = _Twist()
        message.linear.x = math.nan
        self.guard._raw_cmd_callback(message)
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.FAILED)
        self.assertFalse(status.valid)

    def test_raw_timeout_overrides_running_status(self):
        self._set_path_status()
        self.guard._last_raw_cmd_time = _MockTimeInstance(9.0)
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertFalse(status.valid)

    def test_raw_time_rollback_is_timeout(self):
        self._set_path_status()
        self.guard._last_raw_cmd_time = _MockTimeInstance(11.0)
        _, status = self._run()
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertIn("backwards", status.detail)

    def test_unused_twist_dof_remain_zero(self):
        self._set_fresh_raw()
        self._set_path_status()
        cmd, _ = self._run()
        self.assertEqual(cmd.linear.z, 0.0)
        self.assertEqual(cmd.angular.x, 0.0)
        self.assertEqual(cmd.angular.y, 0.0)


if __name__ == "__main__":
    unittest.main()
