#!/usr/bin/env python3
"""Pure-Python tests for control status aggregation and velocity gating."""

from __future__ import annotations

import math
import sys
import threading
import time
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
        self.guard._lock = threading.RLock()
        self.guard._generation = 0
        self.guard._last_ros_time = 10.0
        self.guard._min_path_status_stamp = None
        self.guard._raw_generation = None
        self.guard._path_status_generation = None

        self.guard._timeout = 0.5
        self.guard._path_status_timeout = 1.0
        self.guard._max_future_stamp_skew = 0.2
        self.guard._max_linear_x = 0.5
        self.guard._max_linear_y = 0.3
        self.guard._max_angular_z = 1.0

        self.guard._last_raw_cmd_time = None
        self.guard._last_path_status_time = None
        self.guard._path_status_stamp = None
        self.guard._path_status = None
        self.guard._path_status_error = None
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
        self.guard._raw_generation = self.guard._generation
        self.guard._raw_cmd_finite = True
        self.guard._raw_linear_x = x
        self.guard._raw_linear_y = y
        self.guard._raw_angular_z = angular

    def _set_path_status(
        self, state=_ModuleStatus.RUNNING, valid=True, detail="following"
    ):
        status = _ModuleStatus()
        status.header.stamp = _MockTime.now()
        status.state = state
        status.valid = valid
        status.confidence = 0.8 if valid else 0.0
        status.detail = detail
        self.guard._path_status = status
        self.guard._last_path_status_time = _MockTime.now()
        self.guard._path_status_stamp = status.header.stamp
        self.guard._path_status_generation = self.guard._generation
        self.guard._path_status_error = None
        return status

    def _run(self):
        self.guard._control_loop(None)
        return (
            self.guard._cmd_vel_pub.messages[-1],
            self.guard._status_pub.messages[-1],
        )

    def _assert_safety_callback_is_atomic(self, callback):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        entered = threading.Event()
        release = threading.Event()
        callback_done = threading.Event()
        original_select = self.guard._select_status

        def blocking_select(now):
            result = original_select(now)
            entered.set()
            self.assertTrue(release.wait(1.0))
            return result

        self.guard._select_status = blocking_select

        control_thread = threading.Thread(
            target=self.guard._control_loop,
            args=(None,),
        )
        control_thread.start()
        self.assertTrue(entered.wait(1.0))

        def run_callback():
            callback()
            callback_done.set()

        callback_thread = threading.Thread(target=run_callback)
        callback_thread.start()

        # The callback must not be able to change safety state after the
        # decision but before the same locked control-loop publishes.
        time.sleep(0.02)
        self.assertFalse(callback_done.is_set())

        release.set()
        control_thread.join(1.0)
        callback_thread.join(1.0)
        self.assertFalse(control_thread.is_alive())
        self.assertFalse(callback_thread.is_alive())
        self.assertTrue(callback_done.is_set())

        count_after_callback = len(self.guard._cmd_vel_pub.messages)
        self.guard._select_status = original_select
        self.guard._control_loop(None)

        # Once the safety callback has completed, no later command is nonzero.
        for cmd in self.guard._cmd_vel_pub.messages[count_after_callback:]:
            self.assertEqual(cmd.linear.x, 0.0)
            self.assertEqual(cmd.linear.y, 0.0)
            self.assertEqual(cmd.angular.z, 0.0)

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

    def test_follower_invalid_then_valid_does_not_reuse_old_raw(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        invalid = _ModuleStatus()
        invalid.header.stamp = _MockTime.now()
        invalid.state = _ModuleStatus.FAILED
        invalid.valid = False
        invalid.detail = "follower invalid"
        self.guard._path_status_callback(invalid)

        self.assertIsNone(self.guard._raw_generation)
        self.assertEqual(self.guard._raw_linear_x, 0.0)

        valid = _ModuleStatus()
        valid.header.stamp = _MockTime.now()
        valid.state = _ModuleStatus.RUNNING
        valid.valid = True
        valid.confidence = 0.8
        valid.detail = "follower recovered"
        self.guard._path_status_callback(valid)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertIn("raw_cmd_vel", status.detail)

    def test_follower_invalid_new_raw_then_valid_recovers(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        invalid = _ModuleStatus()
        invalid.header.stamp = _MockTime.now()
        invalid.state = _ModuleStatus.FAILED
        invalid.valid = False
        invalid.detail = "follower invalid"
        self.guard._path_status_callback(invalid)

        raw = _Twist()
        raw.linear.x = 0.3
        self.guard._raw_cmd_callback(raw)

        valid = _ModuleStatus()
        valid.header.stamp = _MockTime.now()
        valid.state = _ModuleStatus.RUNNING
        valid.valid = True
        valid.confidence = 0.8
        valid.detail = "follower recovered"
        self.guard._path_status_callback(valid)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.3)
        self.assertTrue(status.valid)

    def test_follower_invalid_valid_then_new_raw_recovers(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        invalid = _ModuleStatus()
        invalid.header.stamp = _MockTime.now()
        invalid.state = _ModuleStatus.FAILED
        invalid.valid = False
        invalid.detail = "follower invalid"
        self.guard._path_status_callback(invalid)

        valid = _ModuleStatus()
        valid.header.stamp = _MockTime.now()
        valid.state = _ModuleStatus.RUNNING
        valid.valid = True
        valid.confidence = 0.8
        valid.detail = "follower recovered"
        self.guard._path_status_callback(valid)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)

        raw = _Twist()
        raw.linear.x = 0.3
        self.guard._raw_cmd_callback(raw)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.3)
        self.assertTrue(status.valid)

    def test_rollback_triggering_raw_is_discarded(self):
        _MockTime.set(100.0)
        self.guard._last_ros_time = 100.0
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        _MockTime.set(10.0)
        triggering_raw = _Twist()
        triggering_raw.linear.x = 0.35
        self.guard._raw_cmd_callback(triggering_raw)

        self.assertIsNone(self.guard._last_raw_cmd_time)
        self.assertIsNone(self.guard._raw_generation)
        self.assertEqual(self.guard._raw_linear_x, 0.0)

        cmd = self.guard._cmd_vel_pub.messages[-1]
        status = self.guard._status_pub.messages[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertFalse(status.valid)

    def test_invalid_follower_timestamp_invalidates_raw_generation(self):
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        invalid_stamp = _ModuleStatus()
        invalid_stamp.header.stamp = _MockTimeInstance(10.5)
        invalid_stamp.state = _ModuleStatus.RUNNING
        invalid_stamp.valid = True
        invalid_stamp.detail = "future stamp"

        self.guard._path_status_callback(invalid_stamp)

        self.assertIsNone(self.guard._raw_generation)
        self.assertEqual(self.guard._raw_linear_x, 0.0)
        self.assertIsNotNone(self.guard._path_status_error)
        self.assertIn("future", self.guard._path_status_error)

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

    # ------------------------------------------------------------------
    # Second-review threading regressions
    # ------------------------------------------------------------------

    def test_estop_during_control_loop_decision_is_atomic(self):
        self._assert_safety_callback_is_atomic(
            lambda: self.guard._estop_callback(_Bool(True))
        )

    def test_control_disabled_during_control_loop_decision_is_atomic(self):
        self._assert_safety_callback_is_atomic(
            lambda: self.guard._enabled_callback(_Bool(False))
        )

    def test_follower_invalid_during_control_loop_decision_is_atomic(self):
        invalid = _ModuleStatus()
        invalid.header.stamp = _MockTime.now()
        invalid.state = _ModuleStatus.FAILED
        invalid.valid = False
        invalid.detail = "follower invalid"
        self._assert_safety_callback_is_atomic(
            lambda: self.guard._path_status_callback(invalid)
        )

    # ------------------------------------------------------------------
    # Second-review rollback / timestamp regressions
    # ------------------------------------------------------------------

    def test_rollback_clears_cached_raw_and_path_status(self):
        _MockTime.set(100.0)
        self.guard._last_ros_time = 100.0
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        _MockTime.set(10.0)
        cmd, status = self._run()

        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertIsNone(self.guard._last_raw_cmd_time)
        self.assertIsNone(self.guard._last_path_status_time)
        self.assertIsNone(self.guard._path_status)
        self.assertEqual(self.guard._raw_linear_x, 0.0)
        self.assertEqual(self.guard._generation, 1)
        self.assertEqual(self.guard._min_path_status_stamp, 10.0)

    def test_rollback_requires_fresh_raw_and_status_before_recovery(self):
        _MockTime.set(100.0)
        self.guard._last_ros_time = 100.0
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        _MockTime.set(10.0)
        cmd, _ = self._run()
        self.assertEqual(cmd.linear.x, 0.0)

        raw = _Twist()
        raw.linear.x = 0.4
        self.guard._raw_cmd_callback(raw)
        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)

        fresh_status = _ModuleStatus()
        fresh_status.header.stamp = _MockTime.now()
        fresh_status.state = _ModuleStatus.RUNNING
        fresh_status.valid = True
        fresh_status.confidence = 0.8
        fresh_status.detail = "fresh generation"
        self.guard._path_status_callback(fresh_status)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.4)
        self.assertTrue(status.valid)
        self.assertEqual(status.state, _ModuleStatus.RUNNING)

    def test_callback_detected_rollback_publishes_zero_invalid_immediately(self):
        _MockTime.set(100.0)
        self.guard._last_ros_time = 100.0
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.4)
        self.assertTrue(status.valid)

        _MockTime.set(10.0)
        raw = _Twist()
        raw.linear.x = 0.2
        self.guard._raw_cmd_callback(raw)

        cmd = self.guard._cmd_vel_pub.messages[-1]
        status = self.guard._status_pub.messages[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.linear.y, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)
        self.assertEqual(status.state, _ModuleStatus.TIMEOUT)
        self.assertFalse(status.valid)
        self.assertIsNone(self.guard._path_status)

    def test_second_rollback_resets_epoch_cutoff(self):
        self.guard._last_ros_time = 100.0

        _MockTime.set(10.0)
        self._run()
        self.assertEqual(self.guard._min_path_status_stamp, 10.0)

        _MockTime.set(20.0)
        self._run()
        _MockTime.set(5.0)
        self._run()

        self.assertEqual(self.guard._min_path_status_stamp, 5.0)

        raw = _Twist()
        raw.linear.x = 0.3
        self.guard._raw_cmd_callback(raw)

        fresh_status = _ModuleStatus()
        fresh_status.header.stamp = _MockTime.now()
        fresh_status.state = _ModuleStatus.RUNNING
        fresh_status.valid = True
        fresh_status.confidence = 0.8
        fresh_status.detail = "second epoch"
        self.guard._path_status_callback(fresh_status)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.3)
        self.assertTrue(status.valid)

    def test_follower_status_zero_stamp_is_rejected(self):
        self._set_fresh_raw()
        status_msg = _ModuleStatus()
        status_msg.header.stamp = _MockTimeInstance(0.0)
        status_msg.state = _ModuleStatus.RUNNING
        status_msg.valid = True

        self.guard._path_status_callback(status_msg)
        cmd, status = self._run()

        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertIn("non-zero", status.detail)

    def test_follower_status_stale_stamp_is_rejected(self):
        self._set_fresh_raw()
        status_msg = _ModuleStatus()
        status_msg.header.stamp = _MockTimeInstance(8.0)
        status_msg.state = _ModuleStatus.RUNNING
        status_msg.valid = True

        self.guard._path_status_callback(status_msg)
        cmd, status = self._run()

        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertIn("stale", status.detail)

    def test_follower_status_future_stamp_is_rejected(self):
        self._set_fresh_raw()
        status_msg = _ModuleStatus()
        status_msg.header.stamp = _MockTimeInstance(10.5)
        status_msg.state = _ModuleStatus.RUNNING
        status_msg.valid = True

        self.guard._path_status_callback(status_msg)
        cmd, status = self._run()

        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertIn("future", status.detail)

    def test_follower_status_old_generation_stamp_is_rejected(self):
        _MockTime.set(100.0)
        self.guard._last_ros_time = 100.0
        self._set_fresh_raw(x=0.4)
        self._set_path_status()

        _MockTime.set(10.0)
        self._run()

        raw = _Twist()
        raw.linear.x = 0.4
        self.guard._raw_cmd_callback(raw)

        delayed = _ModuleStatus()
        delayed.header.stamp = _MockTimeInstance(9.5)
        delayed.state = _ModuleStatus.RUNNING
        delayed.valid = True
        delayed.detail = "old generation"
        self.guard._path_status_callback(delayed)

        cmd, status = self._run()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertFalse(status.valid)
        self.assertIn("old generation", status.detail)


if __name__ == "__main__":
    unittest.main()
