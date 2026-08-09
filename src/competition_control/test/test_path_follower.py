#!/usr/bin/env python3
"""Pure-Python safety/interface tests for path_follower_node.

The ROS API is mocked so these tests run without roscore. They cover the
v0.2.0 safety contract, including callback/step concurrency, Header timestamp
validation, generation invalidation, timeout behavior, and world/base frames.
"""

from __future__ import annotations

import math
import sys
import threading
import unittest
from pathlib import Path


# ---------------------------------------------------------------------------
# Mock rospy time
# ---------------------------------------------------------------------------


class _MockDuration:
    def __init__(self, seconds):
        self._seconds = float(seconds)

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
    _now = 10.0

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

    def get_param(self, _name, default):
        return default

    def Publisher(self, topic, *_args, **_kwargs):
        pub = _Publisher(topic)
        self.publishers.append(pub)
        return pub

    def Subscriber(self, topic, *_args, **_kwargs):
        self.subscribers.append(topic)
        return object()

    def init_node(self, *_args, **_kwargs):
        pass

    def loginfo(self, *_args, **_kwargs):
        pass

    def loginfo_throttle(self, *_args, **_kwargs):
        pass


ROSPY = _MockRospy()
sys.modules["rospy"] = ROSPY


# ---------------------------------------------------------------------------
# Mock ROS messages
# ---------------------------------------------------------------------------


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


class _GeometryMsgsMsg:
    Twist = _Twist


class _GeometryMsgs:
    msg = _GeometryMsgsMsg()


sys.modules["geometry_msgs"] = _GeometryMsgs()
sys.modules["geometry_msgs.msg"] = _GeometryMsgsMsg()


class _Header:
    def __init__(self, frame_id="", stamp=None):
        self.frame_id = frame_id
        self.stamp = _MockTime.now() if stamp is None else _MockTimeInstance(stamp)


class _Pose:
    def __init__(self, x=0.0, y=0.0):
        self.position = _Vector3(x, y, 0.0)
        self.orientation = _Quaternion()


class _PoseStamped:
    def __init__(self, frame_id="world", x=0.0, y=0.0, stamp=None):
        self.header = _Header(frame_id, stamp)
        self.pose = _Pose(x, y)


class _Path:
    def __init__(self, frame_id="world", poses=None, stamp=None):
        self.header = _Header(frame_id, stamp)
        self.poses = list(poses or [])


class _Odometry:
    def __init__(
        self,
        frame_id="world",
        child_frame_id="base",
        x=0.0,
        y=0.0,
        stamp=None,
    ):
        self.header = _Header(frame_id, stamp)
        self.child_frame_id = child_frame_id
        self.pose = type("PoseWithCovariance", (), {})()
        self.pose.pose = _Pose(x, y)


class _NavMsgsMsg:
    Path = _Path
    Odometry = _Odometry


class _NavMsgs:
    msg = _NavMsgsMsg()


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

    def __init__(self, stamp=None):
        self.header = _Header(stamp=stamp)
        self.state = self.UNKNOWN
        self.valid = False
        self.confidence = 0.0
        self.detail = ""


sys.modules["competition_interfaces"] = type(
    "CompetitionInterfaces", (), {}
)()
sys.modules["competition_interfaces.msg"] = type(
    "CompetitionInterfacesMsg",
    (),
    {"ModuleStatus": _ModuleStatus},
)()


# ---------------------------------------------------------------------------
# Load real node file
# ---------------------------------------------------------------------------


def _load_node():
    node_path = Path(__file__).resolve().parents[1] / "scripts" / "path_follower_node"
    namespace = {"__name__": "test_module"}
    with node_path.open(encoding="utf-8") as handle:
        source = handle.read()
    exec(compile(source, str(node_path), "exec"), namespace)
    return namespace


NODE = _load_node()
PathFollower = NODE["PathFollower"]


class TestPathFollower(unittest.TestCase):
    def setUp(self):
        _MockTime.set(10.0)
        self.follower = PathFollower.__new__(PathFollower)

        self.follower._lock = threading.RLock()
        self.follower._generation = 0
        self.follower._state_version = 0
        self.follower._last_ros_time = None
        self.follower._min_path_stamp = None
        self.follower._min_odom_stamp = None
        self.follower._min_planning_status_stamp = None
        self.follower._min_localization_status_stamp = None

        self.follower.path = []
        self.follower.path_frame = ""
        self.follower.path_received_at = None
        self.follower.path_stamp = None
        self.follower.path_error = None

        self.follower.odom = None
        self.follower.odom_received_at = None
        self.follower.odom_stamp = None
        self.follower.odom_error = None

        self.follower.planning_status = None
        self.follower.planning_status_received_at = None
        self.follower.planning_status_stamp = None
        self.follower.planning_status_error = None

        self.follower.localization_status = None
        self.follower.localization_status_received_at = None
        self.follower.localization_status_stamp = None
        self.follower.localization_status_error = None

        self.follower.target_index = 0
        self.follower.control_frequency = 10.0
        self.follower.max_linear = 0.25
        self.follower.max_angular = 0.8
        self.follower.goal_tolerance = 0.2
        self.follower.path_timeout = _MockDuration(5.0)
        self.follower.odom_timeout = _MockDuration(1.0)
        self.follower.planning_status_timeout = _MockDuration(1.0)
        self.follower.localization_status_timeout = _MockDuration(1.0)
        self.follower.max_future_stamp_skew = 0.2

        self.follower.raw_vel_pub = _Publisher("/competition/raw_cmd_vel")
        self.follower.status_pub = _Publisher(
            "/competition/control/path_follower_status"
        )

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _status(valid=True, state=None, detail="", stamp=None):
        msg = _ModuleStatus(stamp=stamp)
        msg.valid = bool(valid)
        msg.state = (
            _ModuleStatus.RUNNING if valid else _ModuleStatus.FAILED
        ) if state is None else state
        msg.confidence = 1.0 if valid else 0.0
        msg.detail = detail
        return msg

    def _planning(self, valid=True, stamp=None, detail=""):
        self.follower.on_planning_status(
            self._status(valid=valid, stamp=stamp, detail=detail)
        )

    def _localization(self, valid=True, stamp=None, detail=""):
        self.follower.on_localization_status(
            self._status(valid=valid, stamp=stamp, detail=detail)
        )

    def _odom(
        self,
        frame="world",
        child="base",
        x=0.0,
        y=0.0,
        stamp=None,
    ):
        msg = _Odometry(frame, child, x, y, stamp)
        self.follower.on_odom(msg)
        return msg

    def _path(
        self,
        frame="world",
        pose_frame=None,
        points=None,
        stamp=None,
    ):
        pose_frame = frame if pose_frame is None else pose_frame
        points = [(1.0, 0.0)] if points is None else points
        poses = [
            _PoseStamped(pose_frame, x, y, stamp)
            for x, y in points
        ]
        msg = _Path(frame, poses, stamp)
        self.follower.on_path(msg)
        return msg

    def _arm(self, points=None):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(points=points)

    def _last_status(self):
        return self.follower.status_pub.messages[-1]

    def _last_cmd(self):
        return self.follower.raw_vel_pub.messages[-1]

    def _assert_zero(self):
        cmd = self._last_cmd()
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.linear.y, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    def _assert_failed_with(self, text):
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.FAILED)
        self.assertFalse(self._last_status().valid)
        self.assertIn(text, self._last_status().detail)

    # ------------------------------------------------------------------
    # Interface
    # ------------------------------------------------------------------

    def test_constructor_uses_only_official_topics(self):
        ROSPY.publishers.clear()
        ROSPY.subscribers.clear()
        PathFollower()

        pubs = [pub.topic for pub in ROSPY.publishers]
        self.assertEqual(
            pubs,
            [
                "/competition/raw_cmd_vel",
                "/competition/control/path_follower_status",
            ],
        )
        self.assertEqual(
            ROSPY.subscribers,
            [
                "/competition/path",
                "/competition/planning/status",
                "/competition/localization/odom",
                "/competition/localization/status",
            ],
        )
        self.assertNotIn("/cmd_vel", pubs)
        self.assertNotIn("/competition/control/status", pubs)
        self.assertNotIn("/Odometry_gazebo", ROSPY.subscribers)

    # ------------------------------------------------------------------
    # Normal / terminal behavior
    # ------------------------------------------------------------------

    def test_normal_tracking_reports_running_and_nonzero(self):
        self._arm()
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.RUNNING)
        self.assertTrue(self._last_status().valid)

    def test_finished_path_reports_succeeded_and_zero(self):
        self._arm(points=[(0.05, 0.0)])
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.SUCCEEDED)
        self.assertTrue(self._last_status().valid)

    def test_missing_planning_status_stops(self):
        self._localization(True)
        self._odom()
        self._path()
        self.follower.step()
        self._assert_zero()
        self.assertIn("planning/status", self._last_status().detail)

    def test_missing_localization_status_stops(self):
        self._planning(True)
        self._odom()
        self._path()
        self.follower.step()
        self._assert_zero()
        self.assertIn("localization/status", self._last_status().detail)

    def test_empty_path_cancels_immediately(self):
        self._arm()
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)
        empty = _Path("world", [], stamp=10.0)
        self.follower.on_path(empty)
        self._assert_zero()
        self.assertEqual(self.follower.path, [])
        self.assertEqual(self.follower.target_index, 0)

    def test_empty_path_still_requires_nonzero_header_stamp(self):
        self._arm()
        empty = _Path("world", [], stamp=0.0)
        self.follower.on_path(empty)
        self._assert_failed_with("non-zero")
        self.assertEqual(self.follower.path, [])

    # ------------------------------------------------------------------
    # Invalid / recovery generation semantics
    # ------------------------------------------------------------------

    def test_planning_invalid_clears_old_path(self):
        self._arm()
        self._planning(False, detail="planner failed")
        self._assert_zero()
        self.assertEqual(self.follower.path, [])
        self.assertEqual(self._last_status().state, _ModuleStatus.BLOCKED)

    def test_planning_recovery_does_not_revive_old_path(self):
        self._arm()
        self._planning(False)
        _MockTime.set(10.1)
        self._planning(True)
        self.follower.step()
        self._assert_zero()
        self.assertIn("/competition/path", self._last_status().detail)

    def test_candidate_path_while_planning_invalid_can_run_after_recovery(self):
        self._arm()
        self._planning(False)
        _MockTime.set(10.1)
        self._path(points=[(2.0, 0.0)], stamp=10.1)
        self.follower.step()
        self._assert_zero()
        _MockTime.set(10.2)
        self._planning(True, stamp=10.2)
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)

    def test_repeated_planning_invalid_does_not_clear_candidate_path(self):
        self._arm()
        self._planning(False)
        _MockTime.set(10.1)
        self._path(stamp=10.1)
        _MockTime.set(10.2)
        self._planning(False, stamp=10.2)
        self.assertTrue(self.follower.path)

    def test_localization_invalid_clears_old_odom_and_path(self):
        self._arm()
        self._localization(False, detail="lost")
        self._assert_zero()
        self.assertIsNone(self.follower.odom)
        self.assertEqual(self.follower.path, [])

    def test_localization_recovery_plus_new_odom_needs_new_path(self):
        self._arm()
        self._localization(False)
        _MockTime.set(10.1)
        self._localization(True, stamp=10.1)
        self._odom(x=0.2, stamp=10.1)
        self.follower.step()
        self._assert_zero()
        self.assertIn("/competition/path", self._last_status().detail)

    def test_candidate_data_while_localization_invalid_runs_after_recovery(self):
        self._arm()
        self._localization(False)
        _MockTime.set(10.1)
        self._odom(x=0.1, stamp=10.1)
        self._path(stamp=10.1)
        self.follower.step()
        self._assert_zero()
        _MockTime.set(10.2)
        self._localization(True, stamp=10.2)
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)

    def test_repeated_localization_invalid_does_not_clear_candidates(self):
        self._arm()
        self._localization(False)
        _MockTime.set(10.1)
        self._odom(stamp=10.1)
        self._path(stamp=10.1)
        _MockTime.set(10.2)
        self._localization(False, stamp=10.2)
        self.assertIsNotNone(self.follower.odom)
        self.assertTrue(self.follower.path)

    # ------------------------------------------------------------------
    # Callback/step race blocker
    # ------------------------------------------------------------------

    def test_localization_invalid_during_compute_prevents_stale_nonzero_publish(self):
        self._arm()
        entered_compute = threading.Event()
        release_compute = threading.Event()
        original = self.follower._compute_follow_snapshot

        def blocked_compute(*args):
            entered_compute.set()
            self.assertTrue(release_compute.wait(2.0))
            return original(*args)

        self.follower._compute_follow_snapshot = blocked_compute

        worker = threading.Thread(target=self.follower.step)
        worker.start()
        self.assertTrue(entered_compute.wait(2.0))

        self._localization(False, detail="race invalid")
        count_after_invalid = len(self.follower.raw_vel_pub.messages)
        self._assert_zero()

        release_compute.set()
        worker.join(2.0)
        self.assertFalse(worker.is_alive())

        # Old step() must not publish anything after invalid callback.
        self.assertEqual(
            len(self.follower.raw_vel_pub.messages),
            count_after_invalid,
        )
        self._assert_zero()

    def test_planning_invalid_during_compute_prevents_stale_nonzero_publish(self):
        self._arm()
        entered_compute = threading.Event()
        release_compute = threading.Event()
        original = self.follower._compute_follow_snapshot

        def blocked_compute(*args):
            entered_compute.set()
            self.assertTrue(release_compute.wait(2.0))
            return original(*args)

        self.follower._compute_follow_snapshot = blocked_compute

        worker = threading.Thread(target=self.follower.step)
        worker.start()
        self.assertTrue(entered_compute.wait(2.0))

        self._planning(False, detail="race invalid")
        count_after_invalid = len(self.follower.raw_vel_pub.messages)
        self._assert_zero()

        release_compute.set()
        worker.join(2.0)
        self.assertFalse(worker.is_alive())
        self.assertEqual(
            len(self.follower.raw_vel_pub.messages),
            count_after_invalid,
        )

    # ------------------------------------------------------------------
    # Receive watchdog timeout
    # ------------------------------------------------------------------

    def test_planning_status_receive_timeout(self):
        self._arm()
        self.follower.planning_status_received_at = _MockTimeInstance(8.0)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertEqual(self.follower.path, [])

    def test_localization_status_receive_timeout(self):
        self._arm()
        self.follower.localization_status_received_at = _MockTimeInstance(8.0)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertIsNone(self.follower.odom)
        self.assertEqual(self.follower.path, [])

    def test_path_receive_timeout(self):
        self._arm()
        self.follower.path_received_at = _MockTimeInstance(4.0)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertEqual(self.follower.path, [])

    def test_odom_receive_timeout(self):
        self._arm()
        self.follower.odom_received_at = _MockTimeInstance(8.0)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertIsNone(self.follower.odom)
        self.assertEqual(self.follower.path, [])

    # ------------------------------------------------------------------
    # Header timestamp blocker: zero / stale / future for all 4 inputs
    # ------------------------------------------------------------------

    def test_path_zero_stamp_rejected(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(stamp=0.0)
        self._assert_failed_with("non-zero")

    def test_path_stale_stamp_rejected(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(stamp=4.0)
        self._assert_failed_with("stale")

    def test_path_future_stamp_rejected(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(stamp=10.5)
        self._assert_failed_with("future")

    def test_odom_zero_stamp_rejected(self):
        self._odom(stamp=0.0)
        self._assert_failed_with("non-zero")

    def test_odom_stale_stamp_rejected(self):
        self._odom(stamp=8.0)
        self._assert_failed_with("stale")

    def test_odom_future_stamp_rejected(self):
        self._odom(stamp=10.5)
        self._assert_failed_with("future")

    def test_planning_status_zero_stamp_rejected(self):
        self._planning(True, stamp=0.0)
        self._assert_failed_with("non-zero")

    def test_planning_status_stale_stamp_rejected(self):
        self._planning(True, stamp=8.0)
        self._assert_failed_with("stale")

    def test_planning_status_future_stamp_rejected(self):
        self._planning(True, stamp=10.5)
        self._assert_failed_with("future")

    def test_localization_status_zero_stamp_rejected(self):
        self._localization(True, stamp=0.0)
        self._assert_failed_with("non-zero")

    def test_localization_status_stale_stamp_rejected(self):
        self._localization(True, stamp=8.0)
        self._assert_failed_with("stale")

    def test_localization_status_future_stamp_rejected(self):
        self._localization(True, stamp=10.5)
        self._assert_failed_with("future")

    def test_message_stamp_can_timeout_even_when_receive_watchdog_is_fresh(self):
        self._arm()
        _MockTime.set(11.2)
        # Pretend callbacks were just invoked again without changing message
        # stamps. received_at is fresh but source data itself is stale.
        self.follower.odom_received_at = _MockTime.now()
        self.follower.localization_status_received_at = _MockTime.now()
        self.follower.planning_status_received_at = _MockTime.now()
        self.follower.path_received_at = _MockTime.now()
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)

    # ------------------------------------------------------------------
    # Delayed old-generation data
    # ------------------------------------------------------------------

    def test_old_path_after_planning_invalid_is_rejected(self):
        self._arm()
        _MockTime.set(10.2)
        self._planning(False, stamp=10.2)
        _MockTime.set(10.3)
        self._path(stamp=10.1)
        self._assert_failed_with("old generation")

    def test_old_odom_after_localization_invalid_is_rejected(self):
        self._arm()
        _MockTime.set(10.2)
        self._localization(False, stamp=10.2)
        _MockTime.set(10.3)
        self._odom(stamp=10.1)
        self._assert_failed_with("old generation")

    def test_old_planning_status_after_invalid_is_rejected(self):
        self._arm()
        _MockTime.set(10.2)
        self._planning(False, stamp=10.2)
        _MockTime.set(10.3)
        self._planning(True, stamp=10.1)
        self._assert_failed_with("old generation")

    def test_old_localization_status_after_invalid_is_rejected(self):
        self._arm()
        _MockTime.set(10.2)
        self._localization(False, stamp=10.2)
        _MockTime.set(10.3)
        self._localization(True, stamp=10.1)
        self._assert_failed_with("old generation")

    # ------------------------------------------------------------------
    # ROS clock rollback
    # ------------------------------------------------------------------

    def test_clock_rollback_invalidates_everything(self):
        self._arm()
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)

        _MockTime.set(2.0)
        self.follower.step()

        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertIn("backwards", self._last_status().detail)
        self.assertIsNone(self.follower.planning_status)
        self.assertIsNone(self.follower.localization_status)
        self.assertIsNone(self.follower.odom)
        self.assertEqual(self.follower.path, [])

    def test_callback_first_after_clock_rollback_still_resets_generation(self):
        self._arm()
        _MockTime.set(2.0)

        # Callback arrives before next step(); rollback must still be detected.
        self._planning(True, stamp=2.0)

        self.assertIsNotNone(self.follower.planning_status)
        self.assertIsNone(self.follower.localization_status)
        self.assertIsNone(self.follower.odom)
        self.assertEqual(self.follower.path, [])

        self.follower.step()
        self._assert_zero()
        self.assertIn("localization/status", self._last_status().detail)

    def test_old_epoch_future_message_after_rollback_is_rejected(self):
        self._arm()
        _MockTime.set(2.0)
        self._planning(True, stamp=2.0)  # detects rollback
        self._odom(stamp=10.0)          # delayed old-epoch data
        self._assert_failed_with("future")

    def test_new_generation_can_recover_after_rollback(self):
        self._arm()
        _MockTime.set(2.0)
        self.follower.step()  # rollback clears all

        _MockTime.set(2.1)
        self._planning(True, stamp=2.1)
        self._localization(True, stamp=2.1)
        self._odom(stamp=2.1)
        self._path(stamp=2.1)
        self.follower.step()

        self.assertGreater(self._last_cmd().linear.x, 0.0)
        self.assertTrue(self._last_status().valid)

    # ------------------------------------------------------------------
    # Frame / numeric validation
    # ------------------------------------------------------------------

    def test_path_header_frame_must_be_world(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(frame="map", pose_frame="map")
        self._assert_failed_with("world")

    def test_each_path_pose_frame_must_be_world(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(frame="world", pose_frame="map")
        self._assert_failed_with("world")

    def test_odom_frame_must_be_world(self):
        self._odom(frame="odom")
        self._assert_failed_with("world")

    def test_odom_child_frame_must_be_base(self):
        self._odom(child="base_link")
        self._assert_failed_with("base")

    def test_non_finite_path_rejected(self):
        self._planning(True)
        self._localization(True)
        self._odom()
        self._path(points=[(math.nan, 0.0)])
        self._assert_failed_with("NaN")

    def test_non_finite_odom_rejected(self):
        self._odom(x=math.inf)
        self._assert_failed_with("NaN or Inf")

    def test_zero_norm_quaternion_rejected(self):
        odom = _Odometry()
        odom.pose.pose.orientation = _Quaternion(0.0, 0.0, 0.0, 0.0)
        self.follower.on_odom(odom)
        self._assert_failed_with("zero norm")


if __name__ == "__main__":
    unittest.main()
