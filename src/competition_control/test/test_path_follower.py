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

    def logwarn(self, *_args, **_kwargs):
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
    def __init__(
        self,
        frame_id="world",
        x=0.0,
        y=0.0,
        stamp=None,
        yaw=0.0,
    ):
        self.header = _Header(frame_id, stamp)
        self.pose = _Pose(x, y)
        self.pose.orientation.z = math.sin(yaw / 2.0)
        self.pose.orientation.w = math.cos(yaw / 2.0)


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


class _Bool:
    def __init__(self, data=False):
        self.data = data


class _StdMsgsMsg:
    Bool = _Bool


class _StdMsgs:
    msg = _StdMsgsMsg()


sys.modules["std_msgs"] = _StdMsgs()
sys.modules["std_msgs.msg"] = _StdMsgsMsg()


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
        self.follower.orientation_tolerance = 0.15
        self.follower.min_linear = 0.0
        self.follower.stuck_duration_sec = 0.0
        self.follower.stuck_progress_epsilon = 0.05
        self.follower.stuck_progress_ratio = 0.2
        self.follower.stuck_expected_progress_min = 0.10
        self.follower._stuck_expected_progress = 0.0
        self.follower._stuck_last_command_time = None
        self.follower.stuck_backoff_linear = 0.0
        self.follower.stuck_backoff_duration = 3.0
        self.follower.stuck_backoff_max = 3
        self.follower._stuck_start_time = None
        self.follower._stuck_start_position = None
        self.follower._stuck_notified = False
        self.follower._recovery_until = None
        self.follower._stuck_backoff_count = 0
        self.follower.stuck_pub = None
        self.follower.path_timeout = _MockDuration(5.0)
        self.follower.odom_timeout = _MockDuration(1.0)
        self.follower.planning_status_timeout = _MockDuration(3.0)
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
        yaw=0.0,
    ):
        msg = _Odometry(frame, child, x, y, stamp)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.follower.on_odom(msg)
        return msg

    def _rotation_path(self, target_yaw, current_yaw=0.0, stamp=None):
        poses = [
            _PoseStamped(
                "world",
                0.0,
                0.0,
                stamp,
                yaw=current_yaw,
            ),
            _PoseStamped(
                "world",
                0.0,
                0.0,
                stamp,
                yaw=target_yaw,
            ),
        ]
        msg = _Path("world", poses, stamp)
        self.follower.on_path(msg)
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

    def test_min_linear_floors_close_approach_command(self):
        # R24 RETURNING stall: 0.6*distance falls below the RL policy's
        # minimum executable speed just outside goal_tolerance, freezing the
        # robot in place. The floor must keep the drive command executable.
        self.follower.min_linear = 0.25
        self._arm(points=[(0.21, 0.0)])
        self.follower.step()
        self.assertAlmostEqual(self._last_cmd().linear.x, 0.25)
        self.assertEqual(self._last_status().state, _ModuleStatus.RUNNING)

    def test_min_linear_does_not_block_finish(self):
        self.follower.min_linear = 0.25
        self._arm(points=[(0.05, 0.0)])
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.SUCCEEDED)

    def test_min_linear_does_not_drive_during_in_place_turn(self):
        self.follower.min_linear = 0.25
        self._arm(points=[(-1.0, 0.0)])
        self.follower.step()
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.assertNotEqual(self._last_cmd().angular.z, 0.0)

    def test_same_position_orientation_waypoint_commands_in_place_turn(self):
        self._planning(True)
        self._localization(True)
        self._odom(yaw=0.0)
        self._rotation_path(math.pi / 2.0, current_yaw=0.0)

        self.follower.step()
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.assertNotEqual(self._last_cmd().angular.z, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.RUNNING)

    def test_rotation_waypoint_finishes_after_yaw_reached(self):
        self._planning(True)
        self._localization(True)
        self._odom(yaw=0.0)
        self._rotation_path(math.pi / 2.0, current_yaw=0.0)
        self.follower.step()
        self.assertNotEqual(self._last_cmd().angular.z, 0.0)

        _MockTime.set(10.1)
        self._odom(yaw=math.pi / 2.0, stamp=10.1)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.SUCCEEDED)
        self.assertTrue(self._last_status().valid)

    def test_rotation_waypoint_does_not_inherit_minimum_forward_speed(self):
        self.follower.min_linear = 0.25
        self._planning(True)
        self._localization(True)
        self._odom(yaw=0.0)
        self._rotation_path(-math.pi / 2.0, current_yaw=0.0)
        self.follower.step()
        self.assertEqual(self._last_cmd().linear.x, 0.0)
        self.assertNotEqual(self._last_cmd().angular.z, 0.0)

    def test_normal_path_still_uses_position_heading(self):
        self._arm(points=[(1.0, 0.0)])
        # Deliberately make the pose orientation disagree with the travel
        # direction.  Normal navigation remains position-driven.
        self.follower.path[0].pose.orientation.z = math.sin(math.pi / 4.0)
        self.follower.path[0].pose.orientation.w = math.cos(math.pi / 4.0)
        self.follower.step()
        self.assertGreater(self._last_cmd().linear.x, 0.0)

    # ------------------------------------------------------------------
    # Stuck detection (advisory notice to planner)
    # ------------------------------------------------------------------

    def _arm_stuck(self, duration=5.0, backoff_linear=0.0, backoff_max=3):
        self.follower.stuck_duration_sec = duration
        self.follower.stuck_backoff_linear = backoff_linear
        self.follower.stuck_backoff_max = backoff_max
        self.follower.stuck_pub = _Publisher("/competition/planning/stuck")
        self._arm()
        # on_path(_arm) resets stuck and publishes a cleared (False) latch.
        self.assertEqual(len(self.follower.stuck_pub.messages), 1)
        self.assertFalse(self.follower.stuck_pub.messages[0].data)

    def _stuck_true_count(self):
        return sum(
            1 for msg in self.follower.stuck_pub.messages
            if getattr(msg, "data", False)
        )

    def _fresh_stuck_step(self, t, x=0.0, y=0.0, points=None):
        """Re-publish fresh inputs at time t with the given measured pose.

        Keeps planning/localization/odom/path stamps fresh (as the real stack
        does at 10 Hz) so the stuck window is what expires, not the inputs.
        Position stays (x, y): progress toward the waypoint is whatever the
        caller chooses.  Same-geometry path refresh does NOT re-arm stuck.
        """
        _MockTime.set(t)
        self._planning(True)
        self._localization(True)
        self._odom(x=x, y=y)
        self._path(points=points if points is not None else [(1.0, 0.0)])
        self.follower.step()

    def test_stuck_notice_after_duration_without_progress(self):
        self._arm_stuck(duration=5.0)
        self._fresh_stuck_step(10.0)  # anchor at (0,0), t=10
        self.assertEqual(self._stuck_true_count(), 0)
        self._fresh_stuck_step(16.0)  # 6 s without motion -> stuck
        self.assertEqual(self._stuck_true_count(), 1)
        # one-shot: no repeat on further stuck steps
        self._fresh_stuck_step(20.0)
        self.assertEqual(self._stuck_true_count(), 1)

    def test_stuck_window_resets_on_progress(self):
        self._arm_stuck(duration=5.0)
        self._fresh_stuck_step(10.0, x=0.0)  # anchor at (0,0)
        self._fresh_stuck_step(14.0, x=0.3)  # moved 0.3 m -> window restarts
        self._fresh_stuck_step(18.5, x=0.3)  # 4.5 s < duration -> no notice
        self.assertEqual(self._stuck_true_count(), 0)
        self._fresh_stuck_step(20.0, x=0.3)  # 6 s since reset -> stuck
        self.assertEqual(self._stuck_true_count(), 1)

    # ------------------------------------------------------------------
    # Command-relative progress gate (run-f1 creeping-livelock regression)
    # ------------------------------------------------------------------

    def _arm_relative(self, duration=8.0, ratio=0.2):
        self._arm_stuck(duration=duration)
        self.follower.stuck_progress_ratio = ratio
        self.follower.stuck_expected_progress_min = 0.10

    def test_creeping_livelock_is_detected_as_stuck(self):
        """run-f1 regression: creep that stays just above the absolute epsilon.

        run-f1 commanded 0.25 m/s while the robot achieved 0.006-0.010 m/s
        (2-4 % of command) for 281 sim s.  Every 8 s window moved 0.056-0.083 m
        -- MORE than the 0.05 m absolute epsilon -- so the legacy window reset
        forever and neither the backoff nor the planner stuck signal ever fired.
        The command-relative gate must catch exactly this case.
        """
        self._arm_relative(duration=8.0)
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        self.assertEqual(self._stuck_true_count(), 0)

        # ~0.008 m/s creep; by t=18.5 the robot has moved 0.068 m > epsilon
        for t in (12.0, 14.0, 16.0, 18.5):
            self._fresh_stuck_step(t, x=0.008 * (t - 10.0), points=[(10.0, 0.0)])

        self.assertGreater(
            0.008 * 8.5, self.follower.stuck_progress_epsilon,
            "test must exercise the case the absolute epsilon cannot see",
        )
        self.assertEqual(self._stuck_true_count(), 1)

    def test_creeping_livelock_still_commands_backoff(self):
        """The relative gate must drive the EXISTING backoff, not just the flag."""
        self._arm_stuck(duration=8.0, backoff_linear=0.15)
        self.follower.stuck_progress_ratio = 0.2
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        for t in (12.0, 14.0, 16.0, 18.5):
            self._fresh_stuck_step(t, x=0.008 * (t - 10.0), points=[(10.0, 0.0)])
        self.assertEqual(self._stuck_true_count(), 1)
        self.assertEqual(self.follower._stuck_backoff_count, 1)
        # next step is inside the recovery window -> backward command
        self._fresh_stuck_step(18.6, x=0.068, points=[(10.0, 0.0)])
        self.assertLess(self.follower.raw_vel_pub.messages[-1].linear.x, 0.0)

    def test_healthy_progress_is_not_stuck(self):
        """Progress that tracks the command must never be declared stuck."""
        self._arm_relative(duration=8.0)
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        for t in (12.0, 14.0, 16.0, 18.0, 20.0):
            self._fresh_stuck_step(t, x=0.25 * (t - 10.0), points=[(10.0, 0.0)])
        self.assertEqual(self._stuck_true_count(), 0)

    def test_slower_but_tracking_progress_is_not_stuck(self):
        """Genuine motion at 60 % of command is slow, not stuck."""
        self._arm_relative(duration=8.0)
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        for t in (12.0, 14.0, 16.0, 18.0, 20.0):
            self._fresh_stuck_step(t, x=0.15 * (t - 10.0), points=[(10.0, 0.0)])
        self.assertEqual(self._stuck_true_count(), 0)

    def test_precision_approach_is_not_falsely_stuck(self):
        """Short low-speed approach that tracks its command stays clean."""
        self._arm_relative(duration=8.0)
        self.follower.min_linear = 0.0
        self._fresh_stuck_step(10.0, x=0.0, points=[(0.3, 0.0)])
        self._fresh_stuck_step(11.0, x=0.18, points=[(0.3, 0.0)])
        self._fresh_stuck_step(12.0, x=0.29, points=[(0.3, 0.0)])
        self.assertEqual(self._stuck_true_count(), 0)

    def test_pure_rotation_does_not_arm_relative_gate(self):
        """In-place yaw turns command zero translation, so the gate stays off."""
        self._arm_relative(duration=2.0)
        # target yaw far from the robot's -> rotation_only path, linear.x == 0
        _MockTime.set(10.0)
        self._planning(True)
        self._localization(True)
        self._odom(x=0.0, y=0.0)
        poses = [_PoseStamped("world", 0.0, 0.0, None, yaw=math.pi)]
        self.follower.on_path(_Path("world", poses, None))
        self.follower.step()
        self.assertEqual(self.follower.raw_vel_pub.messages[-1].linear.x, 0.0)
        for t in (12.0, 14.0, 16.0):
            _MockTime.set(t)
            self._planning(True)
            self._localization(True)
            self._odom(x=0.0, y=0.0)
            self.follower.on_path(_Path("world", poses, None))
            self.follower.step()
        self.assertEqual(self._stuck_true_count(), 0)

    def test_zero_command_and_hold_do_not_arm_relative_gate(self):
        """No path / no driving means no expected progress and no recovery."""
        self._arm_relative(duration=2.0)
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        # drop the path entirely (HOLD analogue): planner reports invalid
        _MockTime.set(11.0)
        self._planning(False)
        self._localization(True)
        self._odom(x=0.0, y=0.0)
        for t in (13.0, 15.0, 17.0):
            _MockTime.set(t)
            self._planning(False)
            self._localization(True)
            self._odom(x=0.0, y=0.0)
            self.follower.step()
        self.assertEqual(self._stuck_true_count(), 0)

    def test_window_resets_when_relative_progress_recovers(self):
        """A livelock that breaks out must clear the window, not stay latched."""
        self._arm_relative(duration=8.0)
        self._fresh_stuck_step(10.0, x=0.0, points=[(10.0, 0.0)])
        # creeping: stays below the ratio but the window is not yet expired
        self._fresh_stuck_step(12.0, x=0.016, points=[(10.0, 0.0)])
        self.assertEqual(self.follower._stuck_expected_progress > 0.0, True)
        # breaks free and tracks the command -> window resets
        self._fresh_stuck_step(13.0, x=0.266, points=[(10.0, 0.0)])
        self.assertEqual(self.follower._stuck_expected_progress, 0.0)
        self._fresh_stuck_step(14.0, x=0.516, points=[(10.0, 0.0)])
        self.assertEqual(self._stuck_true_count(), 0)

    def test_stuck_disabled_by_default(self):
        self._arm()  # stuck_duration_sec = 0.0, stuck_pub None
        self.follower.step()
        _MockTime.set(60.0)
        self.follower.step()
        self.assertIsNone(self.follower.stuck_pub)

    def test_stuck_rearmed_on_new_path_geometry(self):
        self._arm_stuck(duration=5.0)
        self._fresh_stuck_step(10.0)
        self._fresh_stuck_step(16.0)
        self.assertEqual(self._stuck_true_count(), 1)
        # new path geometry re-arms and publishes a cleared latch
        self._path(points=[(2.0, 1.0)], stamp=16.0)
        self.assertEqual(self._stuck_true_count(), 1)
        self.assertFalse(self.follower.stuck_pub.messages[-1].data)
        self.assertFalse(self.follower._stuck_notified)
        # same-geometry refresh must NOT re-arm (commit republish case)
        self._fresh_stuck_step(17.0, points=[(2.0, 1.0)])
        self.assertEqual(self._stuck_true_count(), 1)

    # ------------------------------------------------------------------
    # Stuck backoff (physical un-wedge: brief backward command)
    # ------------------------------------------------------------------

    def test_stuck_backoff_issues_backward_command(self):
        self._arm_stuck(duration=5.0, backoff_linear=0.15)
        self._fresh_stuck_step(10.0)   # anchor
        self._fresh_stuck_step(16.0)   # stuck fires; recovery starts next step
        self.assertEqual(self._stuck_true_count(), 1)
        self.assertIsNotNone(self.follower._recovery_until)
        # recovery active -> backward command
        self._fresh_stuck_step(17.0)
        self.assertLess(self._last_cmd().linear.x, 0.0)
        self.assertIn("backing up", self._last_status().detail)
        # recovery still active
        self._fresh_stuck_step(18.0)
        self.assertLess(self._last_cmd().linear.x, 0.0)
        # recovery over (16 + 3 = 19) -> forward again
        self._fresh_stuck_step(19.5)
        self.assertGreater(self._last_cmd().linear.x, 0.0)

    def test_stuck_backoff_is_bounded(self):
        self._arm_stuck(duration=5.0, backoff_linear=0.15, backoff_max=1)
        self._fresh_stuck_step(10.0)   # anchor
        self._fresh_stuck_step(16.0)   # backoff #1 (recovery_until=19)
        self.assertEqual(self.follower._stuck_backoff_count, 1)
        self._fresh_stuck_step(17.0)   # recovery -> backward
        self.assertLess(self._last_cmd().linear.x, 0.0)
        # re-wedge: stuck #2 at 21, but budget (max=1) is used -> no backoff
        self._fresh_stuck_step(21.0)
        self.assertEqual(self.follower._stuck_backoff_count, 1)
        self.assertFalse(self.follower._recovery_active(
            _MockTimeInstance(22.0)))
        self._fresh_stuck_step(22.0)   # not in recovery -> forward
        self.assertGreater(self._last_cmd().linear.x, 0.0)

    def test_stuck_backoff_resets_on_new_path(self):
        self._arm_stuck(duration=5.0, backoff_linear=0.15)
        self._fresh_stuck_step(10.0)
        self._fresh_stuck_step(16.0)   # backoff #1
        self.assertEqual(self.follower._stuck_backoff_count, 1)
        # new path geometry -> reset count and recovery
        self._path(points=[(2.0, 1.0)], stamp=16.0)
        self.assertEqual(self.follower._stuck_backoff_count, 0)
        self.assertIsNone(self.follower._recovery_until)
        self._fresh_stuck_step(17.0, points=[(2.0, 1.0)])
        self.assertGreater(self._last_cmd().linear.x, 0.0)

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

    def test_identical_path_refresh_keeps_waypoint_progress(self):
        self._arm(points=[(0.5, 0.0), (1.0, 0.0), (2.0, 0.0)])
        self.follower.target_index = 2
        _MockTime.set(10.1)
        self._path(
            points=[(0.5, 0.0), (1.0, 0.0), (2.0, 0.0)],
            stamp=10.1,
        )
        self.assertEqual(self.follower.target_index, 2)
        self.assertEqual(self.follower.path_stamp.to_sec(), 10.1)

    def test_geometry_change_still_resets_waypoint_progress(self):
        self._arm(points=[(0.5, 0.0), (1.0, 0.0), (2.0, 0.0)])
        self.follower.target_index = 2
        _MockTime.set(10.1)
        self._path(
            points=[(0.5, 0.0), (1.1, 0.0), (2.0, 0.0)],
            stamp=10.1,
        )
        self.assertEqual(self.follower.target_index, 0)

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
        self.follower.planning_status_received_at = _MockTimeInstance(6.0)
        self.follower.step()
        self._assert_zero()
        self.assertEqual(self._last_status().state, _ModuleStatus.TIMEOUT)
        self.assertEqual(self.follower.path, [])

    def test_planning_status_two_second_period_fits_three_second_timeout(self):
        self._arm()

        # Current Planning publishes status every 2 s. Allow that cadence
        # plus a small scheduling/jitter margin without a false timeout.
        _MockTime.set(12.5)
        self._localization(True, stamp=12.5)
        self._odom(stamp=12.5)

        self.follower.step()

        self.assertGreater(self._last_cmd().linear.x, 0.0)
        self.assertEqual(self._last_status().state, _ModuleStatus.RUNNING)
        self.assertTrue(self._last_status().valid)

    def test_planning_status_over_three_seconds_still_times_out(self):
        self._arm()

        _MockTime.set(13.1)
        self._localization(True, stamp=13.1)
        self._odom(stamp=13.1)

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
        self._planning(True, stamp=6.0)
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
