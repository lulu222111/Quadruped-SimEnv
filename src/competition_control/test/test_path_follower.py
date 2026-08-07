#!/usr/bin/env python3
"""Pure-Python safety and interface tests for path_follower_node.

These tests intentionally mock rospy and ROS message classes so that the
PathFollower safety logic can be tested without roscore, Gazebo, Planning,
Localization, or a real robot.

Covered behavior includes:

- official topic ownership
- Planning status gating
- Localization status gating
- empty Path cancellation
- Path / Odom / status timeouts
- world/base frame validation
- NaN / Inf protection
- ROS time rollback protection
- old data invalidation
- recovery only after fresh data arrives
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path


# ============================================================
# Mock rospy time
# ============================================================


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


# ============================================================
# Mock Publisher
# ============================================================


class _Publisher:
    def __init__(self, topic):
        self.topic = topic
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


# ============================================================
# Mock rospy
# ============================================================


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

    def Publisher(
        self,
        topic,
        *_args,
        **_kwargs,
    ):
        publisher = _Publisher(topic)

        self.publishers.append(publisher)

        return publisher

    def Subscriber(
        self,
        topic,
        *_args,
        **_kwargs,
    ):
        self.subscribers.append(topic)

        return object()

    def loginfo(
        self,
        *_args,
        **_kwargs,
    ):
        pass

    def loginfo_throttle(
        self,
        *_args,
        **_kwargs,
    ):
        pass


ROSPY = _MockRospy()

sys.modules["rospy"] = ROSPY


# ============================================================
# Mock geometry_msgs
# ============================================================


class _Vector3:
    def __init__(
        self,
        x=0.0,
        y=0.0,
        z=0.0,
    ):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(
        self,
        x=0.0,
        y=0.0,
        z=0.0,
        w=1.0,
    ):
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


# ============================================================
# Mock nav_msgs / geometry pose structures
# ============================================================


class _Header:
    def __init__(
        self,
        frame_id="",
    ):
        self.frame_id = frame_id
        self.stamp = _MockTime.now()


class _Pose:
    def __init__(
        self,
        x=0.0,
        y=0.0,
    ):
        self.position = _Vector3(
            x,
            y,
            0.0,
        )

        self.orientation = _Quaternion()


class _PoseStamped:
    def __init__(
        self,
        frame_id="world",
        x=0.0,
        y=0.0,
    ):
        self.header = _Header(frame_id)

        self.pose = _Pose(
            x,
            y,
        )


class _Path:
    def __init__(
        self,
        frame_id="world",
        poses=None,
    ):
        self.header = _Header(frame_id)

        self.poses = list(poses or [])


class _Odometry:
    def __init__(
        self,
        frame_id="world",
        child_frame_id="base",
        x=0.0,
        y=0.0,
    ):
        self.header = _Header(frame_id)

        self.child_frame_id = child_frame_id

        self.pose = type(
            "PoseWithCovariance",
            (),
            {},
        )()

        self.pose.pose = _Pose(
            x,
            y,
        )


class _NavMsgsMsg:
    Path = _Path
    Odometry = _Odometry


class _NavMsgs:
    msg = _NavMsgsMsg()


sys.modules["nav_msgs"] = _NavMsgs()
sys.modules["nav_msgs.msg"] = _NavMsgsMsg()


# ============================================================
# Mock competition_interfaces/ModuleStatus
# ============================================================


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


sys.modules["competition_interfaces"] = type(
    "CompetitionInterfaces",
    (),
    {},
)()

sys.modules["competition_interfaces.msg"] = type(
    "CompetitionInterfacesMsg",
    (),
    {
        "ModuleStatus": _ModuleStatus,
    },
)()


# ============================================================
# Load real path_follower_node
# ============================================================


def _load_node():
    path = Path(__file__).resolve().parents[1] / "scripts" / "path_follower_node"

    namespace = {
        "__name__": "test_module",
    }

    with path.open(encoding="utf-8") as source_file:
        source = source_file.read()

    exec(
        compile(
            source,
            str(path),
            "exec",
        ),
        namespace,
    )

    return namespace


NODE = _load_node()

PathFollower = NODE["PathFollower"]


# ============================================================
# Tests
# ============================================================


class TestPathFollower(unittest.TestCase):
    def setUp(self):
        # 每个测试从 ROS time = 10 s 开始。
        _MockTime.set(10.0)

        # 绕过 __init__，
        # 手动构造一个干净的 PathFollower。
        #
        # 这样各个测试之间不会共享 Subscriber / Publisher 状态。
        self.follower = PathFollower.__new__(PathFollower)

        # =========================
        # Path state
        # =========================

        self.follower.path = []
        self.follower.path_frame = ""
        self.follower.path_time = None
        self.follower.path_error = None

        # =========================
        # Odom state
        # =========================

        self.follower.odom = None
        self.follower.odom_time = None
        self.follower.odom_error = None

        # =========================
        # Planning status
        # =========================

        self.follower.planning_status = None
        self.follower.planning_status_time = None

        # =========================
        # Localization status
        # =========================

        self.follower.localization_status = None
        self.follower.localization_status_time = None

        # =========================
        # Path follower state
        # =========================

        self.follower.target_index = 0

        # =========================
        # Parameters
        # =========================

        self.follower.control_frequency = 10.0

        self.follower.max_linear = 0.25
        self.follower.max_angular = 0.8

        self.follower.goal_tolerance = 0.2

        self.follower.path_timeout = _MockDuration(5.0)

        self.follower.odom_timeout = _MockDuration(1.0)

        self.follower.planning_status_timeout = _MockDuration(1.0)

        self.follower.localization_status_timeout = _MockDuration(1.0)

        # =========================
        # Publishers
        # =========================

        self.follower.raw_vel_pub = _Publisher("/competition/raw_cmd_vel")

        self.follower.status_pub = _Publisher(
            "/competition/control/path_follower_status"
        )

    # ========================================================
    # Helper functions
    # ========================================================

    @staticmethod
    def _make_status(
        valid=True,
        state=None,
        detail="",
    ):
        msg = _ModuleStatus()

        msg.valid = bool(valid)

        if state is None:
            msg.state = _ModuleStatus.RUNNING if valid else _ModuleStatus.FAILED
        else:
            msg.state = state

        msg.confidence = 1.0 if valid else 0.0

        msg.detail = detail

        return msg

    def _set_planning_status(
        self,
        valid=True,
        state=None,
        detail="",
    ):
        self.follower.on_planning_status(
            self._make_status(
                valid,
                state,
                detail,
            )
        )

    def _set_localization_status(
        self,
        valid=True,
        state=None,
        detail="",
    ):
        self.follower.on_localization_status(
            self._make_status(
                valid,
                state,
                detail,
            )
        )

    def _set_valid_odom(
        self,
        frame="world",
        child_frame="base",
        x=0.0,
        y=0.0,
    ):
        msg = _Odometry(
            frame_id=frame,
            child_frame_id=child_frame,
            x=x,
            y=y,
        )

        self.follower.on_odom(msg)

        return msg

    def _set_valid_path(
        self,
        frame="world",
        pose_frame=None,
        points=None,
    ):
        if pose_frame is None:
            pose_frame = frame

        if points is None:
            points = [
                (1.0, 0.0),
            ]

        poses = [
            _PoseStamped(
                pose_frame,
                x,
                y,
            )
            for x, y in points
        ]

        msg = _Path(
            frame,
            poses,
        )

        self.follower.on_path(msg)

        return msg

    def _arm_nominal_inputs(
        self,
        points=None,
    ):
        """Provide a complete valid input set."""

        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom()

        self._set_valid_path(points=points)

    def _last_status(self):
        return self.follower.status_pub.messages[-1]

    def _last_cmd(self):
        return self.follower.raw_vel_pub.messages[-1]

    def _assert_zero_cmd(self):
        cmd = self._last_cmd()

        self.assertEqual(
            cmd.linear.x,
            0.0,
        )

        self.assertEqual(
            cmd.linear.y,
            0.0,
        )

        self.assertEqual(
            cmd.angular.z,
            0.0,
        )

    # ========================================================
    # Interface ownership
    # ========================================================

    def test_constructor_uses_only_official_topics(self):
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

        subscriber_topics = ROSPY.subscribers

        self.assertEqual(
            subscriber_topics,
            [
                "/competition/path",
                "/competition/planning/status",
                "/competition/localization/odom",
                "/competition/localization/status",
            ],
        )

        # PathFollower 永远不能直接拥有最终 /cmd_vel。
        self.assertNotIn(
            "/cmd_vel",
            publisher_topics,
        )

        # 公共 control/status 只能由 Guard 发布。
        self.assertNotIn(
            "/competition/control/status",
            publisher_topics,
        )

        # 正式算法不能偷看 Gazebo ground truth。
        self.assertNotIn(
            "/Odometry_gazebo",
            subscriber_topics,
        )

        for topic in subscriber_topics:
            self.assertNotIn(
                "ground_truth",
                topic,
            )

    # ========================================================
    # Normal behavior
    # ========================================================

    def test_normal_tracking_reports_running_and_nonzero(self):
        self._arm_nominal_inputs()

        self.follower.step()

        cmd = self._last_cmd()
        status = self._last_status()

        self.assertGreater(
            cmd.linear.x,
            0.0,
        )

        self.assertEqual(
            status.state,
            _ModuleStatus.RUNNING,
        )

        self.assertTrue(status.valid)

    def test_finished_path_reports_succeeded_and_zero(self):
        self._arm_nominal_inputs(
            points=[
                (0.05, 0.0),
            ]
        )

        self.follower.step()

        self._assert_zero_cmd()

        status = self._last_status()

        self.assertEqual(
            status.state,
            _ModuleStatus.SUCCEEDED,
        )

        self.assertTrue(status.valid)

    # ========================================================
    # Missing upstream inputs
    # ========================================================

    def test_missing_planning_status_stops(self):
        self._set_localization_status(valid=True)

        self._set_valid_odom()
        self._set_valid_path()

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.STARTING,
        )

        self.assertFalse(self._last_status().valid)

        self.assertIn(
            "planning/status",
            self._last_status().detail,
        )

    def test_missing_localization_status_stops(self):
        self._set_planning_status(valid=True)

        self._set_valid_odom()
        self._set_valid_path()

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.STARTING,
        )

        self.assertFalse(self._last_status().valid)

        self.assertIn(
            "localization/status",
            self._last_status().detail,
        )

    # ========================================================
    # Empty Path
    # ========================================================

    def test_empty_path_cancels_old_path_immediately(self):
        self._arm_nominal_inputs()

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        # Planning 发布空 Path。
        self.follower.on_path(
            _Path(
                "world",
                [],
            )
        )

        # 旧 Path 必须立刻消失。
        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertEqual(
            self.follower.target_index,
            0,
        )

        self._assert_zero_cmd()

        self.assertFalse(self._last_status().valid)

    # ========================================================
    # Planning invalid
    # ========================================================

    def test_planning_invalid_stops_immediately_and_clears_old_path(self):
        self._arm_nominal_inputs()

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        # Planning 突然失效。
        self._set_planning_status(
            valid=False,
            state=_ModuleStatus.FAILED,
            detail="planner failed",
        )

        # callback 本身就必须立刻停车。
        self._assert_zero_cmd()

        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertEqual(
            self.follower.target_index,
            0,
        )

        status = self._last_status()

        self.assertFalse(status.valid)

        self.assertEqual(
            status.state,
            _ModuleStatus.BLOCKED,
        )

    def test_planning_recovery_does_not_revive_old_path(self):
        self._arm_nominal_inputs()

        self.follower.step()

        # Planning invalid -> 旧 Path 作废。
        self._set_planning_status(valid=False)

        # Planning 恢复。
        self._set_planning_status(valid=True)

        # 没有新 Path。
        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertIn(
            "/competition/path",
            self._last_status().detail,
        )

    def test_new_path_received_while_planning_invalid_can_run_after_recovery(self):
        self._arm_nominal_inputs()

        # 旧 Path 作废。
        self._set_planning_status(valid=False)

        # 在 Planning 仍然 invalid 时，
        # 可以缓存当前 generation 的候选新 Path。
        self._set_valid_path(
            points=[
                (2.0, 0.0),
            ]
        )

        # 但 status 仍然 invalid，绝对不能运动。
        self.follower.step()

        self._assert_zero_cmd()

        # Planning 恢复。
        self._set_planning_status(valid=True)

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.RUNNING,
        )

    # ========================================================
    # Localization invalid
    # ========================================================

    def test_localization_invalid_stops_and_clears_old_odom_and_path(self):
        self._arm_nominal_inputs()

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        self._set_localization_status(
            valid=False,
            state=_ModuleStatus.FAILED,
            detail="localization lost",
        )

        self._assert_zero_cmd()

        # 失效前 Odom 不能继续用。
        self.assertIsNone(self.follower.odom)

        # 失效前 Path 也不能继续用。
        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertEqual(
            self.follower.target_index,
            0,
        )

        self.assertFalse(self._last_status().valid)

    def test_localization_recovery_plus_new_odom_does_not_revive_old_path(self):
        self._arm_nominal_inputs()

        self.follower.step()

        # Localization 失效，
        # 旧 Odom + 旧 Path 一起清除。
        self._set_localization_status(valid=False)

        # Localization 恢复。
        self._set_localization_status(valid=True)

        # 来了一份新 Odom。
        self._set_valid_odom(
            x=0.2,
            y=0.0,
        )

        # 但没有新 Path。
        self.follower.step()

        # 必须仍然停车。
        self._assert_zero_cmd()

        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertIn(
            "/competition/path",
            self._last_status().detail,
        )

    def test_localization_recovery_with_new_odom_and_new_path_runs_again(self):
        self._arm_nominal_inputs()

        self.follower.step()

        # Localization invalid。
        self._set_localization_status(valid=False)

        # 恢复。
        self._set_localization_status(valid=True)

        # 新 generation 的 Odom。
        self._set_valid_odom(
            x=0.2,
            y=0.0,
        )

        # 此时还不能运行。
        self.follower.step()

        self._assert_zero_cmd()

        # Planning 重新发布新 Path。
        self._set_valid_path(
            points=[
                (1.5, 0.0),
            ]
        )

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.RUNNING,
        )

        self.assertTrue(self._last_status().valid)

    def test_new_data_received_while_localization_invalid_runs_after_recovery(self):
        self._arm_nominal_inputs()

        # Localization invalid：
        # 先清除旧 generation。
        self._set_localization_status(valid=False)

        # status 仍 invalid，
        # 但允许缓存当前 generation 的候选数据。
        self._set_valid_odom(
            x=0.1,
            y=0.0,
        )

        self._set_valid_path(
            points=[
                (1.0, 0.0),
            ]
        )

        # status=false 时仍然绝对不能走。
        self.follower.step()

        self._assert_zero_cmd()

        # Localization 恢复。
        self._set_localization_status(valid=True)

        self.follower.step()

        self.assertGreater(
            self._last_cmd().linear.x,
            0.0,
        )

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.RUNNING,
        )

    # ========================================================
    # Status timeout
    # ========================================================

    def test_planning_status_timeout_stops_and_invalidates_path(self):
        self._arm_nominal_inputs()

        # 当前时间 10 s，
        # 假装上次 Planning status 是 8 s。
        self.follower.planning_status_time = _MockTimeInstance(8.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertEqual(
            self.follower.path,
            [],
        )

        self.assertIsNone(self.follower.planning_status)

    def test_localization_status_timeout_stops_and_invalidates_odom_and_path(self):
        self._arm_nominal_inputs()

        self.follower.localization_status_time = _MockTimeInstance(8.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertIsNone(self.follower.localization_status)

        self.assertIsNone(self.follower.odom)

        self.assertEqual(
            self.follower.path,
            [],
        )

    # ========================================================
    # Data timeout
    # ========================================================

    def test_path_timeout_stops_and_invalidates_path(self):
        self._arm_nominal_inputs()

        # path_timeout = 5 s
        # 10 - 4 = 6 s
        self.follower.path_time = _MockTimeInstance(4.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertEqual(
            self.follower.path,
            [],
        )

    def test_odom_timeout_stops_and_invalidates_odom_and_path(self):
        self._arm_nominal_inputs()

        # odom_timeout = 1 s
        # 10 - 8 = 2 s
        self.follower.odom_time = _MockTimeInstance(8.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertIsNone(self.follower.odom)

        self.assertEqual(
            self.follower.path,
            [],
        )

    # ========================================================
    # Frame validation
    # ========================================================

    def test_path_header_frame_must_be_world(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom()

        self._set_valid_path(
            frame="map",
            pose_frame="map",
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertFalse(self._last_status().valid)

        self.assertIn(
            "world",
            self._last_status().detail,
        )

    def test_every_path_pose_frame_must_be_world(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom()

        # Path header 正确，
        # 但其中 PoseStamped 错误。
        self._set_valid_path(
            frame="world",
            pose_frame="map",
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertIn(
            "world",
            self._last_status().detail,
        )

    def test_odom_frame_must_be_world(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom(
            frame="odom",
            child_frame="base",
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertIn(
            "world",
            self._last_status().detail,
        )

    def test_odom_child_frame_must_be_base(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom(
            frame="world",
            child_frame="base_link",
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertIn(
            "base",
            self._last_status().detail,
        )

    # ========================================================
    # Numeric validation
    # ========================================================

    def test_non_finite_path_is_rejected(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom()

        self._set_valid_path(
            points=[
                (math.nan, 0.0),
            ]
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertFalse(self._last_status().valid)

        self.assertIn(
            "NaN",
            self._last_status().detail,
        )

    def test_non_finite_odom_is_rejected(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        self._set_valid_odom(
            x=math.inf,
        )

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertIn(
            "NaN or Inf",
            self._last_status().detail,
        )

    def test_invalid_quaternion_is_rejected(self):
        self._set_planning_status(valid=True)

        self._set_localization_status(valid=True)

        odom = _Odometry(
            frame_id="world",
            child_frame_id="base",
        )

        odom.pose.pose.orientation = _Quaternion(
            0.0,
            0.0,
            0.0,
            0.0,
        )

        self.follower.on_odom(odom)

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.FAILED,
        )

        self.assertIn(
            "zero norm",
            self._last_status().detail,
        )

    # ========================================================
    # ROS time rollback
    # ========================================================

    def test_planning_status_time_rollback_invalidates_all_inputs(self):
        self._arm_nominal_inputs()

        # 当前 now = 10 s，
        # 却假装最后一次 status 来自未来 11 s。
        #
        # age = -1，代表 ROS 时间发生回退。
        self.follower.planning_status_time = _MockTimeInstance(11.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertIn(
            "backwards",
            self._last_status().detail,
        )

        # 旧 generation 所有数据都必须失效。
        self.assertIsNone(self.follower.planning_status)

        self.assertIsNone(self.follower.localization_status)

        self.assertIsNone(self.follower.odom)

        self.assertEqual(
            self.follower.path,
            [],
        )

    def test_odom_time_rollback_invalidates_all_inputs(self):
        self._arm_nominal_inputs()

        self.follower.odom_time = _MockTimeInstance(11.0)

        self.follower.step()

        self._assert_zero_cmd()

        self.assertEqual(
            self._last_status().state,
            _ModuleStatus.TIMEOUT,
        )

        self.assertIn(
            "backwards",
            self._last_status().detail,
        )

        self.assertIsNone(self.follower.odom)

        self.assertEqual(
            self.follower.path,
            [],
        )


if __name__ == "__main__":
    unittest.main()
