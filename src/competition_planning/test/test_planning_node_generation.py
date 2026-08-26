#!/usr/bin/env python3

"""Deterministic generation-boundary tests for planning_node.

These tests run without a ROS master: ``rospy`` and the ROS message
modules are replaced by a small controllable fake, so the clock, every
generation transition and every published message is fully
deterministic. The planning node itself is loaded from the scripts
directory next to this file.
"""

import importlib.machinery
import importlib.util
import json
import math
import sys
import tempfile
import types
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

_PLANNING_NODE_PATH = SCRIPTS_DIR / "planning_node"

_STATE = {
    "clock": 100.0,
    "params": {},
    "published": {},
    "timers": [],
    "subscribers": [],
}


def _reset_fake_ros(clock=100.0):
    _STATE["clock"] = float(clock)
    _STATE["params"].clear()
    _STATE["published"].clear()
    del _STATE["timers"][:]
    del _STATE["subscribers"][:]


def set_clock(value):
    _STATE["clock"] = float(value)


class FakeTime:
    def __init__(self, value=0.0):
        self.secs = value.secs if isinstance(value, FakeTime) else float(value)

    @staticmethod
    def now():
        return FakeTime(_STATE["clock"])

    def is_zero(self):
        return self.secs == 0.0

    def to_sec(self):
        return self.secs

    def __sub__(self, other):
        if isinstance(other, FakeTime):
            return FakeDuration(self.secs - other.secs)
        return NotImplemented

    def __eq__(self, other):
        return self.secs == other.secs

    def __ne__(self, other):
        return self.secs != other.secs

    def __lt__(self, other):
        return self.secs < other.secs

    def __le__(self, other):
        return self.secs <= other.secs

    def __gt__(self, other):
        return self.secs > other.secs

    def __ge__(self, other):
        return self.secs >= other.secs

    def __hash__(self):
        return hash(self.secs)

    def __repr__(self):
        return "FakeTime({:.3f})".format(self.secs)


class FakeDuration:
    def __init__(self, secs=0.0):
        self.secs = float(secs)

    def to_sec(self):
        return self.secs


class FakePublisher:
    def __init__(self, topic, msg_class=None, **kwargs):
        self.topic = topic
        _STATE["published"].setdefault(topic, [])

    def publish(self, msg):
        _STATE["published"].setdefault(self.topic, []).append(msg)


class FakeSubscriber:
    def __init__(self, topic, msg_class=None, callback=None, **kwargs):
        _STATE["subscribers"].append((topic, callback))


class FakeTimer:
    def __init__(self, period, callback=None, **kwargs):
        _STATE["timers"].append((period, callback))


def _fake_get_param(name, default=None):
    key = name[1:] if name.startswith("~") else name
    return _STATE["params"].get(key, default)


def _fake_init_node(*args, **kwargs):
    pass


def _fake_spin():
    pass


def _fake_log(*args, **kwargs):
    pass


_ROSPY = types.ModuleType("rospy")
_ROSPY.Time = FakeTime
_ROSPY.Duration = FakeDuration
_ROSPY.Publisher = FakePublisher
_ROSPY.Subscriber = FakeSubscriber
_ROSPY.Timer = FakeTimer
_ROSPY.get_param = _fake_get_param
_ROSPY.init_node = _fake_init_node
_ROSPY.spin = _fake_spin
_ROSPY.loginfo = _fake_log
_ROSPY.logwarn = _fake_log
_ROSPY.logerr = _fake_log
_ROSPY.loginfo_throttle = _fake_log
_ROSPY.logwarn_throttle = _fake_log
_ROSPY.logerr_throttle = _fake_log


class FakeHeader:
    def __init__(self):
        self.stamp = FakeTime(0.0)
        self.frame_id = ""


class FakePoint:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class FakeQuaternion:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class FakePose:
    def __init__(self):
        self.position = FakePoint()
        self.orientation = FakeQuaternion()


class FakePoseStamped:
    def __init__(self):
        self.header = FakeHeader()
        self.pose = FakePose()


class FakePath:
    def __init__(self):
        self.header = FakeHeader()
        self.poses = []


class FakeMapMetaData:
    def __init__(self):
        self.map_load_time = FakeTime(0.0)
        self.resolution = 1.0
        self.width = 0
        self.height = 0
        self.origin = FakePose()


class FakeOccupancyGrid:
    def __init__(self):
        self.header = FakeHeader()
        self.info = FakeMapMetaData()
        self.data = []


class FakeOdometry:
    def __init__(self):
        self.header = FakeHeader()
        self.child_frame_id = ""
        self.pose = FakePoseStamped()


class FakeModuleStatus:
    STARTING = 1
    RUNNING = 2
    SUCCEEDED = 3
    BLOCKED = 4
    TIMEOUT = 5
    FAILED = 6

    def __init__(self):
        self.header = FakeHeader()
        self.state = self.STARTING
        self.valid = False
        self.confidence = 0.0
        self.detail = ""


class FakeCompetitionState:
    WAITING = 0
    RUNNING = 1
    RETURNING = 2
    FINISHED = 3
    ERROR = 4

    def __init__(self):
        self.header = FakeHeader()
        self.state = self.WAITING
        self.detail = ""


def _make_package(name, attributes):
    package = types.ModuleType(name)
    package.__path__ = []
    for attr, value in attributes.items():
        setattr(package, attr, value)
    return package


_CI_MSG = _make_package(
    "competition_interfaces.msg",
    {
        "ModuleStatus": FakeModuleStatus,
        "CompetitionState": FakeCompetitionState,
    },
)
_CI = _make_package("competition_interfaces", {"msg": _CI_MSG})

_NAV_MSG = _make_package(
    "nav_msgs.msg",
    {
        "OccupancyGrid": FakeOccupancyGrid,
        "Odometry": FakeOdometry,
        "Path": FakePath,
    },
)
_NAV = _make_package("nav_msgs", {"msg": _NAV_MSG})

_GEO_MSG = _make_package("geometry_msgs.msg", {"PoseStamped": FakePoseStamped})
_GEO = _make_package("geometry_msgs", {"msg": _GEO_MSG})


class FakeBool:
    def __init__(self, data=False):
        self.data = data


_STD_MSG = _make_package("std_msgs.msg", {"Bool": FakeBool})
_STD = _make_package("std_msgs", {"msg": _STD_MSG})


def _install_fake_modules():
    # Deliberately shadow any real ROS python modules so this test file
    # stays deterministic with or without a ROS installation.
    sys.modules["rospy"] = _ROSPY
    sys.modules["competition_interfaces"] = _CI
    sys.modules["competition_interfaces.msg"] = _CI_MSG
    sys.modules["nav_msgs"] = _NAV
    sys.modules["nav_msgs.msg"] = _NAV_MSG
    sys.modules["geometry_msgs"] = _GEO
    sys.modules["geometry_msgs.msg"] = _GEO_MSG
    sys.modules["std_msgs"] = _STD
    sys.modules["std_msgs.msg"] = _STD_MSG


_install_fake_modules()

# planning_node is an extensionless Python script, so the source-file
# loader must be given explicitly.
_PLANNING_NODE_LOADER = importlib.machinery.SourceFileLoader(
    "planning_node_fake_ros_test",
    str(_PLANNING_NODE_PATH),
)
_PLANNING_NODE_SPEC = importlib.util.spec_from_file_location(
    "planning_node_fake_ros_test",
    str(_PLANNING_NODE_PATH),
    loader=_PLANNING_NODE_LOADER,
)
PLANNING_NODE = importlib.util.module_from_spec(_PLANNING_NODE_SPEC)
_PLANNING_NODE_SPEC.loader.exec_module(PLANNING_NODE)

# Ensure the pure-python helpers imported by the node come from the
# same scripts directory this test file ships with.
assert (
    str(Path(PLANNING_NODE.__file__).resolve()) == str(_PLANNING_NODE_PATH)
)


class PlanningNodeGenerationBoundaryTest(unittest.TestCase):
    def setUp(self):
        _reset_fake_ros(clock=100.0)
        _STATE["params"].update(
            {
                "map_timeout": 1.0,
                "localization_timeout": 2.0,
                "mapping_status_timeout": 3.0,
                "localization_status_timeout": 3.0,
                "future_stamp_tolerance": 0.2,
                "status_heartbeat_interval": 0.1,
                "replan_interval": 2.0,
            }
        )
        self.node = PLANNING_NODE.PlanningNode()

    @staticmethod
    def _make_map(stamp, frame_id="world"):
        msg = FakeOccupancyGrid()
        msg.header.stamp = FakeTime(stamp)
        msg.header.frame_id = frame_id
        msg.info.resolution = 1.0
        msg.info.width = 5
        msg.info.height = 5
        msg.data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]
        return msg

    @staticmethod
    def _make_odom(stamp, frame_id="world", child_frame_id="base"):
        msg = FakeOdometry()
        msg.header.stamp = FakeTime(stamp)
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.pose.pose.position.x = 2.5
        msg.pose.pose.position.y = 2.5
        return msg

    @staticmethod
    def _make_fully_known_map(stamp):
        msg = FakeOccupancyGrid()
        msg.header.stamp = FakeTime(stamp)
        msg.header.frame_id = "world"
        msg.info.resolution = 1.0
        msg.info.width = 5
        msg.info.height = 5
        msg.data = [0] * 25
        return msg

    @staticmethod
    def _set_odom_yaw(msg, yaw):
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return msg

    @staticmethod
    def _make_status(valid, stamp, detail="test"):
        msg = FakeModuleStatus()
        msg.header.stamp = FakeTime(stamp)
        msg.state = (
            FakeModuleStatus.RUNNING
            if valid
            else FakeModuleStatus.BLOCKED
        )
        msg.valid = valid
        msg.confidence = 1.0 if valid else 0.0
        msg.detail = detail
        return msg

    def _published(self, topic):
        return _STATE["published"].get(topic, [])

    def _last_path(self):
        paths = self._published("/competition/path")
        return paths[-1] if paths else None

    def _last_status_msg(self):
        statuses = self._published("/competition/planning/status")
        return statuses[-1] if statuses else None

    def _publish_healthy_generation(self, base_time):
        set_clock(base_time)
        self.node.mapping_status_callback(
            self._make_status(True, base_time)
        )
        self.node.localization_status_callback(
            self._make_status(True, base_time)
        )
        self.node.map_callback(self._make_map(base_time + 0.1))
        self.node.odom_callback(self._make_odom(base_time + 0.1))
        set_clock(base_time + 0.2)
        self.node.plan_callback(None)

    def test_start_recovery_parameter_defaults_disabled(self):
        self.assertEqual(self.node.start_recovery_radius_cells, 0)

    def test_new_reached_target_starts_stationary_visual_scan(self):
        self.node.active_exploration_target = (2.5, 2.5)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)

        self.assertTrue(self.node._handle_reached_exploration_target(odom))
        self.assertTrue(self.node.visual_coverage.active)
        path = self._last_path()
        self.assertEqual(len(path.poses), 2)
        self.assertAlmostEqual(path.poses[0].pose.position.x, 2.5)
        self.assertAlmostEqual(path.poses[1].pose.position.x, 2.5)
        self.assertAlmostEqual(path.poses[0].pose.position.y, 2.5)
        self.assertAlmostEqual(path.poses[1].pose.position.y, 2.5)
        self.assertEqual(self._last_status_msg().state, FakeModuleStatus.RUNNING)
        self.assertTrue(self._last_status_msg().valid)

    def test_planner_target_reached_via_callback_starts_scan(self):
        """The real plan callback, rather than a helper call, owns the trigger."""
        self._publish_healthy_generation(100.0)
        initial_path = self._last_path()
        self.assertIsNotNone(initial_path)
        self.assertGreater(len(initial_path.poses), 0)

        goal = initial_path.poses[-1].pose.position
        odom = self._make_odom(100.3)
        odom.pose.pose.position.x = goal.x
        odom.pose.pose.position.y = goal.y
        set_clock(100.4)
        self.node.odom_callback(odom)
        self.node.plan_callback(None)

        self.assertTrue(self.node.visual_coverage.active)
        scan_path = self._last_path()
        self.assertEqual(len(scan_path.poses), 2)
        self.assertAlmostEqual(
            scan_path.poses[0].pose.position.x,
            scan_path.poses[1].pose.position.x,
        )
        self.assertAlmostEqual(
            scan_path.poses[0].pose.position.y,
            scan_path.poses[1].pose.position.y,
        )


    def test_target_not_reached_does_not_start_scan(self):
        self.node.active_exploration_target = (10.0, 10.0)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)

        self.assertFalse(self.node._handle_reached_exploration_target(odom))
        self.assertFalse(self.node.visual_coverage.active)
        self.assertEqual(self.node.active_exploration_target, (10.0, 10.0))

    def test_nearby_scanned_target_is_deduplicated(self):
        self.node.visual_coverage.scanned_positions = [(2.5, 2.5)]
        self.node.active_exploration_target = (2.6, 2.5)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)

        self.assertFalse(self.node._handle_reached_exploration_target(odom))
        self.assertFalse(self.node.visual_coverage.active)
        self.assertIsNone(self.node.active_exploration_target)

    def test_visual_scan_disabled_preserves_normal_planner_path(self):
        self.node.visual_coverage.enabled = False
        self.node.active_exploration_target = (2.5, 2.5)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)

        self.assertFalse(self.node._handle_reached_exploration_target(odom))
        self.assertFalse(self.node.visual_coverage.active)
        self.assertIsNone(self.node.active_exploration_target)

    def test_no_frontier_waits_for_final_scan_before_completion(self):
        self.node.visual_coverage = PLANNING_NODE.VisualCoverageTracker(
            enabled=True,
            yaw_step=math.pi / 2.0,
            yaw_tolerance=0.1,
            dwell=0.0,
            heading_timeout=2.0,
            scan_timeout=20.0,
            position_dedup_radius=1.0,
            max_heading_retries=0,
        )
        self.node.visual_scan_dwell = 0.0
        set_clock(100.0)
        self.node.mapping_status_callback(self._make_status(True, 100.0))
        self.node.localization_status_callback(
            self._make_status(True, 100.0)
        )
        self.node.map_callback(self._make_fully_known_map(100.1))
        self.node.odom_callback(self._make_odom(100.1))
        set_clock(100.2)
        self.node.plan_callback(None)

        self.assertTrue(self.node.visual_coverage.active)
        self.assertFalse(self._published(
            "/competition/planning/exploration_complete"
        )[-1].data)
        self.assertTrue(self._last_status_msg().valid)
        self.assertIn("visual scan heading", self._last_status_msg().detail)

        now = 100.3
        while self.node.visual_coverage.active:
            target_yaw = self.node.visual_coverage.current_heading
            set_clock(now)
            odom = self._set_odom_yaw(self._make_odom(now), target_yaw)
            self.node.odom_callback(odom)
            self.node.visual_scan_callback(None)
            now += 0.01

            if self.node.visual_coverage.active:
                set_clock(now)
                odom = self._set_odom_yaw(
                    self._make_odom(now),
                    self.node.visual_coverage.current_heading,
                )
                self.node.odom_callback(odom)
                self.node.visual_scan_callback(None)
                now += 0.01

        self.assertEqual(
            len(self.node.visual_coverage.scanned_positions),
            1,
        )
        self.assertFalse(self.node.visual_scan_final_pending)

        # Completion debounce starts only after the final scan ended.
        for _ in range(3):
            now += 0.05
            set_clock(now)
            self.node.plan_callback(None)
        self.assertTrue(self.node._exploration_complete_active)
        self.assertTrue(self._published(
            "/competition/planning/exploration_complete"
        )[-1].data)
        true_messages = [
            message.data
            for message in self._published(
                "/competition/planning/exploration_complete"
            )
            if message.data
        ]
        self.assertEqual(len(true_messages), 1)

    def test_generation_reset_clears_visual_coverage(self):
        self.node.active_exploration_target = (2.5, 2.5)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)
        self.assertTrue(self.node._handle_reached_exploration_target(odom))
        self.node.visual_coverage.scanned_positions.append((4.0, 4.0))

        self.node._enter_new_generation("test_reset", FakeTime(101.0))
        self.assertFalse(self.node.visual_coverage.active)
        self.assertEqual(self.node.visual_coverage.scanned_positions, [])
        self.assertIsNone(self.node.active_exploration_target)
        self.assertFalse(self.node.visual_scan_final_pending)

    def test_planning_status_stays_running_during_scan_progress(self):
        self.node.active_exploration_target = (2.5, 2.5)
        odom = self._make_odom(100.1)
        self.node.odom_msg = odom
        set_clock(100.2)
        self.node._handle_reached_exploration_target(odom)
        self.node.visual_scan_callback(None)

        status = self._last_status_msg()
        self.assertEqual(status.state, FakeModuleStatus.RUNNING)
        self.assertTrue(status.valid)

    def test_returning_state_loads_public_start_and_uses_goal_planner(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            scene_path = Path(temp_dir) / "team_scene_info.json"
            scene_path.write_text(
                json.dumps(
                    {
                        "coordinate_frame": "world",
                        "robot_start": {
                            "x": 2.5,
                            "y": 2.5,
                            "z": 0.0,
                            "yaw": 0.0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            _reset_fake_ros(clock=100.0)
            _STATE["params"]["team_scene_info_path"] = str(scene_path)
            node = PLANNING_NODE.PlanningNode()
            self.node = node
            state = FakeCompetitionState()
            state.state = FakeCompetitionState.RETURNING
            node.competition_state_callback(state)
            self.assertTrue(node.returning)
            self.assertEqual(node.return_start_pose["x"], 2.5)

            calls = []
            original = PLANNING_NODE.plan_to_goal_region_with_status

            def capture_goal(*args, **kwargs):
                calls.append(kwargs)
                return {
                    "status": PLANNING_NODE.PLAN_SUCCESS,
                    "goal_cell": (2, 2),
                    "path": [(2, 2)],
                    "frontier_count": 0,
                    "start_recovery_used": False,
                    "start_recovery_cell": None,
                    "robot_cell_raw_value": 0,
                    "robot_cell_inflated_value": 0,
                }

            PLANNING_NODE.plan_to_goal_region_with_status = capture_goal
            try:
                self._publish_healthy_generation(100.0)
            finally:
                PLANNING_NODE.plan_to_goal_region_with_status = original
            self.assertEqual(len(calls), 1)
            self.assertEqual(calls[0]["goal_cell"], (2, 2))
            self.assertEqual(calls[0]["goal_search_radius_cells"], 2)

    def test_return_path_commit_republishes_planner_geometry_without_replanning(self):
        """A committed route survives map churn only while inputs stay valid."""
        with tempfile.TemporaryDirectory() as temp_dir:
            scene_path = Path(temp_dir) / "team_scene_info.json"
            scene_path.write_text(
                json.dumps(
                    {
                        "coordinate_frame": "world",
                        "robot_start": {
                            "x": 1.5,
                            "y": 1.5,
                            "z": 0.0,
                            "yaw": 0.0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            _reset_fake_ros(clock=100.0)
            _STATE["params"].update(
                {
                    "team_scene_info_path": str(scene_path),
                    "return_path_commit_enabled": True,
                    "map_timeout": 1.0,
                    "localization_timeout": 2.0,
                    "mapping_status_timeout": 3.0,
                    "localization_status_timeout": 3.0,
                    "future_stamp_tolerance": 0.2,
                    "status_heartbeat_interval": 0.1,
                }
            )
            self.node = PLANNING_NODE.PlanningNode()
            state = FakeCompetitionState()
            state.state = FakeCompetitionState.RETURNING
            self.node.competition_state_callback(state)
            self._publish_healthy_generation(100.0)

            initial_path = self._last_path()
            self.assertIsNotNone(initial_path)
            self.assertIsNotNone(self.node.return_committed_path)
            self.assertIn("RETURNING path", self._last_status_msg().detail)

            original = PLANNING_NODE.plan_to_goal_region_with_status

            def should_not_replan(*args, **kwargs):
                raise AssertionError("committed return route was replanned")

            PLANNING_NODE.plan_to_goal_region_with_status = should_not_replan
            try:
                self._publish_healthy_generation(101.0)
            finally:
                PLANNING_NODE.plan_to_goal_region_with_status = original

            committed_path = self._last_path()
            self.assertEqual(
                len(committed_path.poses), len(initial_path.poses)
            )
            self.assertIn(
                "RETURNING committed planner path",
                self._last_status_msg().detail,
            )

            # A committed route is not a way around upstream safety gates.
            # Invalid localization must still block the follower path.
            set_clock(102.0)
            self.node.localization_status_callback(
                self._make_status(False, 102.0)
            )
            self.node.plan_callback(None)
            self.assertEqual(len(self._last_path().poses), 0)
            self.assertFalse(self._last_status_msg().valid)

    def test_stuck_report_drops_commit_once_then_recommits(self):
        """A stuck notice clears the committed route once; the next healthy
        generation re-commits and a repeat stuck report is ignored (budget)."""
        with tempfile.TemporaryDirectory() as temp_dir:
            scene_path = Path(temp_dir) / "team_scene_info.json"
            scene_path.write_text(
                json.dumps(
                    {
                        "coordinate_frame": "world",
                        "robot_start": {
                            "x": 1.5,
                            "y": 1.5,
                            "z": 0.0,
                            "yaw": 0.0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            _reset_fake_ros(clock=100.0)
            _STATE["params"].update(
                {
                    "team_scene_info_path": str(scene_path),
                    "return_path_commit_enabled": True,
                    "return_recommit_on_stuck_enabled": True,
                    "map_timeout": 1.0,
                    "localization_timeout": 2.0,
                    "mapping_status_timeout": 3.0,
                    "localization_status_timeout": 3.0,
                    "future_stamp_tolerance": 0.2,
                    "status_heartbeat_interval": 0.1,
                }
            )
            self.node = PLANNING_NODE.PlanningNode()
            state = FakeCompetitionState()
            state.state = FakeCompetitionState.RETURNING
            self.node.competition_state_callback(state)
            self._publish_healthy_generation(100.0)
            self.assertIsNotNone(self.node.return_committed_path)

            # Stuck True -> committed route cleared, plan invalidated.
            stuck = FakeBool(data=True)
            self.node.stuck_callback(stuck)
            self.assertIsNone(self.node.return_committed_path)
            self.assertFalse(self._last_status_msg().valid)

            # Healthy generation replans and re-commits (full A* ran).
            self._publish_healthy_generation(101.0)
            self.assertIsNotNone(self.node.return_committed_path)
            # A following generation republishes the committed route.
            self._publish_healthy_generation(102.0)
            self.assertIn(
                "RETURNING committed planner path",
                self._last_status_msg().detail,
            )

            # Repeat stuck in the same epoch is ignored (budget used).
            self.node.stuck_callback(stuck)
            self.assertIsNotNone(self.node.return_committed_path)

    def test_stuck_report_ignored_when_disabled(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            scene_path = Path(temp_dir) / "team_scene_info.json"
            scene_path.write_text(
                json.dumps(
                    {
                        "coordinate_frame": "world",
                        "robot_start": {
                            "x": 1.5,
                            "y": 1.5,
                            "z": 0.0,
                            "yaw": 0.0,
                        },
                    }
                ),
                encoding="utf-8",
            )
            _reset_fake_ros(clock=100.0)
            _STATE["params"].update(
                {
                    "team_scene_info_path": str(scene_path),
                    "return_path_commit_enabled": True,
                    "return_recommit_on_stuck_enabled": False,
                    "map_timeout": 1.0,
                    "localization_timeout": 2.0,
                    "mapping_status_timeout": 3.0,
                    "localization_status_timeout": 3.0,
                    "future_stamp_tolerance": 0.2,
                    "status_heartbeat_interval": 0.1,
                }
            )
            self.node = PLANNING_NODE.PlanningNode()
            state = FakeCompetitionState()
            state.state = FakeCompetitionState.RETURNING
            self.node.competition_state_callback(state)
            self._publish_healthy_generation(100.0)
            self.assertIsNotNone(self.node.return_committed_path)
            self.node.stuck_callback(FakeBool(data=True))
            self.assertIsNotNone(self.node.return_committed_path)

    def test_exploration_blocked_nudge_publishes_short_blind_path(self):
        """PLAN_UNREACHABLE in exploration publishes a bounded nudge path."""
        _reset_fake_ros(clock=100.0)
        _STATE["params"].update(
            {
                "exploration_blocked_nudge_enabled": True,
                "exploration_blocked_nudge_distance": 2.0,
                "map_timeout": 1.0,
                "localization_timeout": 2.0,
                "mapping_status_timeout": 3.0,
                "localization_status_timeout": 3.0,
                "future_stamp_tolerance": 0.2,
                "status_heartbeat_interval": 0.1,
            }
        )
        self.node = PLANNING_NODE.PlanningNode()

        # Robot (2,2) in a small free pocket fully enclosed by known occupied
        # cells (no reachable frontier).  A separate free region (bottom-right)
        # has an unknown frontier boundary but is unreachable -> PLAN_UNREACHABLE.
        enclosed = FakeOccupancyGrid()
        enclosed.header.stamp = FakeTime(100.1)
        enclosed.header.frame_id = "world"
        enclosed.info.resolution = 1.0
        enclosed.info.width = 9
        enclosed.info.height = 9
        enclosed.data = [
            100,100,100,100,100,100,100,100,100,
            100,  0,  0,100,100,100,100,100,100,
            100,  0,  0,100,100,100,100,100,100,
            100,100,100,100,100,100,100,100,100,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
             -1, -1, -1, -1, -1, -1, -1, -1, -1,
        ]
        set_clock(100.0)
        self.node.mapping_status_callback(self._make_status(True, 100.0))
        self.node.localization_status_callback(
            self._make_status(True, 100.0)
        )
        self.node.map_callback(enclosed)
        self.node.odom_callback(self._make_odom(100.1))
        set_clock(100.2)
        self.node.plan_callback(None)

        nudge_path = self._last_path()
        self.assertIsNotNone(nudge_path)
        self.assertGreaterEqual(len(nudge_path.poses), 2)
        detail = self._last_status_msg().detail
        self.assertIn("nudging toward nearest frontier", detail)

    def test_exploration_blocked_nudge_disabled_falls_back_to_blocked(self):
        _reset_fake_ros(clock=100.0)
        _STATE["params"].update(
            {
                "exploration_blocked_nudge_enabled": False,
                "map_timeout": 1.0,
                "localization_timeout": 2.0,
                "mapping_status_timeout": 3.0,
                "localization_status_timeout": 3.0,
                "future_stamp_tolerance": 0.2,
                "status_heartbeat_interval": 0.1,
            }
        )
        self.node = PLANNING_NODE.PlanningNode()
        enclosed = FakeOccupancyGrid()
        enclosed.header.stamp = FakeTime(100.1)
        enclosed.header.frame_id = "world"
        enclosed.info.resolution = 1.0
        enclosed.info.width = 9
        enclosed.info.height = 9
        enclosed.data = [
            100,100,100,100,100,100,100,100,100,
            100,  0,  0,100,100,100,100,100,100,
            100,  0,  0,100,100,100,100,100,100,
            100,100,100,100,100,100,100,100,100,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
            100,100,100,  0,  0,  0,  0,  0, -1,
             -1, -1, -1, -1, -1, -1, -1, -1, -1,
        ]
        set_clock(100.0)
        self.node.mapping_status_callback(self._make_status(True, 100.0))
        self.node.localization_status_callback(
            self._make_status(True, 100.0)
        )
        self.node.map_callback(enclosed)
        self.node.odom_callback(self._make_odom(100.1))
        set_clock(100.2)
        self.node.plan_callback(None)
        self.assertIn(
            "none is currently reachable", self._last_status_msg().detail
        )
        self.assertFalse(self._last_status_msg().valid)

    def test_start_recovery_parameter_is_forwarded_to_core(self):
        _reset_fake_ros(clock=100.0)
        _STATE["params"]["start_recovery_radius_cells"] = 1
        self.node = PLANNING_NODE.PlanningNode()
        calls = []
        original = PLANNING_NODE.plan_to_frontier_with_status

        def capture_start_recovery_radius(*args, **kwargs):
            calls.append(kwargs)
            return original(*args, **kwargs)

        PLANNING_NODE.plan_to_frontier_with_status = (
            capture_start_recovery_radius
        )
        try:
            self._publish_healthy_generation(100.0)
        finally:
            PLANNING_NODE.plan_to_frontier_with_status = original

        self.assertEqual(self.node.start_recovery_radius_cells, 1)
        self.assertEqual(len(calls), 1)
        self.assertEqual(calls[0]["start_recovery_radius_cells"], 1)

    def test_negative_start_recovery_parameter_is_rejected(self):
        _reset_fake_ros(clock=100.0)
        _STATE["params"]["start_recovery_radius_cells"] = -1
        with self.assertRaisesRegex(
            ValueError,
            "start_recovery_radius_cells must not be negative",
        ):
            PLANNING_NODE.PlanningNode()

    def test_delayed_pre_generation_map_is_dropped(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # Mapping invalid opens generation 1 with boundary 101.0.
        set_clock(101.0)
        node.mapping_status_callback(self._make_status(False, 101.0))
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )
        self.assertFalse(node.last_status[1])

        # A repeated invalid heartbeat must not advance the generation
        # or move the boundary again.
        set_clock(101.1)
        node.mapping_status_callback(self._make_status(False, 101.1))
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )

        # Delayed old Map: stamp <= boundary but well inside map_timeout.
        # It must be dropped without touching the generation.
        set_clock(101.2)
        node.map_callback(self._make_map(100.5))
        self.assertIsNone(node.map_msg)
        self.assertEqual(node.generation, 1)

        # Fresh generation-1 Odometry and valid statuses alone must not
        # recover: the current-generation Map is still missing.
        set_clock(101.3)
        node.odom_callback(self._make_odom(101.3))
        node.mapping_status_callback(self._make_status(True, 101.2))
        node.localization_status_callback(
            self._make_status(True, 101.2)
        )
        set_clock(101.4)
        node.plan_callback(None)
        self.assertFalse(node.last_status[1])
        self.assertEqual(node.generation, 1)

        # Only a fresh Map after the boundary enables recovery.
        set_clock(101.5)
        node.map_callback(self._make_map(101.5))
        set_clock(101.6)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 1)
        self.assertGreater(len(self._last_path().poses), 0)

    def test_delayed_pre_generation_odometry_is_dropped(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # Localization invalid opens generation 1 with boundary 101.0.
        set_clock(101.0)
        node.localization_status_callback(
            self._make_status(False, 101.0)
        )
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )
        self.assertFalse(node.last_status[1])

        # Delayed old Odometry: stamp <= boundary, dropped silently.
        set_clock(101.2)
        node.odom_callback(self._make_odom(100.5))
        self.assertIsNone(node.odom_msg)
        self.assertEqual(node.generation, 1)

        # Fresh generation-1 Map and valid statuses alone must not
        # recover: the current-generation Odometry is still missing.
        set_clock(101.3)
        node.map_callback(self._make_map(101.3))
        node.mapping_status_callback(self._make_status(True, 101.2))
        node.localization_status_callback(
            self._make_status(True, 101.2)
        )
        set_clock(101.4)
        node.plan_callback(None)
        self.assertFalse(node.last_status[1])
        self.assertEqual(node.generation, 1)

        # Only a fresh Odometry after the boundary enables recovery.
        set_clock(101.5)
        node.odom_callback(self._make_odom(101.5))
        set_clock(101.6)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 1)
        self.assertGreater(len(self._last_path().poses), 0)

    def test_stamp_equal_to_generation_boundary_is_rejected(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])

        set_clock(101.0)
        node.mapping_status_callback(self._make_status(False, 101.0))
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )

        # stamp == boundary exactly: rejected for both Map and Odometry.
        set_clock(101.3)
        node.map_callback(self._make_map(101.0))
        self.assertIsNone(node.map_msg)
        node.odom_callback(self._make_odom(101.0))
        self.assertIsNone(node.odom_msg)
        self.assertEqual(node.generation, 1)

        # A stamp just after the boundary is admitted.
        node.map_callback(self._make_map(101.0001))
        self.assertIsNotNone(node.map_msg)
        self.assertEqual(node.map_generation, 1)
        node.odom_callback(self._make_odom(101.0001))
        self.assertIsNotNone(node.odom_msg)
        self.assertEqual(node.odom_generation, 1)

    def test_repeated_stale_timestamp_opens_new_generations(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # First stale Map: admitted by the boundary (stamp > time 0)
        # but older than map_timeout, so the watchdog raises the
        # generic timestamp fault and opens generation 1.
        set_clock(102.0)
        node.map_callback(self._make_map(100.8))
        node.heartbeat_callback(None)
        self.assertEqual(node.generation, 1)
        self.assertIn(
            "message_timestamp_invalid",
            node.active_faults,
        )
        self.assertFalse(node.last_status[1])

        # Complete recovery with all-fresh inputs clears the fault.
        set_clock(102.5)
        node.mapping_status_callback(self._make_status(True, 102.5))
        node.localization_status_callback(
            self._make_status(True, 102.5)
        )
        node.map_callback(self._make_map(102.6))
        node.odom_callback(self._make_odom(102.6))
        set_clock(102.7)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertNotIn(
            "message_timestamp_invalid",
            node.active_faults,
        )
        self.assertEqual(node.generation, 1)

        # A second independent stale message must open generation 2
        # instead of being absorbed by a leftover active fault.
        set_clock(104.0)
        node.map_callback(self._make_map(102.9))
        node.heartbeat_callback(None)
        self.assertEqual(node.generation, 2)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            104.0,
        )
        self.assertFalse(node.last_status[1])

    def test_stale_no_frontier_result_is_discarded(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        original = PLANNING_NODE.plan_to_frontier_with_status

        def stale_result(*args, **kwargs):
            # Generation 2 opens while the old computation is running.
            set_clock(102.0)
            node.mapping_status_callback(
                self._make_status(False, 102.0)
            )
            return {
                "status": PLANNING_NODE.PLAN_NO_FRONTIER,
                "goal_cell": None,
                "path": None,
                "cluster": None,
                "frontier_count": 0,
            }

        PLANNING_NODE.plan_to_frontier_with_status = stale_result
        try:
            set_clock(101.0)
            node.plan_callback(None)
        finally:
            PLANNING_NODE.plan_to_frontier_with_status = original

        # The stale SUCCEEDED must not be published in generation 1.
        self.assertEqual(node.generation, 1)
        self.assertFalse(node.last_status[1])
        for msg in self._published("/competition/planning/status"):
            self.assertNotEqual(
                msg.state,
                PLANNING_NODE.ModuleStatus.SUCCEEDED,
            )

    def test_stale_unreachable_result_is_discarded(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        original = PLANNING_NODE.plan_to_frontier_with_status

        def stale_result(*args, **kwargs):
            # Generation 2 opens while the old computation is running.
            set_clock(102.0)
            node.localization_status_callback(
                self._make_status(False, 102.0)
            )
            return {
                "status": PLANNING_NODE.PLAN_UNREACHABLE,
                "goal_cell": None,
                "path": None,
                "cluster": None,
                "frontier_count": 3,
            }

        PLANNING_NODE.plan_to_frontier_with_status = stale_result
        try:
            set_clock(101.0)
            node.plan_callback(None)
        finally:
            PLANNING_NODE.plan_to_frontier_with_status = original

        # The stale BLOCKED must not overwrite the new generation's
        # invalidation status.
        self.assertEqual(node.generation, 1)
        self.assertFalse(node.last_status[1])
        last = self._last_status_msg()
        self.assertIn("localization status is invalid", last.detail)
        for msg in self._published("/competition/planning/status"):
            self.assertNotIn(
                "none is currently reachable",
                msg.detail,
            )

    def test_delayed_old_mapping_valid_status_is_dropped(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # Mapping invalid opens generation 1 (boundary 101.0).
        set_clock(101.0)
        node.mapping_status_callback(self._make_status(False, 101.0))
        self.assertEqual(node.generation, 1)
        self.assertIn("mapping_invalid", node.active_faults)
        self.assertFalse(node.last_status[1])
        self.assertAlmostEqual(
            node.last_mapping_status_stamp.to_sec(),
            101.0,
        )

        # A delayed old valid=true mapping status (stamp before the last
        # accepted invalid one) must be dropped: it must not overwrite
        # the stored status, clear the fault, or open a generation.
        set_clock(101.2)
        node.mapping_status_callback(self._make_status(True, 100.5))
        self.assertFalse(node.mapping_status.valid)
        self.assertIn("mapping_invalid", node.active_faults)
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.last_mapping_status_stamp.to_sec(),
            101.0,
        )

        # Even with fresh current-generation Map and Odometry the stale
        # valid status must not enable recovery.
        set_clock(101.3)
        node.map_callback(self._make_map(101.3))
        node.odom_callback(self._make_odom(101.3))
        set_clock(101.4)
        node.plan_callback(None)
        self.assertFalse(node.last_status[1])
        self.assertEqual(node.generation, 1)

        # A subsequent real invalid heartbeat must not force an extra
        # generation: previous_valid stayed False because the delayed
        # old valid status was dropped.
        set_clock(101.6)
        node.mapping_status_callback(self._make_status(False, 101.6))
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )
        self.assertFalse(node.last_status[1])

        # A fresh valid mapping status (strictly after the last accepted
        # one) enables recovery within the same generation.
        set_clock(101.7)
        node.mapping_status_callback(self._make_status(True, 101.7))
        self.assertNotIn("mapping_invalid", node.active_faults)
        set_clock(101.8)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 1)
        self.assertGreater(len(self._last_path().poses), 0)

    def test_delayed_old_localization_valid_status_is_dropped(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # Localization invalid opens generation 1 (boundary 101.0).
        set_clock(101.0)
        node.localization_status_callback(
            self._make_status(False, 101.0)
        )
        self.assertEqual(node.generation, 1)
        self.assertIn("localization_invalid", node.active_faults)
        self.assertFalse(node.last_status[1])
        self.assertAlmostEqual(
            node.last_localization_status_stamp.to_sec(),
            101.0,
        )

        # A delayed old valid=true localization status is dropped without
        # touching any state.
        set_clock(101.2)
        node.localization_status_callback(
            self._make_status(True, 100.5)
        )
        self.assertFalse(node.localization_status.valid)
        self.assertIn("localization_invalid", node.active_faults)
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.last_localization_status_stamp.to_sec(),
            101.0,
        )

        # Fresh current-generation Map/Odometry cannot recover either.
        set_clock(101.3)
        node.map_callback(self._make_map(101.3))
        node.odom_callback(self._make_odom(101.3))
        set_clock(101.4)
        node.plan_callback(None)
        self.assertFalse(node.last_status[1])
        self.assertEqual(node.generation, 1)

        # No forced extra generation from a later real invalid heartbeat.
        set_clock(101.6)
        node.localization_status_callback(
            self._make_status(False, 101.6)
        )
        self.assertEqual(node.generation, 1)
        self.assertAlmostEqual(
            node.generation_start_stamp.to_sec(),
            101.0,
        )

        # A fresh valid localization status enables recovery.
        set_clock(101.7)
        node.localization_status_callback(
            self._make_status(True, 101.7)
        )
        self.assertNotIn("localization_invalid", node.active_faults)
        set_clock(101.8)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 1)
        self.assertGreater(len(self._last_path().poses), 0)

    def test_older_invalid_status_does_not_overwrite_newer_valid(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # A fresh valid status advances the per-stream baseline.
        set_clock(101.0)
        node.mapping_status_callback(self._make_status(True, 101.0))
        self.assertTrue(node.mapping_status.valid)
        statuses_before = len(
            self._published("/competition/planning/status")
        )

        # A delayed older invalid status (stamp < last accepted) must be
        # dropped: it must not overwrite the newer valid status, open a
        # generation, or publish an invalidation.
        set_clock(101.2)
        node.mapping_status_callback(self._make_status(False, 100.5))
        self.assertTrue(node.mapping_status.valid)
        self.assertNotIn("mapping_invalid", node.active_faults)
        self.assertEqual(node.generation, 0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(
            len(self._published("/competition/planning/status")),
            statuses_before,
        )
        self.assertAlmostEqual(
            node.last_mapping_status_stamp.to_sec(),
            101.0,
        )

    def test_duplicate_status_stamp_is_dropped(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])

        # Mapping invalid at 101.0 advances the baseline to 101.0.
        set_clock(101.0)
        node.mapping_status_callback(self._make_status(False, 101.0))
        self.assertEqual(node.generation, 1)
        statuses_before = len(
            self._published("/competition/planning/status")
        )

        # A re-delivered status with stamp == last accepted stamp is
        # dropped: no state change, no new publication, no generation.
        set_clock(101.2)
        node.mapping_status_callback(self._make_status(False, 101.0))
        self.assertEqual(node.generation, 1)
        self.assertIn("mapping_invalid", node.active_faults)
        self.assertEqual(
            len(self._published("/competition/planning/status")),
            statuses_before,
        )
        self.assertAlmostEqual(
            node.last_mapping_status_stamp.to_sec(),
            101.0,
        )

        # The first strictly newer status is accepted again.
        set_clock(101.3)
        node.mapping_status_callback(self._make_status(True, 101.3))
        self.assertTrue(node.mapping_status.valid)
        self.assertNotIn("mapping_invalid", node.active_faults)
        self.assertEqual(node.generation, 1)

    def test_clock_rollback_resets_status_baselines(self):
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)

        # Clock rolls back to 90.0: the rollback generation opens and
        # both per-stream baselines reset for the new timeline.
        set_clock(90.0)
        node.heartbeat_callback(None)
        self.assertEqual(node.generation, 1)
        self.assertIsNone(node.last_mapping_status_stamp)
        self.assertIsNone(node.last_localization_status_stamp)
        self.assertFalse(node.last_status[1])
        self.assertIn(
            "moved backwards",
            self._last_status_msg().detail,
        )

        # A status on the new timeline (stamp 91.0, lower than the
        # pre-rollback baseline 100.0) must be accepted; without the
        # baseline reset it would have been dropped as out-of-order.
        set_clock(91.0)
        node.mapping_status_callback(self._make_status(True, 91.0))
        node.localization_status_callback(
            self._make_status(True, 91.0)
        )
        self.assertTrue(node.mapping_status.valid)
        self.assertAlmostEqual(
            node.last_mapping_status_stamp.to_sec(),
            91.0,
        )
        self.assertAlmostEqual(
            node.last_localization_status_stamp.to_sec(),
            91.0,
        )

        # Fresh current-generation data after the new boundary (90.0)
        # enables recovery on the new timeline.
        set_clock(91.1)
        node.map_callback(self._make_map(91.1))
        node.odom_callback(self._make_odom(91.1))
        set_clock(91.2)
        node.plan_callback(None)
        self.assertTrue(node.last_status[1])
        self.assertGreater(len(self._last_path().poses), 0)

    def test_same_generation_input_update_still_commits(self):
        """Regression for the C2R2 live-integration starvation.

        A completed planning result must still commit when fresh map/odom
        objects arrive DURING planning while the generation is unchanged
        (no rollback, no invalid upstream status, no timeout).

        Pre-fix, ``plan_and_publish`` required ``self.map_msg is map_msg``
        and ``self.odom_msg is odom_msg`` (object identity), so under live
        odom streaming (~9 Hz) every plan was discarded and
        /competition/path never advanced past its first commit (fix-C2R2
        evidence: zero fresh paths, zero raw/guarded cmd_vel, no motion).
        Post-fix the commit check uses map_generation / odom_generation.
        """
        node = self.node

        self._publish_healthy_generation(100.0)
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)
        self.assertIsNotNone(self._last_path())
        paths_before = len(self._published("/competition/path"))

        original = PLANNING_NODE.plan_to_frontier_with_status

        def commit_with_mid_plan_input_update(*args, **kwargs):
            # Compute the real plan from the captured snapshot, then
            # simulate a live robot: fresh odom + map objects arrive
            # while the result is still in flight.
            result = original(*args, **kwargs)
            set_clock(100.4)
            node.odom_callback(self._make_odom(100.4))
            node.map_callback(self._make_map(100.4))
            return result

        PLANNING_NODE.plan_to_frontier_with_status = (
            commit_with_mid_plan_input_update
        )
        try:
            set_clock(100.3)
            node.plan_callback(None)
        finally:
            PLANNING_NODE.plan_to_frontier_with_status = original

        # Same-generation instance updates must not starve the commit.
        self.assertEqual(
            len(self._published("/competition/path")),
            paths_before + 1,
        )
        self.assertTrue(node.last_status[1])
        self.assertEqual(node.generation, 0)


if __name__ == "__main__":
    unittest.main()
