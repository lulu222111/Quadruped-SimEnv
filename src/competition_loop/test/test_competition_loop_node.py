#!/usr/bin/env python3
"""Node-level tests for competition_loop_node — helper functions and path logic.

Full-constructor tests (CompetitionLoopNode()) require a ROS master and
are verified in the container via the ROS integration smoke test.
"""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_SRC = str(Path(__file__).resolve().parents[1] / "src")
_SCRIPTS = str(Path(__file__).resolve().parents[1] / "scripts")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)
if _SCRIPTS not in sys.path:
    sys.path.insert(0, _SCRIPTS)

# Minimal mock rospy (just enough for module imports)
class _MockLog:
    calls: list = []
    @classmethod
    def reset(cls): cls.calls.clear()
    @classmethod
    def info(cls, *a, **kw): cls.calls.append(("info", a))
    @classmethod
    def warn(cls, *a, **kw): cls.calls.append(("warn", a))
    @classmethod
    def logwarn(cls, *a, **kw): cls.calls.append(("warn", a))
    @classmethod
    def logwarn_throttle(cls, *a, **kw): cls.calls.append(("warn", a))
    @classmethod
    def error(cls, *a, **kw): cls.calls.append(("error", a))
    @classmethod
    def logerr(cls, *a, **kw): cls.calls.append(("error", a))
    @classmethod
    def logfatal(cls, *a, **kw): cls.calls.append(("fatal", a))
    @classmethod
    def loginfo(cls, *a, **kw): cls.calls.append(("info", a))
    INFO = 1

class _MockTime:
    _now_secs = 100.0
    def __init__(self, secs=0.0): self._secs = float(secs)
    @staticmethod
    def now(): return _MockTime(_MockTime._now_secs)
    @staticmethod
    def set_now(secs): _MockTime._now_secs = float(secs)
    @classmethod
    def from_sec(cls, secs): return cls(secs)
    def to_sec(self): return self._secs
    def is_zero(self): return self._secs == 0.0
    def __add__(self, o):
        if hasattr(o, "to_sec"): return _MockTime(self._secs + o.to_sec())
        return NotImplemented
    def __sub__(self, o):
        if isinstance(o, _MockTime): return _MockDuration(self._secs - o._secs)
        if hasattr(o, "to_sec"): return _MockTime(self._secs - o.to_sec())
        return NotImplemented
    def __lt__(self, o):
        if isinstance(o, _MockTime): return self._secs < o._secs
        return NotImplemented
    def __ge__(self, o):
        if hasattr(o, "to_sec"): return self._secs >= o.to_sec()
        return NotImplemented
    def __eq__(self, o):
        if isinstance(o, _MockTime): return self._secs == o._secs
        return NotImplemented

class _MockDuration:
    def __init__(self, secs=0.0): self._secs = float(secs)
    @classmethod
    def from_sec(cls, secs): return cls(secs)
    def to_sec(self): return self._secs
    def __add__(self, o):
        if hasattr(o, "to_sec"): return _MockDuration(self._secs + o.to_sec())
        return NotImplemented
    def __sub__(self, o):
        if hasattr(o, "to_sec"): return _MockDuration(self._secs - o.to_sec())
        return NotImplemented
    def __lt__(self, o):
        if hasattr(o, "to_sec"): return self._secs < o.to_sec()
        return NotImplemented
    def __ge__(self, o):
        if hasattr(o, "to_sec"): return self._secs >= o.to_sec()
        return NotImplemented
    def __eq__(self, o):
        if hasattr(o, "to_sec"): return self._secs == o.to_sec()
        return NotImplemented

class _MockPub:
    def __init__(self, *a, **kw): self.msgs = []
    def publish(self, m): self.msgs.append(m)

class _MockSub:
    def __init__(self, *a, **kw): self.args = a; self.kwargs = kw
    def unregister(self): pass

class _MockTfBuffer:
    def __init__(self, *a, **kw): pass

class _MockTfListener:
    def __init__(self, *a, **kw): pass

class _MockRospy:
    ROSException = type("ROSException", (Exception,), {})
    Time = _MockTime
    Duration = _MockDuration
    Publisher = _MockPub
    Subscriber = _MockSub
    Service = type("Svc", (), {"__init__": lambda s, *a, **kw: None})
    Timer = type("Timer", (), {"__init__": lambda s, *a, **kw: None})
    INFO = 1
    def __init__(self):
        self._params = {}
    def get_param(self, n, d=None): return self._params.get(n, d)
    def init_node(self, *a, **kw): pass
    def get_published_topics(self): return [["/rosout", "rosgraph_msgs/Log"]]
    def spin(self): pass
    def __getattr__(self, n):
        if n in ("ROSException","Time","Duration","Publisher","Subscriber","Service","Timer",
                  "INFO","_params","get_param","init_node","get_published_topics","spin"):
            return object.__getattribute__(self, n)
        return getattr(_MockLog, n, _MockLog.info)

_mock = _MockRospy()
sys.modules["rospy"] = _mock

# Mock other ROS modules
_std_msgs = type("m",(),{})()
_std_msgs_msg = type("m",(),{})()
_std_msgs.msg = _std_msgs_msg
_std_msgs_msg.Bool = type("Bool",(),{"__init__":lambda s,data=None:setattr(s,"data",data)})
sys.modules["std_msgs"] = _std_msgs
sys.modules["std_msgs.msg"] = _std_msgs_msg
_std_srvs = type("m",(),{})()
_std_srvs_srv = type("m",(),{})()
_std_srvs.srv = _std_srvs_srv
_std_srvs_srv.Trigger = type("T",(),{})
_std_srvs_srv.TriggerResponse = type("TR",(),{"__init__":lambda s,**kw:[setattr(s,k,v) for k,v in kw.items()]})
sys.modules["std_srvs"] = _std_srvs
sys.modules["std_srvs.srv"] = _std_srvs_srv

_ci = type("m",(),{})()
_ci_msg = type("m",(),{})()
_ci.msg = _ci_msg
_CSM = type("CS",(),{"WAITING":0,"RUNNING":1,"RETURNING":2,"FINISHED":3,"ERROR":4})
def _csm_init(s, **kw):
    s.header = type("Hdr",(),{"stamp":None})()
    for k, v in kw.items():
        setattr(s, k, v)
_CSM.__init__ = _csm_init
_ci_msg.CompetitionState = _CSM
_ci_msg.HazardCandidate = type("HC",(),{"__init__":lambda s,**kw:[setattr(s,k,v) for k,v in kw.items()]})
_ci_msg.HazardCandidateArray = type("HCA",(),{"__init__":lambda s,**kw:[setattr(s,k,v) for k,v in kw.items()]})
class _ModuleStatus:
    STARTING = 1
    RUNNING = 2
    SUCCEEDED = 3
    BLOCKED = 4
    TIMEOUT = 5
    FAILED = 6

    def __init__(self, **kw):
        self.header = type("Hdr", (), {"stamp": None})()
        self.state = self.STARTING
        self.valid = False
        self.confidence = 0.0
        self.detail = ""
        for key, value in kw.items():
            setattr(self, key, value)

_ci_msg.ModuleStatus = _ModuleStatus
sys.modules["competition_interfaces"] = _ci
sys.modules["competition_interfaces.msg"] = _ci_msg
_bgi = type("m",(),{})()
_bgi_srv = type("m",(),{})()
_bgi.srv = _bgi_srv
sys.modules["building_generator_interfaces"] = _bgi
sys.modules["building_generator_interfaces.srv"] = _bgi_srv

_nav = type("m",(),{})()
_nav_msg = type("m",(),{})()
_nav.msg = _nav_msg
class _Odometry:
    def __init__(self):
        self.header = type("Hdr", (), {"stamp": _MockTime(0.0)})()
        self.pose = type(
            "PoseWithCovariance",
            (),
            {"pose": type("Pose", (), {
                "position": type("Point", (), {"x": 0.0, "y": 0.0, "z": 0.0})(),
            })()},
        )()
_nav_msg.Odometry = _Odometry
sys.modules["nav_msgs"] = _nav
sys.modules["nav_msgs.msg"] = _nav_msg

_geo = type("m",(),{})()
_geo_msg = type("m",(),{})()
_geo.msg = _geo_msg
sys.modules["geometry_msgs"] = _geo
sys.modules["geometry_msgs.msg"] = _geo_msg

_tf2 = type("m", (), {})()
_tf2.Buffer = _MockTfBuffer
_tf2.TransformListener = _MockTfListener
_tf2.LookupException = type("LookupException", (Exception,), {})
_tf2.ConnectivityException = type("ConnectivityException", (Exception,), {})
_tf2.ExtrapolationException = type("ExtrapolationException", (Exception,), {})
sys.modules["tf2_ros"] = _tf2

# Import the module
_cn_path = str(Path(__file__).resolve().parents[1] / "scripts" / "competition_loop_node")
import types as _types
_cn_mod = _types.ModuleType("competition_loop_node")
_cn_mod.__file__ = _cn_path
with open(_cn_path, "r") as f:
    _cn_code = compile(f.read(), _cn_path, "exec")
exec(_cn_code, _cn_mod.__dict__)
sys.modules["competition_loop_node"] = _cn_mod

_is_valid_finite_positive_number = _cn_mod._is_valid_finite_positive_number
_is_valid_finite_non_negative_number = _cn_mod._is_valid_finite_non_negative_number
_safe_repr = _cn_mod._safe_repr
CompetitionLoopNode = _cn_mod.CompetitionLoopNode
CompetitionPhase = _cn_mod.CompetitionPhase


class TestParameterHelpers(unittest.TestCase):
    """Test standalone parameter validation helpers."""

    def test_positive_rejects_bool_true(self):
        self.assertFalse(_is_valid_finite_positive_number(True))

    def test_positive_rejects_bool_false(self):
        self.assertFalse(_is_valid_finite_positive_number(False))

    def test_positive_rejects_string(self):
        self.assertFalse(_is_valid_finite_positive_number("600"))

    def test_positive_rejects_nan(self):
        self.assertFalse(_is_valid_finite_positive_number(float("nan")))

    def test_positive_rejects_inf(self):
        self.assertFalse(_is_valid_finite_positive_number(float("inf")))

    def test_positive_rejects_neg_inf(self):
        self.assertFalse(_is_valid_finite_positive_number(float("-inf")))

    def test_positive_rejects_zero(self):
        self.assertFalse(_is_valid_finite_positive_number(0))

    def test_positive_rejects_negative(self):
        self.assertFalse(_is_valid_finite_positive_number(-1))

    def test_positive_accepts_int(self):
        self.assertTrue(_is_valid_finite_positive_number(600))

    def test_positive_accepts_float(self):
        self.assertTrue(_is_valid_finite_positive_number(600.0))

    def test_non_negative_rejects_bool(self):
        self.assertFalse(_is_valid_finite_non_negative_number(True))

    def test_non_negative_accepts_zero(self):
        self.assertTrue(_is_valid_finite_non_negative_number(0))

    def test_non_negative_accepts_positive(self):
        self.assertTrue(_is_valid_finite_non_negative_number(5.0))

    def test_non_negative_rejects_nan(self):
        self.assertFalse(_is_valid_finite_non_negative_number(float("nan")))

    def test_safe_repr_handles_string(self):
        self.assertIsInstance(_safe_repr("hello"), str)

    def test_safe_repr_handles_bool(self):
        self.assertIsInstance(_safe_repr(True), str)

    def test_safe_repr_handles_nan(self):
        self.assertIsInstance(_safe_repr(float("nan")), str)


class TestParameterValidationInNode(unittest.TestCase):
    """Verify invalid parameters → ERROR state, all members exist (no crash)."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)
        _mock._params = {
            "~max_time": 600.0, "~start_delay": 5.0,
            "~auto_start": True, "~competition_mode": True,
            "~result_path": None, "~workspace_root": "/workspace/SimEnv",
        }

    def _make_node(self, **overrides):
        """Create a CompetitionLoopNode with parameter overrides."""
        _mock._params.update(overrides)
        node = CompetitionLoopNode()
        return node

    def test_invalid_max_time_bool_enters_error(self):
        node = self._make_node(**{"~max_time": True})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)
        self.assertTrue(hasattr(node, "_timer"))
        self.assertTrue(hasattr(node, "_validator"))
        self.assertTrue(hasattr(node, "_result_writer"))
        self.assertTrue(hasattr(node, "_tick_timer"))

    def test_invalid_max_time_nan_enters_error(self):
        node = self._make_node(**{"~max_time": float("nan")})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_max_time_inf_enters_error(self):
        node = self._make_node(**{"~max_time": float("inf")})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_max_time_neg_inf_enters_error(self):
        node = self._make_node(**{"~max_time": float("-inf")})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_max_time_string_enters_error(self):
        node = self._make_node(**{"~max_time": "bad"})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_max_time_zero_enters_error(self):
        node = self._make_node(**{"~max_time": 0})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_start_delay_negative_enters_error(self):
        node = self._make_node(**{"~start_delay": -1.0})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_start_delay_bool_enters_error(self):
        node = self._make_node(**{"~start_delay": False})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_start_delay_nan_enters_error(self):
        node = self._make_node(**{"~start_delay": float("nan")})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_competition_mode_string_enters_error(self):
        node = self._make_node(**{"~competition_mode": "yes"})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_invalid_workspace_root_empty_enters_error(self):
        node = self._make_node(**{"~workspace_root": ""})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)

    def test_error_state_safe_signals(self):
        node = self._make_node(**{"~max_time": float("nan")})
        self.assertEqual(node._fsm.phase, CompetitionPhase.ERROR)
        self.assertFalse(node._control_enabled_pub.msgs[-1].data,
                         "control_enabled must be false in ERROR")
        self.assertTrue(node._emergency_stop_pub.msgs[-1].data,
                        "emergency_stop must be true in ERROR")


class TestResultPathResolution(unittest.TestCase):
    """Result path resolution logic."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)
        _mock._params = {
            "~max_time": 600.0, "~start_delay": 5.0,
            "~auto_start": True, "~competition_mode": False,
            "~result_path": None, "~workspace_root": "/workspace/SimEnv",
        }

    def _make_node(self, **overrides):
        _mock._params.update(overrides)
        return CompetitionLoopNode()

    def test_default_result_path(self):
        node = self._make_node(**{"~result_path": None, "~workspace_root": "/workspace/SimEnv"})
        self.assertEqual(str(node._result_path), "/workspace/SimEnv/results/detected_danger.json")

    def test_relative_result_path(self):
        node = self._make_node(**{"~result_path": "my_results/output.json", "~workspace_root": "/tmp/test-ws"})
        self.assertEqual(str(node._result_path), "/tmp/test-ws/my_results/output.json")

    def test_absolute_result_path(self):
        node = self._make_node(**{"~result_path": "/tmp/output/danger.json", "~workspace_root": "/workspace/SimEnv"})
        self.assertEqual(str(node._result_path), "/tmp/output/danger.json")

    def test_result_path_nonexistent_parent(self):
        node = self._make_node(**{"~result_path": "deep/nested/dir/out.json", "~workspace_root": "/tmp/ghost"})
        self.assertEqual(str(node._result_path), "/tmp/ghost/deep/nested/dir/out.json")


class _MissionRouterStub:
    def __init__(self, accepted_counts):
        self._accepted_counts = list(accepted_counts)

    def process(self, candidates, transform_fn):
        del candidates, transform_fn
        return self._accepted_counts.pop(0) if self._accepted_counts else 0


class _MissionTimerStub:
    elapsed_seconds = 4.0

    def __init__(self, events):
        self._events = events

    def stop(self):
        self._events.append("timer_stop")


class _MissionWriterStub:
    candidate_count = 1

    def __init__(self, events):
        self._events = events

    def set_exploration_time(self, seconds):
        self._events.append(("set_time", seconds))

    def write(self):
        self._events.append("write")


class TestMissionFoundDanger(unittest.TestCase):
    """Mission-level found event and first-detection finish policy."""

    def _make_node(
        self,
        accepted_counts,
        finish_on_first_detection=False,
        return_on_first_detection=False,
    ):
        node = object.__new__(CompetitionLoopNode)
        node._fsm = _cn_mod.CompetitionStateMachine()
        node._fsm.transition_to(CompetitionPhase.RUNNING, "exploring")
        node._candidate_router = _MissionRouterStub(accepted_counts)
        node._result_writer = type("Writer", (), {"candidate_count": 1})()
        node._finish_on_first_detection = finish_on_first_detection
        node._return_on_first_detection = return_on_first_detection
        node._return_start_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        node._return_arrival_count = 0
        node._return_arrival_samples = 3
        node._return_arrival_tolerance = 0.25
        node._return_input_timeout = 1.0
        node._return_odom = None
        node._return_localization_status = None
        node._return_follower_status = None
        node._found_danger = False
        node._published_states = []
        node._safety_events = []
        node._publish_state = lambda: node._published_states.append(node._fsm.detail)
        node._publish_safety_signals = lambda **kwargs: node._safety_events.append(kwargs)
        node._transform_candidate_to_world = lambda candidate: candidate
        node._end_competition = lambda reason: node._published_states.append(
            "END:" + reason
        )
        return node

    def test_first_candidate_triggers_found_danger(self):
        node = self._make_node([1])
        node._hazard_candidates_callback(type("Msg", (), {"candidates": []})())
        self.assertTrue(node._found_danger)
        self.assertIn("FOUND_DANGER", node._published_states)

    def test_duplicate_does_not_retrigger_found_danger(self):
        node = self._make_node([1, 0])
        msg = type("Msg", (), {"candidates": []})()
        node._hazard_candidates_callback(msg)
        first_publication_count = len(node._published_states)
        node._hazard_candidates_callback(msg)
        self.assertEqual(len(node._published_states), first_publication_count)

    def test_first_detection_finish_policy_calls_finish_after_event(self):
        node = self._make_node([1], finish_on_first_detection=True)
        node._hazard_candidates_callback(type("Msg", (), {"candidates": []})())
        self.assertEqual(node._published_states, ["FOUND_DANGER", "END:FOUND_DANGER: finish_on_first_detection"])

    def test_first_candidate_enters_returning_once(self):
        node = self._make_node([1, 1], return_on_first_detection=True)
        msg = type("Msg", (), {"candidates": []})()
        node._hazard_candidates_callback(msg)
        self.assertEqual(node._fsm.phase, CompetitionPhase.RETURNING)
        self.assertIn("RETURNING", node._published_states)
        first_event_count = len(node._published_states)
        node._hazard_candidates_callback(msg)
        self.assertEqual(len(node._published_states), first_event_count)

    def test_no_candidate_keeps_running_without_event(self):
        node = self._make_node([0])
        node._hazard_candidates_callback(type("Msg", (), {"candidates": []})())
        self.assertFalse(node._found_danger)
        self.assertEqual(node._fsm.phase, CompetitionPhase.RUNNING)
        self.assertEqual(node._published_states, [])

    def test_finish_stops_before_result_write(self):
        events = []
        node = object.__new__(CompetitionLoopNode)
        node._fsm = _cn_mod.CompetitionStateMachine()
        node._fsm.transition_to(CompetitionPhase.RUNNING, "exploring")
        node._timer = _MissionTimerStub(events)
        node._result_writer = _MissionWriterStub(events)
        node._publish_safety_signals = lambda **kwargs: events.append("safety_stop")
        node._publish_state = lambda: events.append("state")
        node._end_competition("FOUND_DANGER")
        self.assertLess(events.index("safety_stop"), events.index("write"))

    def test_return_arrival_requires_persistent_valid_inputs(self):
        ros_time = _cn_mod.rospy.Time
        ros_time.set_now(100.0)
        node = self._make_node([], return_on_first_detection=True)
        node._fsm.transition_to(CompetitionPhase.RETURNING, "return")
        node._detect_time_regression = lambda: False
        node._timer = type("Timer", (), {"is_expired": False})()
        node._end_competition = lambda reason: node._published_states.append(
            "END:" + reason
        )
        odom = _Odometry()
        odom.header.stamp = ros_time(100.0)
        odom.pose.pose.position.x = 0.1
        odom.pose.pose.position.y = 0.1
        localization = _ModuleStatus()
        localization.header.stamp = ros_time(100.0)
        localization.valid = True
        follower = _ModuleStatus()
        follower.header.stamp = ros_time(100.0)
        follower.valid = True
        follower.state = _ModuleStatus.SUCCEEDED
        node._return_odom = odom
        node._return_localization_status = localization
        node._return_follower_status = follower

        node._tick_returning()
        node._tick_returning()
        self.assertEqual(node._fsm.phase, CompetitionPhase.RETURNING)
        self.assertNotIn("END:ARRIVED_START", node._published_states)
        node._tick_returning()
        self.assertIn("ARRIVED_START", node._published_states)
        self.assertIn("END:ARRIVED_START", node._published_states)

    def test_stale_localization_resets_arrival_persistence(self):
        ros_time = _cn_mod.rospy.Time
        ros_time.set_now(100.0)
        node = self._make_node([], return_on_first_detection=True)
        node._fsm.transition_to(CompetitionPhase.RETURNING, "return")
        node._detect_time_regression = lambda: False
        node._timer = type("Timer", (), {"is_expired": False})()
        node._end_competition = lambda reason: node._published_states.append(
            "END:" + reason
        )
        odom = _Odometry()
        odom.pose.pose.position.x = 0.0
        odom.pose.pose.position.y = 0.0
        odom.header.stamp = ros_time(100.0)
        localization = _ModuleStatus()
        localization.valid = True
        localization.header.stamp = ros_time(100.0)
        follower = _ModuleStatus()
        follower.valid = True
        follower.state = _ModuleStatus.SUCCEEDED
        follower.header.stamp = ros_time(100.0)
        node._return_odom = odom
        node._return_localization_status = localization
        node._return_follower_status = follower
        node._tick_returning()
        self.assertEqual(node._return_arrival_count, 1)
        localization.header.stamp = ros_time(98.0)
        node._tick_returning()
        self.assertEqual(node._return_arrival_count, 0)


class TestFloorLifecycleCompletion(unittest.TestCase):
    """The floor-complete evidence chain must drive RETURNING exactly once.

    The planner emits the Bool level and the confirming SUCCEEDED status from
    the same completion commit, and re-asserts the level on every heartbeat
    while the condition holds.  These tests pin the resulting loop behaviour:
    a *jointly fresh* pair enters RETURNING once, and a pair that is never
    jointly fresh cannot.
    """

    def _make_node(self):
        node = object.__new__(CompetitionLoopNode)
        node._fsm = _cn_mod.CompetitionStateMachine()
        node._fsm.transition_to(CompetitionPhase.RUNNING, "exploring")
        node._floor_lifecycle = _cn_mod.FloorLifecycle(
            {
                "door_ids": [{"kind": "main_entrance", "floor_index": 0}],
                "elevators": [],
            }
        )
        node._published_states = []
        node._publish_state = lambda: node._published_states.append(
            node._fsm.detail
        )
        node._publish_safety_signals = lambda **kwargs: None
        node._return_start_pose = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}
        node._return_arrival_count = 0
        node._detect_time_regression = lambda: False
        node._timer = type("Timer", (), {"is_expired": False})()
        return node

    def _complete(self, node, now):
        """One planner completion commit as the planner actually emits it."""
        floor = node._floor_lifecycle
        floor.observe_status(
            floor.task_id, now, True, True, now
        )
        floor.observe_bool(True, now)

    def test_jointly_fresh_evidence_enters_returning_once(self):
        ros_time = _cn_mod.rospy.Time
        ros_time.set_now(100.0)
        node = self._make_node()
        floor = node._floor_lifecycle
        floor.activate(0, 100.0)

        # Activation re-arms the edge detector with an explicit False level.
        floor.observe_bool(False, 100.1)

        ros_time.set_now(101.0)
        self._complete(node, 101.0)
        node._tick_running()
        self.assertEqual(node._fsm.phase, CompetitionPhase.RETURNING)
        self.assertFalse(floor.active)

        # Consumed exactly once: a repeated fresh pair cannot re-fire.
        return_count = len(node._published_states)
        ros_time.set_now(101.5)
        self._complete(node, 101.5)
        node._tick_running()
        self.assertEqual(len(node._published_states), return_count)
        self.assertEqual(node._fsm.phase, CompetitionPhase.RETURNING)

    def test_expired_evidence_never_enters_returning(self):
        ros_time = _cn_mod.rospy.Time
        ros_time.set_now(100.0)
        node = self._make_node()
        floor = node._floor_lifecycle
        floor.activate(0, 100.0)
        floor.observe_bool(False, 100.1)

        # Bool and status are each individually valid but never jointly fresh:
        # the confirming status arrives long after the level was last asserted.
        floor.observe_bool(True, 101.0)
        floor.observe_status(floor.task_id, 103.0, True, True, 103.0)
        ros_time.set_now(103.0)
        node._tick_running()
        self.assertEqual(node._fsm.phase, CompetitionPhase.RUNNING)


class TestCandidateWorldZSnap(unittest.TestCase):
    """P2: world-frame candidate z is snapped to source-centre height."""

    def _make_node(self, ground_z=0.0, radius=0.15):
        node = object.__new__(CompetitionLoopNode)
        node._hazard_source_ground_z = ground_z
        node._hazard_source_radius = radius
        return node

    def _make_candidate(self, z, valid=True):
        point = type("Point", (), {})()
        point.x, point.y, point.z = 3.0, 4.0, z
        cand = object.__new__(_ci_msg.HazardCandidate)
        cand.position = point
        cand.position_valid = valid
        cand.label = "red_sphere"
        cand.confidence = 1.0
        return cand

    def test_snap_z_to_ground_plus_radius(self):
        node = self._make_node(ground_z=0.0, radius=0.15)
        cand = self._make_candidate(z=0.9)
        node._snap_source_center_z(cand)
        self.assertAlmostEqual(cand.position.z, 0.15, places=6)

    def test_snap_respects_ground_z_and_radius_params(self):
        node = self._make_node(ground_z=3.0, radius=0.15)
        cand = self._make_candidate(z=3.9)
        node._snap_source_center_z(cand)
        self.assertAlmostEqual(cand.position.z, 3.15, places=6)

    def test_snap_preserves_xy(self):
        node = self._make_node()
        cand = self._make_candidate(z=0.9)
        node._snap_source_center_z(cand)
        self.assertEqual((cand.position.x, cand.position.y), (3.0, 4.0))

    def test_snap_skips_invalid_candidates(self):
        node = self._make_node()
        cand = self._make_candidate(z=0.9, valid=False)
        node._snap_source_center_z(cand)
        self.assertAlmostEqual(cand.position.z, 0.9, places=6)


if __name__ == "__main__":
    unittest.main()
