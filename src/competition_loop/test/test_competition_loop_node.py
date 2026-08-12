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

class _MockRospy:
    ROSException = type("ROSException", (Exception,), {})
    Time = _MockTime
    Duration = _MockDuration
    Publisher = _MockPub
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
        if n in ("ROSException","Time","Duration","Publisher","Service","Timer",
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
_ci_msg.ModuleStatus = type("MS",(),{"RUNNING":1})
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
sys.modules["nav_msgs"] = _nav
sys.modules["nav_msgs.msg"] = _nav_msg

_geo = type("m",(),{})()
_geo_msg = type("m",(),{})()
_geo.msg = _geo_msg
sys.modules["geometry_msgs"] = _geo
sys.modules["geometry_msgs.msg"] = _geo_msg

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


if __name__ == "__main__":
    unittest.main()
