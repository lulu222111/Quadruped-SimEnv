#!/usr/bin/env python3
"""Unit tests for cmd_vel_guard_node — pure Python, no ROS infrastructure needed.

Tests: disabled, emergency stop, timeout, velocity limiting, normal forwarding.
"""

from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

# ---------------------------------------------------------------------------
# Minimal ROS mock — we test the CmdVelGuard logic without rospy infrastructure
# ---------------------------------------------------------------------------

class _MockDuration:
    def __init__(self, secs):
        self._s = secs
    def to_sec(self):
        return self._s

class _MockTimeInstance:
    __slots__ = ('_sec',)
    def __init__(self, s=0.0): self._sec = s
    def to_sec(self): return self._sec
    def __sub__(self, other):
        if isinstance(other, _MockTimeInstance):
            return _MockDuration(self._sec - other._sec)
        return NotImplemented

class _MockTime:
    _now = 0.0
    @classmethod
    def now(cls):
        return _MockTimeInstance(cls._now)
    @classmethod
    def advance(cls, s):
        cls._now += s

class _MockLog:
    @staticmethod
    def info(*a, **kw): pass
    @staticmethod
    def warn(*a, **kw): pass
    @staticmethod
    def loginfo(*a, **kw): pass
    @staticmethod
    def logwarn(*a, **kw): pass
    @staticmethod
    def logerr(*a, **kw): pass
    @staticmethod
    def error(*a, **kw): pass

class _MockRospy:
    Time = _MockTime
    Duration = _MockDuration
    def __getattr__(self, n):
        if n in ("loginfo","logwarn","logerr","info","warn","error"):
            return getattr(_MockLog, n)
        return lambda *a,**kw: None
    def init_node(self, *a, **kw): pass
    def get_param(self, name, default):
        return default
    def Publisher(self, *a, **kw):
        return type("P", (), {"publish": lambda s,m: None})()
    def Subscriber(self, *a, **kw):
        return None
    def Timer(self, *a, **kw):
        return None
    def spin(self): pass
    Rate = lambda hz: type("R", (), {"sleep": lambda: None})()
    def on_shutdown(self, fn): pass
    def sleep(self, d): pass

sys.modules["rospy"] = _MockRospy()

# Fake geometry_msgs and std_msgs — must match geometry_msgs.msg.Twist etc.
class _Twist:
    def __init__(self):
        self.linear = type("L", (), {"x": 0.0, "y": 0.0, "z": 0.0})()
        self.angular = type("A", (), {"x": 0.0, "y": 0.0, "z": 0.0})()

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

# Fake competition_interfaces
class _ModuleStatus:
    UNKNOWN=0; STARTING=1; READY=2; RUNNING=3; SUCCEEDED=4
    BLOCKED=5; TIMEOUT=6; FAILED=7
    def __init__(self):
        self.header = type("H",(),{"stamp": type("S",(),{"to_sec": lambda:0.0})()})()
        self.state=0; self.valid=False; self.confidence=0.0; self.detail=""

sys.modules["competition_interfaces"] = type("m",(),{})()
sys.modules["competition_interfaces.msg"] = type("m",(),{"ModuleStatus": _ModuleStatus})()

# Now import the module under test
_SRC = str(Path(__file__).resolve().parents[1] / "scripts")
_NODE_PATH = f"{_SRC}/cmd_vel_guard_node"

# Load the cmd_vel_guard module without triggering its __main__ block.
# Using exec since the file has no .py extension and spec_from_file_location
# may not resolve it.
mod = type("mod", (), {})()
with open(_NODE_PATH) as f:
    source = f.read()
# Remove the __main__ guard block so we only get the class definition
_source_no_main = source.split("if __name__")[0]
exec(compile(_source_no_main, _NODE_PATH, "exec"), mod.__dict__)


class TestCmdVelGuard(unittest.TestCase):
    """Test cmd_vel_guard safety logic in isolation."""

    def setUp(self):
        _MockTime._now = 0.0
        # Create a guard instance with known parameters
        self.guard = mod.CmdVelGuard.__new__(mod.CmdVelGuard)
        # Manually init fields to avoid __init__ which hits rospy
        self.guard._timeout = 0.5
        self.guard._max_linear_x = 0.5
        self.guard._max_linear_y = 0.3
        self.guard._max_angular_z = 1.0
        self.guard._last_raw_cmd_time = None
        self.guard._control_enabled = False
        self.guard._emergency_stop = True
        self.guard._raw_linear_x = 0.0
        self.guard._raw_linear_y = 0.0
        self.guard._raw_angular_z = 0.0
        # Track published cmd_vel for inspection
        self._published = []
        self.guard._cmd_vel_pub = type("P", (), {"publish": lambda s, m: self._published.append(m)})()
        self.guard._status_pub = type("P", (), {"publish": lambda s, m: None})()
        self.guard._timer = None

    # ------------------------------------------------------------------
    # Disabled — should output zero regardless of raw input
    # ------------------------------------------------------------------

    def test_disabled_outputs_zero(self):
        self.guard._control_enabled = False
        self.guard._emergency_stop = False
        self.guard._raw_linear_x = 1.0
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        cmd = self._published[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.linear.y, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    # ------------------------------------------------------------------
    # Emergency stop — should output zero regardless of everything
    # ------------------------------------------------------------------

    def test_estop_outputs_zero(self):
        self.guard._control_enabled = True
        self.guard._emergency_stop = True
        self.guard._raw_linear_x = 1.0
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        cmd = self._published[-1]
        self.assertEqual(cmd.linear.x, 0.0)
        self.assertEqual(cmd.linear.y, 0.0)
        self.assertEqual(cmd.angular.z, 0.0)

    # ------------------------------------------------------------------
    # Timeout — no raw_cmd_vel within timeout → zero
    # ------------------------------------------------------------------

    def test_timeout_when_no_raw_cmd_ever(self):
        self.guard._control_enabled = True
        self.guard._emergency_stop = False
        self.guard._last_raw_cmd_time = None
        self.guard._control_loop(None)
        cmd = self._published[-1]
        self.assertEqual(cmd.linear.x, 0.0)

    def test_timeout_after_delay(self):
        self.guard._control_enabled = True
        self.guard._emergency_stop = False
        self.guard._raw_linear_x = 0.4
        self.guard._last_raw_cmd_time = _MockTime.now()  # t=0
        self.guard._control_loop(None)
        # Should still be fresh → forward
        self.assertAlmostEqual(self._published[-1].linear.x, 0.4)
        # Advance time past timeout
        _MockTime.advance(0.6)
        self.guard._control_loop(None)
        self.assertEqual(self._published[-1].linear.x, 0.0)

    # ------------------------------------------------------------------
    # Velocity limiting
    # ------------------------------------------------------------------

    def test_clamp_linear_x_positive(self):
        self._enable_normal()
        self.guard._raw_linear_x = 2.0  # exceeds max 0.5
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        self.assertAlmostEqual(self._published[-1].linear.x, 0.5)

    def test_clamp_linear_x_negative(self):
        self._enable_normal()
        self.guard._raw_linear_x = -2.0
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        self.assertAlmostEqual(self._published[-1].linear.x, -0.5)

    def test_clamp_linear_y(self):
        self._enable_normal()
        self.guard._raw_linear_y = 1.0  # exceeds max 0.3
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        self.assertAlmostEqual(self._published[-1].linear.y, 0.3)

    def test_clamp_angular_z(self):
        self._enable_normal()
        self.guard._raw_angular_z = 3.0  # exceeds max 1.0
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        self.assertAlmostEqual(self._published[-1].angular.z, 1.0)

    # ------------------------------------------------------------------
    # Normal forwarding
    # ------------------------------------------------------------------

    def test_forward_within_limits(self):
        self._enable_normal()
        self.guard._raw_linear_x = 0.25
        self.guard._raw_linear_y = -0.1
        self.guard._raw_angular_z = 0.5
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        cmd = self._published[-1]
        self.assertAlmostEqual(cmd.linear.x, 0.25)
        self.assertAlmostEqual(cmd.linear.y, -0.1)
        self.assertAlmostEqual(cmd.angular.z, 0.5)

    # ------------------------------------------------------------------
    # Unused DOF must be zero
    # ------------------------------------------------------------------

    def test_unused_dof_always_zero(self):
        self._enable_normal()
        self.guard._raw_linear_x = 0.2
        self.guard._raw_linear_y = 0.1
        self.guard._raw_angular_z = 0.3
        self.guard._last_raw_cmd_time = _MockTime.now()
        self.guard._control_loop(None)
        cmd = self._published[-1]
        self.assertEqual(cmd.linear.z, 0.0)
        self.assertEqual(cmd.angular.x, 0.0)
        self.assertEqual(cmd.angular.y, 0.0)

    # ------------------------------------------------------------------
    # Helper
    # ------------------------------------------------------------------

    def _enable_normal(self):
        self.guard._control_enabled = True
        self.guard._emergency_stop = False


if __name__ == "__main__":
    unittest.main()
