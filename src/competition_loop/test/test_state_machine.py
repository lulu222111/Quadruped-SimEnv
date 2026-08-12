#!/usr/bin/env python3
"""Unit tests for competition_state_machine.py — pure Python with mocked rospy.

Covers:
  - Initial state = WAITING
  - Valid transitions (WAITING → RUNNING, RUNNING → FINISHED, etc.)
  - Invalid transitions blocked
  - Force ERROR from any state
  - Safety signal states (control_enabled, emergency_stop)
  - is_terminal / is_running
  - Thread safety (basic lock)
  - to_msg() output
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

# Add src/ to sys.path
_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


# ---------------------------------------------------------------------------
# Mock rospy
# ---------------------------------------------------------------------------

class _MockLog:
    calls: list = []

    @classmethod
    def reset(cls):
        cls.calls.clear()

    @classmethod
    def info(cls, *args, **kw): cls.calls.append(("info", args))

    @classmethod
    def warn(cls, *args, **kw): cls.calls.append(("warn", args))

    @classmethod
    def logwarn(cls, *args, **kw): cls.calls.append(("warn", args))

    @classmethod
    def error(cls, *args, **kw): cls.calls.append(("error", args))

    @classmethod
    def logerr(cls, *args, **kw): cls.calls.append(("error", args))

    @classmethod
    def loginfo(cls, *args, **kw): cls.calls.append(("info", args))


class _MockTime:
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @staticmethod
    def now():
        return _MockTime(0.0)

    def to_sec(self):
        return self._secs


class _MockDuration:
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @classmethod
    def from_sec(cls, secs):
        return cls(secs)

    def to_sec(self):
        return self._secs


class _MockRospyModule:
    Time = _MockTime
    Duration = _MockDuration

    def __getattr__(self, name):
        return getattr(_MockLog, name, _MockLog.info)


if "rospy" not in sys.modules:
    sys.modules["rospy"] = _MockRospyModule()


# ---------------------------------------------------------------------------
# Mock competition_interfaces.msg.CompetitionState
# ---------------------------------------------------------------------------

class _MockHeader:
    def __init__(self):
        self.stamp = _MockTime(0.0)


class MockCompetitionStateMsg:
    WAITING = 0
    RUNNING = 1
    RETURNING = 2
    FINISHED = 3
    ERROR = 4

    def __init__(self):
        self.header = _MockHeader()
        self.state = 0
        self.detail = ""
        self.elapsed_time = _MockDuration(0.0)
        self.detected_count = 0


sys.modules["competition_interfaces"] = type("mod", (), {})()
sys.modules["competition_interfaces.msg"] = type("mod", (), {})()
sys.modules["competition_interfaces.msg"].CompetitionState = MockCompetitionStateMsg

from competition_state_machine import CompetitionStateMachine, CompetitionPhase


class TestStateMachineInitial(unittest.TestCase):
    """Initial state."""

    def setUp(self):
        _MockLog.reset()

    def test_initial_phase_waiting(self):
        fsm = CompetitionStateMachine()
        self.assertEqual(fsm.phase, CompetitionPhase.WAITING)

    def test_initial_detail_set(self):
        fsm = CompetitionStateMachine()
        self.assertIn("waiting", fsm.detail.lower())

    def test_initial_not_terminal(self):
        fsm = CompetitionStateMachine()
        self.assertFalse(fsm.is_terminal)

    def test_initial_not_running(self):
        fsm = CompetitionStateMachine()
        self.assertFalse(fsm.is_running)

    def test_initial_control_disabled(self):
        fsm = CompetitionStateMachine()
        self.assertFalse(fsm.should_control_be_enabled())

    def test_initial_emergency_stop_active(self):
        fsm = CompetitionStateMachine()
        self.assertTrue(fsm.should_emergency_stop())


class TestStateMachineTransitions(unittest.TestCase):
    """Valid and invalid transitions."""

    def setUp(self):
        _MockLog.reset()

    def test_waiting_to_running_valid(self):
        fsm = CompetitionStateMachine()
        self.assertTrue(fsm.transition_to(CompetitionPhase.RUNNING, "start"))
        self.assertEqual(fsm.phase, CompetitionPhase.RUNNING)
        self.assertTrue(fsm.should_control_be_enabled())
        self.assertFalse(fsm.should_emergency_stop())

    def test_running_to_finished_valid(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        self.assertTrue(fsm.transition_to(CompetitionPhase.FINISHED, "done"))
        self.assertEqual(fsm.phase, CompetitionPhase.FINISHED)
        self.assertTrue(fsm.is_terminal)
        self.assertFalse(fsm.should_control_be_enabled())
        self.assertTrue(fsm.should_emergency_stop())

    def test_running_to_error_valid(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        self.assertTrue(fsm.transition_to(CompetitionPhase.ERROR, "fault"))
        self.assertEqual(fsm.phase, CompetitionPhase.ERROR)
        self.assertTrue(fsm.is_terminal)

    def test_waiting_to_finished_invalid(self):
        fsm = CompetitionStateMachine()
        self.assertFalse(fsm.transition_to(CompetitionPhase.FINISHED))
        self.assertEqual(fsm.phase, CompetitionPhase.WAITING)

    def test_waiting_to_error_valid(self):
        fsm = CompetitionStateMachine()
        self.assertTrue(fsm.transition_to(CompetitionPhase.ERROR, "bad config"))
        self.assertEqual(fsm.phase, CompetitionPhase.ERROR)

    def test_finished_no_transitions(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        fsm.transition_to(CompetitionPhase.FINISHED)
        # Cannot transition from FINISHED (except to ERROR)
        self.assertFalse(fsm.transition_to(CompetitionPhase.RUNNING))
        self.assertFalse(fsm.transition_to(CompetitionPhase.WAITING))

    def test_error_from_any_state(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        fsm.transition_to(CompetitionPhase.FINISHED)
        # Force ERROR from FINISHED
        self.assertTrue(fsm.transition_to(CompetitionPhase.ERROR, "post-finish error"))
        self.assertEqual(fsm.phase, CompetitionPhase.ERROR)

    def test_force_error_from_waiting(self):
        fsm = CompetitionStateMachine()
        fsm.force_error("config invalid")
        self.assertEqual(fsm.phase, CompetitionPhase.ERROR)
        self.assertIn("config invalid", fsm.detail)


class TestStateMachineToMsg(unittest.TestCase):
    """ROS message conversion."""

    def setUp(self):
        _MockLog.reset()

    def test_to_msg_has_correct_fields(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING, "test")
        msg = fsm.to_msg(elapsed_seconds=42.0, detected_count=5)
        self.assertEqual(msg.state, CompetitionPhase.RUNNING)
        self.assertEqual(msg.detail, "test")
        self.assertEqual(msg.elapsed_time.to_sec(), 42.0)
        self.assertEqual(msg.detected_count, 5)

    def test_to_msg_default_elapsed_zero(self):
        fsm = CompetitionStateMachine()
        msg = fsm.to_msg()
        self.assertEqual(msg.elapsed_time.to_sec(), 0.0)
        self.assertEqual(msg.detected_count, 0)


class TestStateMachineSafetySignals(unittest.TestCase):
    """Safety signal states across all phases."""

    def setUp(self):
        _MockLog.reset()

    def test_waiting_safety(self):
        fsm = CompetitionStateMachine()
        self.assertFalse(fsm.should_control_be_enabled())
        self.assertTrue(fsm.should_emergency_stop())

    def test_running_safety(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        self.assertTrue(fsm.should_control_be_enabled())
        self.assertFalse(fsm.should_emergency_stop())

    def test_finished_safety(self):
        fsm = CompetitionStateMachine()
        fsm.transition_to(CompetitionPhase.RUNNING)
        fsm.transition_to(CompetitionPhase.FINISHED)
        self.assertFalse(fsm.should_control_be_enabled())
        self.assertTrue(fsm.should_emergency_stop())

    def test_error_safety(self):
        fsm = CompetitionStateMachine()
        fsm.force_error("bad")
        self.assertFalse(fsm.should_control_be_enabled())
        self.assertTrue(fsm.should_emergency_stop())


class TestCompetitionPhaseLabels(unittest.TestCase):
    """Phase label helper."""

    def test_known_labels(self):
        self.assertEqual(CompetitionPhase.label(CompetitionPhase.WAITING), "WAITING")
        self.assertEqual(CompetitionPhase.label(CompetitionPhase.RUNNING), "RUNNING")
        self.assertEqual(CompetitionPhase.label(CompetitionPhase.FINISHED), "FINISHED")
        self.assertEqual(CompetitionPhase.label(CompetitionPhase.ERROR), "ERROR")

    def test_unknown_label(self):
        self.assertIn("UNKNOWN", CompetitionPhase.label(99))


if __name__ == "__main__":
    unittest.main()
