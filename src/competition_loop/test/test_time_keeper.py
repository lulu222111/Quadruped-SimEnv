#!/usr/bin/env python3
"""Unit tests for time_keeper.py — pure Python with mocked rospy.

Covers:
  - Initial state (not running, zero elapsed)
  - Start / stop / reset
  - Elapsed calculation
  - Expiry detection
  - Time regression detection
  - Negative elapsed suppression
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
# Mock rospy with controllable time
# ---------------------------------------------------------------------------

class _MockTime:
    """Mutable mock for rospy.Time with controllable 'now'."""
    _now_secs = 100.0  # default time for now()

    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @classmethod
    def from_sec(cls, secs):
        return cls(secs)

    @staticmethod
    def now():
        return _MockTime(_MockTime._now_secs)

    @staticmethod
    def set_now(secs: float):
        _MockTime._now_secs = float(secs)

    def to_sec(self):
        return self._secs

    def __add__(self, other):
        if isinstance(other, _MockDuration):
            return _MockTime(self._secs + other.to_sec())
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, _MockTime):
            return _MockDuration(self._secs - other._secs)
        if isinstance(other, _MockDuration):
            return _MockTime(self._secs - other.to_sec())
        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, _MockTime):
            return self._secs < other._secs
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, _MockDuration):
            return self._secs >= other.to_sec()
        return NotImplemented

    def __eq__(self, other):
        if isinstance(other, _MockTime):
            return self._secs == other._secs
        return NotImplemented

    def is_zero(self):
        return self._secs == 0.0


class _MockDuration:
    """Mutable mock for rospy.Duration."""
    def __init__(self, secs=0.0):
        self._secs = float(secs)

    @classmethod
    def from_sec(cls, secs):
        return cls(secs)

    def to_sec(self):
        return self._secs

    def __add__(self, other):
        if isinstance(other, _MockDuration):
            return _MockDuration(self._secs + other._secs)
        return NotImplemented

    def __sub__(self, other):
        if isinstance(other, _MockDuration):
            return _MockDuration(self._secs - other._secs)
        return NotImplemented

    def __lt__(self, other):
        if isinstance(other, _MockDuration):
            return self._secs < other._secs
        return NotImplemented

    def __ge__(self, other):
        if isinstance(other, _MockDuration):
            return self._secs >= other._secs
        return NotImplemented

    def __eq__(self, other):
        if isinstance(other, _MockDuration):
            return self._secs == other._secs
        return NotImplemented


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


class _MockRospyModule:
    ROSException = type("ROSException", (Exception,), {})

    def __getattr__(self, name):
        if name in ("ROSException",):
            return getattr(self, name)
        return getattr(_MockLog, name, _MockLog.info)


if "rospy" not in sys.modules:
    sys.modules["rospy"] = _MockRospyModule()

# Patch rospy.Time and rospy.Duration with our controllable mocks
import rospy as mock_rospy
mock_rospy.Time = _MockTime
mock_rospy.Duration = _MockDuration

from time_keeper import TimeKeeper


class TestTimeKeeperInitial(unittest.TestCase):
    """Initial state."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)

    def test_initial_not_running(self):
        tk = TimeKeeper(max_time=600.0)
        self.assertFalse(tk.is_running)
        self.assertEqual(tk.elapsed_seconds, 0.0)

    def test_initial_not_expired(self):
        tk = TimeKeeper(max_time=600.0)
        self.assertFalse(tk.is_expired)

    def test_initial_no_regression(self):
        tk = TimeKeeper(max_time=600.0)
        self.assertFalse(tk.regression_detected)


class TestTimeKeeperStartStop(unittest.TestCase):
    """Start, stop, reset lifecycle."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)

    def test_start_sets_running(self):
        tk = TimeKeeper(max_time=600.0)
        tk.start()
        self.assertTrue(tk.is_running)

    def test_stop_sets_not_running(self):
        tk = TimeKeeper(max_time=600.0)
        tk.start()
        tk.stop()
        self.assertFalse(tk.is_running)

    def test_reset_clears_state(self):
        tk = TimeKeeper(max_time=600.0)
        tk.start()
        tk.stop()
        tk.reset()
        self.assertFalse(tk.is_running)
        self.assertEqual(tk.elapsed_seconds, 0.0)
        self.assertFalse(tk.regression_detected)

    def test_duplicate_start_is_idempotent(self):
        tk = TimeKeeper(max_time=600.0)
        tk.start()
        self.assertTrue(tk.is_running)
        tk.start()  # duplicate — should not raise, should still be running
        self.assertTrue(tk.is_running)


class TestTimeKeeperElapsed(unittest.TestCase):
    """Elapsed time calculation."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)

    def test_elapsed_zero_before_start(self):
        tk = TimeKeeper(max_time=600.0)
        self.assertEqual(tk.elapsed_seconds, 0.0)

    def test_elapsed_grows_after_start(self):
        tk = TimeKeeper(max_time=600.0)
        _MockTime.set_now(100.0)
        tk.start()  # start_time = 100.0
        self.assertEqual(tk.elapsed_seconds, 0.0)
        _MockTime.set_now(110.0)
        self.assertAlmostEqual(tk.elapsed_seconds, 10.0, places=1)
        _MockTime.set_now(160.0)
        self.assertAlmostEqual(tk.elapsed_seconds, 60.0, places=1)

    def test_remaining_decreases(self):
        tk = TimeKeeper(max_time=100.0)
        _MockTime.set_now(100.0)
        tk.start()
        self.assertAlmostEqual(tk.remaining_seconds, 100.0, places=1)
        _MockTime.set_now(140.0)
        self.assertAlmostEqual(tk.remaining_seconds, 60.0, places=1)

    def test_remaining_floor_zero(self):
        tk = TimeKeeper(max_time=10.0)
        _MockTime.set_now(100.0)
        tk.start()
        _MockTime.set_now(200.0)
        self.assertEqual(tk.remaining_seconds, 0.0)  # floored, not negative


class TestTimeKeeperExpiry(unittest.TestCase):
    """Timer expiry."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)

    def test_not_expired_initially(self):
        tk = TimeKeeper(max_time=600.0)
        self.assertFalse(tk.is_expired)

    def test_expired_after_max_time(self):
        tk = TimeKeeper(max_time=10.0)
        _MockTime.set_now(100.0)
        tk.start()
        _MockTime.set_now(110.0)
        self.assertTrue(tk.is_expired)


class TestTimeKeeperRegression(unittest.TestCase):
    """ROS time regression detection."""

    def setUp(self):
        _MockLog.reset()
        _MockTime.set_now(100.0)

    def test_no_regression_when_time_advances(self):
        tk = TimeKeeper(max_time=600.0)
        _MockTime.set_now(100.0)
        self.assertFalse(tk.check_time_regression())
        _MockTime.set_now(101.0)
        self.assertFalse(tk.check_time_regression())
        _MockTime.set_now(102.0)
        self.assertFalse(tk.check_time_regression())
        self.assertFalse(tk.regression_detected)

    def test_regression_detected_when_time_goes_backwards(self):
        tk = TimeKeeper(max_time=600.0)
        _MockTime.set_now(100.0)
        self.assertFalse(tk.check_time_regression())  # 100
        _MockTime.set_now(101.0)
        self.assertFalse(tk.check_time_regression())  # 101
        _MockTime.set_now(99.0)
        self.assertTrue(tk.check_time_regression())   # 99 < 101 → regression!
        self.assertTrue(tk.regression_detected)

    def test_elapsed_zero_when_clock_behind_start(self):
        """If clock jumps behind start_time, elapsed returns 0 (not negative)."""
        tk = TimeKeeper(max_time=600.0)
        _MockTime.set_now(50.0)
        tk.start()  # start_time = 50.0
        _MockTime.set_now(30.0)  # now behind start
        self.assertEqual(tk.elapsed_seconds, 0.0)  # not negative

    def test_sequential_100_101_01_02_03(self):
        """Sequence 100→101→0.1→0.2→0.3: only one regression detected.

        After regression at 0.1, the baseline is updated to 0.1.
        Subsequent 0.2 and 0.3 are NOT regressions (they are > 0.1).
        """
        tk = TimeKeeper(max_time=600.0)

        # Advance normally
        _MockTime.set_now(100.0)
        self.assertFalse(tk.check_time_regression(), "100: no regression")

        _MockTime.set_now(101.0)
        self.assertFalse(tk.check_time_regression(), "101: no regression")

        # Time resets to 0.1 — regression detected
        _MockTime.set_now(0.1)
        self.assertTrue(tk.check_time_regression(), "0.1: regression from 101")
        self.assertTrue(tk.regression_detected)

        # Next tick at 0.2 — baseline is now 0.1, so 0.2 > 0.1 → no regression
        _MockTime.set_now(0.2)
        self.assertFalse(tk.check_time_regression(), "0.2: no regression (baseline was 0.1)")
        self.assertFalse(tk.regression_detected)

        # Next tick at 0.3 — baseline is now 0.2, so 0.3 > 0.2 → no regression
        _MockTime.set_now(0.3)
        self.assertFalse(tk.check_time_regression(), "0.3: no regression (baseline was 0.2)")
        self.assertFalse(tk.regression_detected)


if __name__ == "__main__":
    unittest.main()
