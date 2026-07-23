#!/usr/bin/env python3
"""Unit tests for result_writer.py — pure Python, no ROS dependency."""

from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest
from pathlib import Path

# We need to mock rospy and competition_interfaces since this is a pure-Python test.
# Add the src/ directory to sys.path.
_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)


# Minimal mock for rospy so we can import result_writer without ROS infrastructure.
class _MockLog:
    @staticmethod
    def info(*args, **kw): pass
    @staticmethod
    def warn(*args, **kw): pass
    @staticmethod
    def logwarn(*args, **kw): pass
    @staticmethod
    def error(*args, **kw): pass
    @staticmethod
    def logerr(*args, **kw): pass
    @staticmethod
    def loginfo(*args, **kw): pass

class _MockRospy:
    def __getattr__(self, name):
        return getattr(_MockLog, name, _MockLog.info)

if "rospy" not in sys.modules:
    sys.modules["rospy"] = _MockRospy()


# Minimal mock for competition_interfaces.msg.HazardCandidate
class MockHazardCandidate:
    def __init__(self, frame_id="world", x=0.0, y=0.0, z=0.0,
                 label="", confidence=0.0, position_valid=True):
        self.header = type("Header", (), {})()
        self.header.frame_id = frame_id
        self.header.stamp = type("Stamp", (), {"to_sec": lambda: 0.0})()
        self.position = type("Point", (), {})()
        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.label = label
        self.confidence = confidence
        self.position_valid = position_valid

# Inject mock into the module before importing
sys.modules["competition_interfaces"] = type("mod", (), {})()
sys.modules["competition_interfaces.msg"] = type("mod", (), {})()
sys.modules["competition_interfaces.msg"].HazardCandidate = MockHazardCandidate

from result_writer import ResultWriter


class TestResultWriter(unittest.TestCase):
    """Full coverage: empty, single, multiple, invalid frame, Non-finite, JSON structure."""

    def setUp(self):
        self.tmpdir = tempfile.mkdtemp()
        self.outpath = Path(self.tmpdir) / "detected_danger.json"

    def tearDown(self):
        import shutil
        shutil.rmtree(self.tmpdir, ignore_errors=True)

    def _make_candidate(self, **kw):
        return MockHazardCandidate(**kw)

    # ------------------------------------------------------------------
    # Empty result
    # ------------------------------------------------------------------

    def test_empty_result(self):
        w = ResultWriter(str(self.outpath))
        w.set_exploration_time(0.0)
        path = w.write()
        data = json.loads(path.read_text())
        self.assertEqual(data["exploration_time"], 0.0)
        self.assertEqual(data["detected_danger_sources"], [])

    # ------------------------------------------------------------------
    # Single result
    # ------------------------------------------------------------------

    def test_single_result(self):
        w = ResultWriter(str(self.outpath))
        w.set_exploration_time(42.5)
        ok = w.accept_candidate(self._make_candidate(x=1.0, y=2.0, z=3.0))
        self.assertTrue(ok)
        path = w.write()
        data = json.loads(path.read_text())
        self.assertEqual(data["exploration_time"], 42.5)
        self.assertEqual(len(data["detected_danger_sources"]), 1)
        self.assertEqual(data["detected_danger_sources"][0]["position"], [1.0, 2.0, 3.0])

    # ------------------------------------------------------------------
    # Multiple results
    # ------------------------------------------------------------------

    def test_multiple_results(self):
        w = ResultWriter(str(self.outpath))
        candidates = [
            self._make_candidate(x=0.0, y=0.0, z=0.0),
            self._make_candidate(x=1.0, y=1.0, z=1.0),
            self._make_candidate(x=2.0, y=2.0, z=2.0),
        ]
        accepted = w.accept_candidates(candidates)
        self.assertEqual(accepted, 3)
        self.assertEqual(w.candidate_count, 3)
        path = w.write()
        data = json.loads(path.read_text())
        self.assertEqual(len(data["detected_danger_sources"]), 3)

    # ------------------------------------------------------------------
    # Invalid frame_id
    # ------------------------------------------------------------------

    def test_wrong_frame_rejected(self):
        w = ResultWriter(str(self.outpath))
        ok = w.accept_candidate(self._make_candidate(frame_id="base_link", x=1.0, y=2.0, z=3.0))
        self.assertFalse(ok)
        self.assertEqual(w.candidate_count, 0)

    # ------------------------------------------------------------------
    # position_valid=false
    # ------------------------------------------------------------------

    def test_invalid_position_rejected(self):
        w = ResultWriter(str(self.outpath))
        ok = w.accept_candidate(
            self._make_candidate(frame_id="world", x=1.0, y=2.0, z=3.0, position_valid=False)
        )
        self.assertFalse(ok)
        self.assertEqual(w.candidate_count, 0)

    # ------------------------------------------------------------------
    # Non-finite coordinates
    # ------------------------------------------------------------------

    def test_nan_rejected(self):
        import math
        w = ResultWriter(str(self.outpath))
        ok = w.accept_candidate(self._make_candidate(x=float("nan"), y=2.0, z=3.0))
        self.assertFalse(ok)

    def test_inf_rejected(self):
        w = ResultWriter(str(self.outpath))
        ok = w.accept_candidate(self._make_candidate(x=1.0, y=float("inf"), z=3.0))
        self.assertFalse(ok)

    # ------------------------------------------------------------------
    # JSON structure correctness
    # ------------------------------------------------------------------

    def test_json_structure_top_level_keys(self):
        w = ResultWriter(str(self.outpath))
        w.set_exploration_time(100.0)
        w.accept_candidate(self._make_candidate(x=5.0, y=6.0, z=7.0))
        path = w.write()
        data = json.loads(path.read_text())
        self.assertIn("exploration_time", data)
        self.assertIn("detected_danger_sources", data)
        self.assertIsInstance(data["exploration_time"], float)
        self.assertIsInstance(data["detected_danger_sources"], list)

    def test_json_structure_position_field(self):
        w = ResultWriter(str(self.outpath))
        w.accept_candidate(self._make_candidate(x=1.5, y=-2.5, z=0.25))
        path = w.write()
        data = json.loads(path.read_text())
        pos = data["detected_danger_sources"][0]["position"]
        self.assertIsInstance(pos, list)
        self.assertEqual(len(pos), 3)
        self.assertTrue(all(isinstance(v, float) for v in pos))

    # ------------------------------------------------------------------
    # Atomic write — temp file should not remain
    # ------------------------------------------------------------------

    def test_atomic_write_no_temp_file_left(self):
        w = ResultWriter(str(self.outpath))
        w.accept_candidate(self._make_candidate())
        before = set(os.listdir(self.tmpdir))
        w.write()
        after = set(os.listdir(self.tmpdir))
        new_files = after - before - {"detected_danger.json"}
        self.assertEqual(len(new_files), 0,
                         f"Temporary files left behind: {new_files}")

    # ------------------------------------------------------------------
    # Clear
    # ------------------------------------------------------------------

    def test_clear(self):
        w = ResultWriter(str(self.outpath))
        w.accept_candidate(self._make_candidate())
        self.assertEqual(w.candidate_count, 1)
        w.clear()
        self.assertEqual(w.candidate_count, 0)
        self.assertEqual(w._exploration_time, 0.0)


if __name__ == "__main__":
    unittest.main()
