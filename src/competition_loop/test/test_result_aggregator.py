#!/usr/bin/env python3
"""Unit tests for result_aggregator.py — pure Python, no ROS dependency."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from result_aggregator import aggregate_candidates


class _Candidate:
    def __init__(self, x, y, z=0.15, confidence=0.9):
        self.position = type("Point", (), {})()
        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.confidence = confidence


class TestAggregateCandidates(unittest.TestCase):
    def test_merges_close_candidates(self):
        cands = [
            _Candidate(1.0, 1.0),
            _Candidate(1.05, 1.03),
            _Candidate(1.1, 1.1),
        ]
        out = aggregate_candidates(cands, radius=1.0)
        self.assertEqual(len(out), 1)
        self.assertAlmostEqual(out[0].position.x, 1.05, places=6)
        self.assertAlmostEqual(out[0].position.y, 1.043333, places=6)
        self.assertEqual(out[0].count, 3)

    def test_keeps_far_candidates_separate(self):
        cands = [
            _Candidate(0.0, 0.0),
            _Candidate(5.0, 5.0),
            _Candidate(-4.0, 2.0),
        ]
        out = aggregate_candidates(cands, radius=1.0)
        self.assertEqual(len(out), 3)

    def test_confidence_is_max(self):
        cands = [
            _Candidate(1.0, 1.0, confidence=0.5),
            _Candidate(1.02, 1.0, confidence=0.95),
        ]
        out = aggregate_candidates(cands, radius=1.0)
        self.assertEqual(out[0].confidence, 0.95)

    def test_z_averaged(self):
        cands = [
            _Candidate(1.0, 1.0, z=0.1),
            _Candidate(1.0, 1.02, z=0.2),
        ]
        out = aggregate_candidates(cands, radius=1.0)
        self.assertAlmostEqual(out[0].position.z, 0.15, places=6)

    def test_bad_radius_rejected(self):
        with self.assertRaises(ValueError):
            aggregate_candidates([], radius=0.0)
        with self.assertRaises(ValueError):
            aggregate_candidates([], radius=float("nan"))


if __name__ == "__main__":
    unittest.main()
