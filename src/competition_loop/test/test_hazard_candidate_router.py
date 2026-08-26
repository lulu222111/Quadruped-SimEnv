#!/usr/bin/env python3
"""Pure tests for candidate validation, transformation handoff, and dedup."""

from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

_SRC = str(Path(__file__).resolve().parents[1] / "src")
if _SRC not in sys.path:
    sys.path.insert(0, _SRC)

from hazard_candidate_router import HazardCandidateRouter, CandidateTransformError


class Stamp:
    def __init__(self, value=10.0):
        self.value = value

    def to_sec(self):
        return self.value


class Candidate:
    def __init__(self, x=1.0, y=2.0, z=3.0, frame="camera_depth_optical_frame"):
        self.header = type("Header", (), {})()
        self.header.frame_id = frame
        self.header.stamp = Stamp()
        self.position = type("Point", (), {})()
        self.position.x = x
        self.position.y = y
        self.position.z = z
        self.label = "red_sphere"
        self.confidence = 0.9
        self.position_valid = True


class Writer:
    def __init__(self):
        self.items = []

    def accept_candidate(self, candidate):
        self.items.append(candidate)
        return True


def to_world(candidate):
    result = Candidate(
        candidate.position.x + 10.0,
        candidate.position.y - 2.0,
        candidate.position.z,
        frame="world",
    )
    result.header.stamp = candidate.header.stamp
    return result


class TestHazardCandidateRouter(unittest.TestCase):
    def setUp(self):
        self.writer = Writer()
        self.router = HazardCandidateRouter(
            self.writer,
            dedup_distance=0.30,
            now_fn=lambda: 10.1,
        )

    def test_normal_candidate_transforms_and_writes(self):
        accepted = self.router.process([Candidate()], to_world)
        self.assertEqual(accepted, 1)
        self.assertEqual(len(self.writer.items), 1)
        self.assertEqual(self.writer.items[0].header.frame_id, "world")
        self.assertAlmostEqual(self.writer.items[0].position.x, 11.0)

    def test_tf_failure_rejects_candidate(self):
        def fail(_candidate):
            raise CandidateTransformError("no transform")

        accepted = self.router.process([Candidate()], fail)
        self.assertEqual(accepted, 0)
        self.assertEqual(self.router.last_stats["transform_failed"], 1)
        self.assertEqual(self.writer.items, [])

    def test_nan_rejects_candidate(self):
        candidate = Candidate(x=float("nan"))
        accepted = self.router.process([candidate], to_world)
        self.assertEqual(accepted, 0)
        self.assertEqual(self.router.last_stats["invalid_position"], 1)

    def test_duplicate_world_position_is_filtered(self):
        first = Candidate()
        second = Candidate(x=1.1, y=2.0, z=3.0)
        accepted = self.router.process([first, second], to_world)
        self.assertEqual(accepted, 1)
        self.assertEqual(self.router.last_stats["duplicate"], 1)
        self.assertEqual(len(self.writer.items), 1)

    def test_invalid_timestamp_rejects_candidate(self):
        candidate = Candidate()
        candidate.header.stamp = Stamp(0.0)
        accepted = self.router.process([candidate], to_world)
        self.assertEqual(accepted, 0)
        self.assertEqual(self.router.last_stats["invalid_timestamp"], 1)


if __name__ == "__main__":
    unittest.main()
