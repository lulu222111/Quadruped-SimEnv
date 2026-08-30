#!/usr/bin/env python3

"""Deterministic tests for planner active visual coverage bookkeeping."""

import math
import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from visual_coverage import (  # noqa: E402
    SCAN_COMPLETE,
    SCAN_DWELLING,
    SCAN_HEADING_REACHED,
    SCAN_HEADING_TIMEOUT,
    SCAN_NEXT_HEADING,
    SCAN_RETRY_HEADING,
    SCAN_SKIPPED,
    SCAN_STARTED,
    SCAN_TIMEOUT,
    VisualCoverageTracker,
)


class VisualCoverageTrackerTest(unittest.TestCase):
    def make_tracker(self, **overrides):
        params = {
            "yaw_step": math.pi / 2.0,
            "yaw_tolerance": 0.1,
            "dwell": 1.0,
            "heading_timeout": 2.0,
            "scan_timeout": 20.0,
            "position_dedup_radius": 1.0,
            "max_heading_retries": 1,
        }
        params.update(overrides)
        return VisualCoverageTracker(**params)

    def test_disabled_scan_is_skipped(self):
        tracker = self.make_tracker(enabled=False)
        self.assertEqual(tracker.start((0.0, 0.0), 0.0, 1.0).kind, SCAN_SKIPPED)
        self.assertFalse(tracker.active)

    def test_new_position_starts_bounded_heading_sequence(self):
        tracker = self.make_tracker()
        event = tracker.start((0.0, 0.0), 0.0, 1.0)
        self.assertEqual(event.kind, SCAN_STARTED)
        self.assertEqual(event.heading_count, 4)
        self.assertEqual(event.heading_index, 0)
        self.assertAlmostEqual(tracker.current_heading, 0.0)

    def test_heading_requires_reach_then_dwell_then_next(self):
        tracker = self.make_tracker()
        tracker.start((0.0, 0.0), 0.0, 1.0)

        self.assertEqual(
            tracker.update(0.0, 1.0).kind,
            SCAN_HEADING_REACHED,
        )
        self.assertEqual(tracker.update(0.0, 1.5).kind, SCAN_DWELLING)
        event = tracker.update(0.0, 2.1)
        self.assertEqual(event.kind, SCAN_NEXT_HEADING)
        self.assertEqual(event.heading_index, 1)
        self.assertAlmostEqual(event.target_yaw, math.pi / 2.0)

    def test_unreached_heading_does_not_start_dwell(self):
        tracker = self.make_tracker()
        tracker.start((0.0, 0.0), 0.0, 1.0)
        event = tracker.update(math.pi / 2.0, 1.1)
        self.assertEqual(event.kind, SCAN_DWELLING)
        self.assertIsNone(tracker.reached_at)

    def test_all_headings_complete_records_position_once(self):
        tracker = self.make_tracker(dwell=0.0)
        tracker.start((0.0, 0.0), 0.0, 1.0)

        now = 1.0
        yaw = 0.0
        for index in range(4):
            self.assertEqual(
                tracker.update(yaw, now).kind,
                SCAN_HEADING_REACHED,
            )
            now += 0.01
            event = tracker.update(yaw, now)
            if index < 3:
                self.assertEqual(event.kind, SCAN_NEXT_HEADING)
                yaw = event.target_yaw
                now += 0.01

        self.assertEqual(event.kind, SCAN_COMPLETE)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.scanned_positions, [(0.0, 0.0)])

    def test_position_dedup_skips_nearby_position(self):
        tracker = self.make_tracker(dwell=0.0)
        tracker.scanned_positions.append((0.0, 0.0))

        event = tracker.start((0.5, 0.5), 0.0, 23.0)

        self.assertEqual(event.kind, SCAN_SKIPPED)
        self.assertFalse(tracker.active)

    def test_far_position_starts_new_scan(self):
        tracker = self.make_tracker(dwell=0.0)
        tracker.scanned_positions.append((0.0, 0.0))

        event = tracker.start((2.0, 0.0), 0.0, 23.0)

        self.assertEqual(event.kind, SCAN_STARTED)
        self.assertEqual(tracker.scanned_positions, [(0.0, 0.0)])

    def test_heading_timeout_retries_once_then_terminates(self):
        tracker = self.make_tracker(dwell=0.0)
        tracker.start((0.0, 0.0), 0.0, 1.0)

        event = tracker.update(math.pi, 3.1)
        self.assertEqual(event.kind, SCAN_RETRY_HEADING)
        self.assertEqual(event.retry_count, 1)

        event = tracker.update(math.pi, 5.2)
        self.assertEqual(event.kind, SCAN_HEADING_TIMEOUT)
        self.assertEqual(event.heading_index, 0)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.scanned_positions, [])

        retry = tracker.start((0.0, 0.0), 0.0, 5.3)
        self.assertEqual(retry.kind, SCAN_STARTED)

    def test_first_heading_timeout_is_bounded(self):
        tracker = self.make_tracker(
            yaw_step=math.pi / 2.0,
            dwell=0.0,
            max_heading_retries=0,
        )
        tracker.start((0.0, 0.0), 0.0, 1.0)

        event = tracker.update(0.7, 3.1)

        self.assertEqual(event.kind, SCAN_HEADING_TIMEOUT)
        self.assertEqual(event.heading_index, 0)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.scanned_positions, [])

        retry = tracker.start((0.0, 0.0), 0.0, 3.2)
        self.assertEqual(retry.kind, SCAN_STARTED)

    def test_whole_scan_timeout_terminates(self):
        tracker = self.make_tracker(scan_timeout=3.0)
        tracker.start((0.0, 0.0), 0.0, 1.0)
        event = tracker.update(math.pi, 4.1)
        self.assertEqual(event.kind, SCAN_TIMEOUT)
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.scanned_positions, [])

        retry = tracker.start((0.0, 0.0), 0.0, 4.2)
        self.assertEqual(retry.kind, SCAN_STARTED)

    def test_reset_clears_active_episode_and_coverage(self):
        tracker = self.make_tracker()
        tracker.start((0.0, 0.0), 0.0, 1.0)
        tracker.reset()
        self.assertFalse(tracker.active)
        self.assertEqual(tracker.scanned_positions, [])
        self.assertIsNone(tracker.current_heading)

    def test_invalid_parameters_are_rejected(self):
        with self.assertRaises(ValueError):
            self.make_tracker(yaw_step=math.pi)
        with self.assertRaises(ValueError):
            self.make_tracker(yaw_tolerance=0.0)
        with self.assertRaises(ValueError):
            self.make_tracker(position_dedup_radius=-1.0)


if __name__ == "__main__":
    unittest.main()
