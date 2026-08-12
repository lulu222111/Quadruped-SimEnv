#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from planning_core import (  # noqa: E402
    PLAN_NO_FRONTIER,
    PLAN_SUCCESS,
    PLAN_UNREACHABLE,
    plan_to_frontier_with_status,
)


class PlanToFrontierStatusTest(unittest.TestCase):
    def test_reports_success_when_frontier_is_reachable(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        result = plan_to_frontier_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=5,
        )

        self.assertEqual(result["status"], PLAN_SUCCESS)
        self.assertIsNotNone(result["goal_cell"])
        self.assertIsNotNone(result["path"])
        self.assertIsNotNone(result["cluster"])
        self.assertEqual(result["path"][0], (2, 2))
        self.assertEqual(result["path"][-1], result["goal_cell"])
        self.assertEqual(result["frontier_count"], 8)

    def test_reports_no_frontier_when_map_is_fully_known(self):
        result = plan_to_frontier_with_status(
            [0] * 25,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=1,
        )

        self.assertEqual(result["status"], PLAN_NO_FRONTIER)
        self.assertIsNone(result["goal_cell"])
        self.assertIsNone(result["path"])
        self.assertIsNone(result["cluster"])
        self.assertEqual(result["frontier_count"], 0)

    def test_reports_unreachable_when_robot_is_enclosed(self):
        data = [
            -1, -1, -1, -1, -1, -1, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1,  0, 100, 100, 100,  0, -1,
            -1,  0, 100,   0, 100,  0, -1,
            -1,  0, 100, 100, 100,  0, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1, -1, -1, -1, -1, -1, -1,
        ]

        result = plan_to_frontier_with_status(
            data,
            width=7,
            height=7,
            robot_cell=(3, 3),
            min_frontier_size=5,
        )

        self.assertEqual(result["status"], PLAN_UNREACHABLE)
        self.assertIsNone(result["goal_cell"])
        self.assertIsNone(result["path"])
        self.assertIsNone(result["cluster"])
        self.assertGreater(result["frontier_count"], 0)


if __name__ == "__main__":
    unittest.main()