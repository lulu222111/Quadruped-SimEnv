#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from planning_core import plan_to_frontier  # noqa: E402


class PlanToFrontierTest(unittest.TestCase):
    def test_plans_from_robot_to_frontier(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        result = plan_to_frontier(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=5,
        )

        self.assertIsNotNone(result)
        self.assertEqual(result["path"][0], (2, 2))
        self.assertEqual(result["path"][-1], result["goal_cell"])
        self.assertIn(result["goal_cell"], result["cluster"])
        self.assertEqual(result["frontier_count"], 8)

    def test_returns_none_when_map_has_no_frontier(self):
        result = plan_to_frontier(
            [0] * 9,
            width=3,
            height=3,
            robot_cell=(1, 1),
            min_frontier_size=1,
        )

        self.assertIsNone(result)

    def test_filters_frontiers_below_minimum_size(self):
        data = [
            -1, 0, 0,
             0, 0, 0,
             0, 0, 0,
        ]

        result = plan_to_frontier(
            data,
            width=3,
            height=3,
            robot_cell=(2, 2),
            min_frontier_size=3,
        )

        self.assertIsNone(result)

    def test_path_contains_only_free_cells(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0, 100, 0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        result = plan_to_frontier(
            data,
            width=5,
            height=5,
            robot_cell=(1, 2),
            min_frontier_size=1,
        )

        self.assertIsNotNone(result)

        for x, y in result["path"]:
            self.assertEqual(data[y * 5 + x], 0)


if __name__ == "__main__":
    unittest.main()