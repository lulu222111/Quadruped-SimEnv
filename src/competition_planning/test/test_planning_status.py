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

    def test_inflation_can_make_a_frontier_unreachable(self):
        width = 9
        height = 7
        data = [0] * (width * height)

        data[3 * width + 8] = -1
        data[2 * width + 8] = 100
        data[4 * width + 8] = 100
        data[2 * width + 7] = 100

        without_inflation = plan_to_frontier_with_status(
            data,
            width=width,
            height=height,
            robot_cell=(1, 3),
            min_frontier_size=1,
            inflation_radius_cells=0,
        )
        with_inflation = plan_to_frontier_with_status(
            data,
            width=width,
            height=height,
            robot_cell=(1, 3),
            min_frontier_size=1,
            inflation_radius_cells=1,
        )

        self.assertEqual(without_inflation["status"], PLAN_SUCCESS)
        self.assertEqual(with_inflation["status"], PLAN_UNREACHABLE)
        self.assertEqual(with_inflation["cluster_count"], 1)
        self.assertGreater(with_inflation["reachable_cell_count"], 1)
        self.assertEqual(with_inflation["reachable_frontier_count"], 0)
        self.assertEqual(with_inflation["robot_cell"], (1, 3))
        self.assertEqual(with_inflation["inflation_radius_cells"], 1)

    def test_start_surrounded_by_inflated_ring_is_diagnosed(self):
        width = 9
        height = 9
        data = [-1] * (width * height)

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                data[y * width + x] = 0

        robot_cell = (4, 4)
        for obstacle_cell in ((4, 2), (6, 4), (4, 6), (2, 4)):
            x, y = obstacle_cell
            data[y * width + x] = 100

        result = plan_to_frontier_with_status(
            data,
            width=width,
            height=height,
            robot_cell=robot_cell,
            min_frontier_size=1,
            inflation_radius_cells=1,
        )

        self.assertEqual(result["status"], PLAN_UNREACHABLE)
        self.assertGreater(result["cluster_count"], 0)
        self.assertEqual(result["reachable_cell_count"], 1)
        self.assertEqual(result["reachable_frontier_count"], 0)
        self.assertEqual(result["robot_cell"], robot_cell)
        self.assertEqual(result["inflation_radius_cells"], 1)

    def test_free_robot_cell_does_not_use_start_recovery(self):
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
            min_frontier_size=1,
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], PLAN_SUCCESS)
        self.assertEqual(result["path"][0], (2, 2))
        self.assertEqual(result["robot_cell_raw_value"], 0)
        self.assertEqual(result["robot_cell_inflated_value"], 0)
        self.assertFalse(result["start_recovery_used"])
        self.assertIsNone(result["start_recovery_cell"])

    def test_raw_free_but_inflated_start_recovers_to_safe_cell(self):
        width = 9
        height = 9
        data = [-1] * (width * height)
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                data[y * width + x] = 0
        robot_cell = (4, 4)
        data[4 * width + 2] = 100
        result = plan_to_frontier_with_status(
            data,
            width=width,
            height=height,
            robot_cell=robot_cell,
            min_frontier_size=1,
            inflation_radius_cells=2,
            start_recovery_radius_cells=2,
        )
        self.assertEqual(result["robot_cell_raw_value"], 0)
        self.assertEqual(result["robot_cell_inflated_value"], 100)
        self.assertTrue(result["start_recovery_used"])
        self.assertIsNotNone(result["start_recovery_cell"])

    def test_unknown_robot_cell_recovers_to_nearest_free_cell(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0, -1,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]
        without_recovery = plan_to_frontier_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=1,
        )
        with_recovery = plan_to_frontier_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=1,
            start_recovery_radius_cells=1,
        )
        self.assertEqual(without_recovery["status"], PLAN_UNREACHABLE)
        self.assertFalse(without_recovery["start_recovery_used"])
        self.assertEqual(with_recovery["status"], PLAN_SUCCESS)
        self.assertEqual(with_recovery["robot_cell_raw_value"], -1)
        self.assertEqual(with_recovery["robot_cell_inflated_value"], -1)
        self.assertTrue(with_recovery["start_recovery_used"])
        self.assertEqual(with_recovery["start_recovery_cell"], (2, 1))
        self.assertEqual(with_recovery["path"][0], (2, 1))

    def test_occupied_robot_cell_does_not_use_start_recovery(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0, 100,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]
        result = plan_to_frontier_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            min_frontier_size=1,
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], PLAN_UNREACHABLE)
        self.assertEqual(result["robot_cell_raw_value"], 100)
        self.assertEqual(result["robot_cell_inflated_value"], 100)
        self.assertFalse(result["start_recovery_used"])
        self.assertIsNone(result["start_recovery_cell"])
        self.assertEqual(result["reachable_cell_count"], 0)

    def test_out_of_bounds_robot_cell_does_not_use_start_recovery(self):
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
            robot_cell=(5, 2),
            min_frontier_size=1,
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], PLAN_UNREACHABLE)
        self.assertIsNone(result["robot_cell_raw_value"])
        self.assertIsNone(result["robot_cell_inflated_value"])
        self.assertFalse(result["start_recovery_used"])
        self.assertIsNone(result["start_recovery_cell"])
        self.assertEqual(result["reachable_cell_count"], 0)

    def test_inflated_recovery_candidate_is_rejected(self):
        width = 7
        height = 7
        data = [-1] * (width * height)
        data[2 * width + 3] = 0
        data[1 * width + 3] = 100
        result = plan_to_frontier_with_status(
            data,
            width=width,
            height=height,
            robot_cell=(3, 3),
            min_frontier_size=1,
            inflation_radius_cells=1,
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], PLAN_UNREACHABLE)
        self.assertEqual(result["robot_cell_raw_value"], -1)
        self.assertEqual(result["robot_cell_inflated_value"], -1)
        self.assertFalse(result["start_recovery_used"])
        self.assertIsNone(result["start_recovery_cell"])
        self.assertEqual(result["reachable_cell_count"], 0)

    def test_negative_start_recovery_radius_is_rejected(self):
        data = [
            -1, -1, -1,
            -1,  0, -1,
            -1, -1, -1,
        ]
        with self.assertRaisesRegex(
            ValueError,
            "start_recovery_radius_cells must not be negative",
        ):
            plan_to_frontier_with_status(
                data,
                width=3,
                height=3,
                robot_cell=(1, 1),
                min_frontier_size=1,
                start_recovery_radius_cells=-1,
            )


if __name__ == "__main__":
    unittest.main()
