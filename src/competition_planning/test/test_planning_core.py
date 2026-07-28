#!/usr/bin/env python3

import statistics
import sys
import time
import unittest
from unittest.mock import patch
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

    def test_tries_another_candidate_in_the_same_cluster(self):
        cluster = [
            (2, 1),
            (1, 1),
            (3, 1),
        ]
        expected_path = [
            (0, 1),
            (1, 1),
        ]

        with patch(
            "planning_core.extract_frontier_cells",
            return_value=list(cluster),
        ):
            with patch(
                "planning_core.cluster_frontier_cells",
                return_value=[cluster],
            ):
                with patch(
                    "planning_core.inflate_obstacles",
                    return_value=[0] * 12,
                ):
                    with patch(
                        "planning_core.astar_path",
                        side_effect=[
                            None,
                            expected_path,
                        ],
                    ) as astar_mock:
                        result = plan_to_frontier(
                            [0] * 12,
                            width=4,
                            height=3,
                            robot_cell=(0, 1),
                            min_frontier_size=1,
                        )

        self.assertIsNotNone(result)
        self.assertEqual(result["goal_cell"], (1, 1))
        self.assertEqual(result["path"], expected_path)
        self.assertEqual(result["cluster"], cluster)

        self.assertEqual(astar_mock.call_count, 2)
        self.assertEqual(
            astar_mock.call_args_list[0].kwargs["goal"],
            (2, 1),
        )
        self.assertEqual(
            astar_mock.call_args_list[1].kwargs["goal"],
            (1, 1),
        )

    def test_planning_time_is_below_500_ms_on_20_by_36_m_map(self):
        resolution = 0.1
        width = int(20.0 / resolution)
        height = int(36.0 / resolution)
        unknown_border = 10

        data = [-1] * (width * height)

        for y in range(
            unknown_border,
            height - unknown_border,
        ):
            row_start = y * width

            for x in range(
                unknown_border,
                width - unknown_border,
            ):
                data[row_start + x] = 0

        robot_cell = (
            width // 2,
            height // 2,
        )

        def run_planner():
            return plan_to_frontier(
                data,
                width=width,
                height=height,
                robot_cell=robot_cell,
                min_frontier_size=5,
                inflation_radius_cells=2,
            )

        warmup_result = run_planner()
        self.assertIsNotNone(warmup_result)

        durations = []

        for _ in range(5):
            start_time = time.perf_counter()
            result = run_planner()
            durations.append(
                time.perf_counter() - start_time
            )

            self.assertIsNotNone(result)
            self.assertEqual(
                result["path"][0],
                robot_cell,
            )
            self.assertEqual(
                result["path"][-1],
                result["goal_cell"],
            )

            for x, y in result["path"]:
                self.assertEqual(
                    data[y * width + x],
                    0,
                )

        median_seconds = statistics.median(durations)
        median_ms = median_seconds * 1000.0

        self.assertLess(
            median_seconds,
            0.5,
            msg=(
                "Median planning time was "
                f"{median_ms:.1f} ms; expected less than "
                "500 ms. "
                f"All runs: "
                f"{[round(value * 1000.0, 1) for value in durations]} "
                "ms"
            ),
        )


if __name__ == "__main__":
    unittest.main()