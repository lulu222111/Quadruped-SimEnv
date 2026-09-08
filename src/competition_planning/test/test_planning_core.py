#!/usr/bin/env python3

import statistics
import sys
import time
import unittest
from pathlib import Path
from unittest.mock import patch


SCRIPTS_DIR = (
    Path(__file__).resolve().parents[1]
    / "scripts"
)
sys.path.insert(0, str(SCRIPTS_DIR))

from planning_core import (  # noqa: E402
    _bfs_reachability,
    _nearest_reachable_standoff,
    compress_collinear_path,
    order_frontier_candidates,
    path_cell_yaws,
    plan_to_goal_with_status,
    plan_to_goal_region_with_status,
    plan_to_heading_goal_with_status,
    plan_to_frontier,
    plan_to_frontier_with_status,
)


class FrontierCandidateHeadingTest(unittest.TestCase):
    def test_optional_heading_orders_forward_cell_first(self):
        cluster = [(-1, 0), (0, 2), (4, 0)]

        ordered = order_frontier_candidates(
            cluster,
            robot_cell=(0, 0),
            heading_yaw=0.0,
            heading_weight=5.0,
        )

        self.assertEqual(ordered[0], (4, 0))

    def test_robot_cell_is_not_returned_as_frontier_goal(self):
        ordered = order_frontier_candidates(
            [(0, 0), (1, 0)],
            robot_cell=(0, 0),
            heading_yaw=0.0,
            heading_weight=1.0,
        )

        self.assertEqual(ordered, [(1, 0)])


class StandoffProgressContractTest(unittest.TestCase):
    def test_skips_controller_equivalent_standoff_candidates(self):
        width = 7
        height = 7
        parent = _bfs_reachability(
            [0] * (width * height),
            width,
            height,
            start=(3, 3),
        )

        nearest = _nearest_reachable_standoff(
            parent,
            frontier_cell=(3, 4),
            radius_cells=2,
            planning_start=(3, 3),
        )
        progressed = _nearest_reachable_standoff(
            parent,
            frontier_cell=(3, 4),
            radius_cells=2,
            planning_start=(3, 3),
            minimum_path_length_cells=3,
        )

        self.assertEqual(nearest, (2, 4))
        self.assertEqual(progressed, (1, 4))
        self.assertGreaterEqual(
            len(parent),
            1,
        )


class PathCompressionTest(unittest.TestCase):
    def test_keeps_start_turns_and_goal(self):
        path = [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2), (3, 2)]
        self.assertEqual(
            compress_collinear_path(path),
            [(0, 0), (2, 0), (2, 2), (3, 2)],
        )

    def test_does_not_change_short_paths(self):
        self.assertEqual(compress_collinear_path([]), [])
        self.assertEqual(compress_collinear_path([(1, 2)]), [(1, 2)])


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
        self.assertEqual(
            result["path"][0],
            (2, 2),
        )
        self.assertEqual(
            result["path"][-1],
            result["goal_cell"],
        )
        self.assertIn(
            result["goal_cell"],
            result["cluster"],
        )
        self.assertEqual(
            result["frontier_count"],
            8,
        )

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
            self.assertEqual(
                data[y * 5 + x],
                0,
            )

    def test_tries_another_candidate_in_the_same_cluster(
        self,
    ):
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
                        "planning_core._bfs_reachability",
                        return_value={
                            (0, 1): None,
                            (1, 1): (0, 1),
                            (3, 1): (0, 1),
                        },
                    ) as bfs_mock:
                        result = plan_to_frontier(
                            [0] * 12,
                            width=4,
                            height=3,
                            robot_cell=(0, 1),
                            min_frontier_size=1,
                        )

        self.assertIsNotNone(result)
        self.assertEqual(
            result["goal_cell"],
            (1, 1),
        )
        self.assertEqual(
            result["path"],
            expected_path,
        )
        self.assertEqual(
            result["cluster"],
            cluster,
        )

        self.assertEqual(
            bfs_mock.call_count,
            1,
        )

    def test_preferred_frontier_cell_limits_selection_to_its_cluster(
        self,
    ):
        preferred_cluster = [(3, 1), (3, 2), (3, 3)]
        nearby_cluster = [(1, 1), (1, 2), (1, 3)]

        with patch(
            "planning_core.extract_frontier_cells",
            return_value=preferred_cluster + nearby_cluster,
        ):
            with patch(
                "planning_core.cluster_frontier_cells",
                return_value=[nearby_cluster, preferred_cluster],
            ):
                with patch(
                    "planning_core.inflate_obstacles",
                    return_value=[0] * 20,
                ):
                    with patch(
                        "planning_core._bfs_reachability",
                        return_value={
                            (0, 2): None,
                            (1, 1): (0, 2),
                            (1, 2): (0, 2),
                            (1, 3): (0, 2),
                            (3, 1): (1, 1),
                            (3, 2): (1, 2),
                            (3, 3): (1, 3),
                        },
                    ):
                        result = plan_to_frontier_with_status(
                            [0] * 20,
                            width=5,
                            height=4,
                            robot_cell=(0, 2),
                            min_frontier_size=1,
                            preferred_frontier_cell=(3, 2),
                        )

        self.assertEqual(result["status"], "success")
        self.assertEqual(result["cluster"], preferred_cluster)
        self.assertIn(result["goal_cell"], preferred_cluster)

    def test_exclude_frontier_cells_skips_cooldowned_cluster(self):
        cluster_a = [(3, 1), (3, 2), (3, 3), (3, 4), (3, 5)]
        cluster_b = [(1, 1), (1, 2), (1, 3)]
        parents = {
            (0, 2): None,
            (1, 1): (0, 2),
            (1, 2): (0, 2),
            (1, 3): (0, 2),
            (3, 1): (2, 2),
            (3, 2): (2, 2),
            (3, 3): (2, 2),
            (3, 4): (2, 2),
            (3, 5): (2, 2),
            (2, 2): (1, 2),
        }
        with patch(
            "planning_core.extract_frontier_cells",
            return_value=cluster_a + cluster_b,
        ):
            with patch(
                "planning_core.cluster_frontier_cells",
                return_value=[cluster_a, cluster_b],
            ):
                with patch(
                    "planning_core.inflate_obstacles",
                    return_value=[0] * 30,
                ):
                    with patch(
                        "planning_core._bfs_reachability",
                        return_value=parents,
                    ):
                        # Without exclusion the larger cluster_a wins.
                        picked = plan_to_frontier_with_status(
                            [0] * 30,
                            width=6,
                            height=5,
                            robot_cell=(0, 2),
                            min_frontier_size=1,
                            size_weight=3.0,
                            distance_weight=1.0,
                        )
                        self.assertEqual(picked["cluster"], cluster_a)
                        # With cluster_a excluded only cluster_b remains.
                        picked = plan_to_frontier_with_status(
                            [0] * 30,
                            width=6,
                            height=5,
                            robot_cell=(0, 2),
                            min_frontier_size=1,
                            size_weight=3.0,
                            distance_weight=1.0,
                            exclude_frontier_cells=set(cluster_a),
                        )
                        self.assertEqual(picked["status"], "success")
                        self.assertEqual(picked["cluster"], cluster_b)

    def test_clears_inflation_from_free_robot_start_cell(
        self,
    ):
        data = [
            -1, -1, -1, -1, -1, -1, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1,  0,  0, 100,  0,  0, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1,  0,  0,  0,  0,  0, -1,
            -1, -1, -1, -1, -1, -1, -1,
        ]
        robot_cell = (3, 3)

        result = plan_to_frontier(
            data,
            width=7,
            height=7,
            robot_cell=robot_cell,
            min_frontier_size=1,
            inflation_radius_cells=1,
        )

        self.assertEqual(
            data[
                robot_cell[1] * 7
                + robot_cell[0]
            ],
            0,
        )
        self.assertIsNotNone(result)
        self.assertEqual(
            result["path"][0],
            robot_cell,
        )

    def test_path_yaws_follow_path_segments(self):
        yaws = path_cell_yaws(
            [
                (1, 1),
                (2, 1),
                (2, 2),
            ],
            origin_yaw=0.0,
        )

        self.assertAlmostEqual(
            yaws[0],
            0.0,
        )
        self.assertAlmostEqual(
            yaws[1],
            1.5707963267948966,
        )
        self.assertAlmostEqual(
            yaws[2],
            yaws[1],
        )

    def test_single_cell_path_uses_map_origin_yaw(self):
        yaws = path_cell_yaws(
            [(2, 3)],
            origin_yaw=0.75,
        )

        self.assertEqual(
            yaws,
            [0.75],
        )

    def test_planning_time_is_below_500_ms_on_20_by_36_m_map(
        self,
    ):
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
                time.perf_counter()
                - start_time
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

        median_seconds = statistics.median(
            durations
        )
        median_ms = median_seconds * 1000.0

        self.assertLess(
            median_seconds,
            0.5,
            msg=(
                "Median planning time was "
                f"{median_ms:.1f} ms; expected "
                "less than 500 ms. "
                f"All runs: "
                f"{[round(value * 1000.0, 1) for value in durations]} "
                "ms"
            ),
        )

    # -- BFS reachability tests ----------------------------------------

    def test_representative_unreachable_falls_back_with_real_grid(
        self,
    ):
        data = [
            -1, -1, -1, -1, -1, -1, -1,
            -1,  0,  0, 100,  0,  0, -1,
            -1,  0,  0, 100,  0,  0, -1,
            -1,  0,  0, 100,  0,  0, -1,
            -1,  0,  0, 100,  0,  0, -1,
            -1,  0,  0,   0,  0,  0, -1,
            -1, -1, -1, -1, -1, -1, -1,
        ]
        robot_cell = (1, 3)

        result = plan_to_frontier(
            data,
            width=7,
            height=7,
            robot_cell=robot_cell,
            min_frontier_size=5,
        )

        self.assertIsNotNone(result)
        self.assertIn(
            result["goal_cell"],
            result["cluster"],
        )
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
                data[y * 7 + x],
                0,
            )

    def test_single_bfs_call_per_plan(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        with patch(
            "planning_core._bfs_reachability",
            wraps=_bfs_reachability,
        ) as bfs_mock:
            plan_to_frontier(
                data,
                width=5,
                height=5,
                robot_cell=(2, 2),
                min_frontier_size=5,
            )

        self.assertEqual(
            bfs_mock.call_count,
            1,
        )

    def test_reconstructed_path_only_traversable_cells(
        self,
    ):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0, 100,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        result = plan_to_frontier(
            data,
            width=5,
            height=5,
            robot_cell=(1, 1),
            min_frontier_size=1,
        )

        self.assertIsNotNone(result)

        for x, y in result["path"]:
            index = y * 5 + x
            self.assertGreaterEqual(data[index], 0)
            self.assertLess(data[index], 50)

    def test_start_equals_goal(self):
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        with patch(
            "planning_core.extract_frontier_cells",
            return_value=[(2, 2)],
        ):
            with patch(
                "planning_core.cluster_frontier_cells",
                return_value=[[(2, 2)]],
            ):
                with patch(
                    "planning_core.inflate_obstacles",
                    return_value=list(data),
                ):
                    result = plan_to_frontier(
                        data,
                        width=5,
                        height=5,
                        robot_cell=(2, 2),
                        min_frontier_size=1,
                    )

        self.assertIsNotNone(result)
        self.assertEqual(
            result["goal_cell"],
            (2, 2),
        )
        self.assertEqual(
            result["path"],
            [(2, 2)],
        )

    def test_bfs_rejects_out_of_bounds_start(self):
        parent = _bfs_reachability(
            [0] * 9,
            width=3,
            height=3,
            start=(5, 5),
        )

        self.assertEqual(parent, {})

    def test_bfs_rejects_blocked_start(self):
        parent = _bfs_reachability(
            [100, 0, 0, 0, 0, 0, 0, 0, 0],
            width=3,
            height=3,
            start=(0, 0),
        )

        self.assertEqual(parent, {})

    def test_bfs_rejects_invalid_grid_length(self):
        with self.assertRaises(IndexError):
            _bfs_reachability(
                [0, 0, 0],
                width=2,
                height=2,
                start=(0, 0),
            )

    def test_large_unreachable_cluster_performance(
        self,
    ):
        width, height = 400, 400
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

        wall_x = width // 2
        gap_y = height // 2
        for y in range(
            unknown_border,
            height - unknown_border,
        ):
            if y == gap_y:
                continue
            data[y * width + wall_x] = 100

        robot_cell = (
            wall_x - 10,
            height // 2,
        )

        frontier_cells = []
        for y in range(
            unknown_border,
            height - unknown_border,
        ):
            row_start = y * width
            for x in range(
                unknown_border,
                width - unknown_border,
            ):
                if data[row_start + x] != 0:
                    continue
                if y > 0 and data[
                    row_start - width + x
                ] == -1:
                    frontier_cells.append((x, y))
                    continue
                if (
                    y < height - 1
                    and data[
                        row_start + width + x
                    ]
                    == -1
                ):
                    frontier_cells.append((x, y))
                    continue
                if x > 0 and data[
                    row_start + x - 1
                ] == -1:
                    frontier_cells.append((x, y))
                    continue
                if (
                    x < width - 1
                    and data[
                        row_start + x + 1
                    ]
                    == -1
                ):
                    frontier_cells.append((x, y))
                    continue

        self.assertGreater(
            len(frontier_cells),
            500,
            "Test precondition: need a large frontier set",
        )

        with patch(
            "planning_core.extract_frontier_cells",
            return_value=frontier_cells,
        ):
            start_time = time.perf_counter()
            result = plan_to_frontier(
                data,
                width=width,
                height=height,
                robot_cell=robot_cell,
                min_frontier_size=5,
                inflation_radius_cells=2,
            )
            elapsed = time.perf_counter() - start_time

        self.assertIsNotNone(result)
        self.assertEqual(
            result["path"][0],
            robot_cell,
        )
        self.assertLess(
            elapsed,
            1.0,
            msg=(
                "Planning with large unreachable cluster took "
                f"{elapsed * 1000:.1f} ms; "
                "expected well under 1.0 s"
            ),
        )


class PlanToReturnGoalTest(unittest.TestCase):
    def test_returns_path_to_free_start_goal(self):
        data = [
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
            0, 0, 0, 0, 0,
        ]
        result = plan_to_goal_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(4, 4),
            goal_cell=(0, 0),
        )
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["path"][0], (4, 4))
        self.assertEqual(result["path"][-1], (0, 0))

    def test_occupied_goal_is_unreachable(self):
        data = [0] * 9
        data[0] = 100
        result = plan_to_goal_with_status(
            data,
            width=3,
            height=3,
            robot_cell=(2, 2),
            goal_cell=(0, 0),
        )
        self.assertEqual(result["status"], "unreachable")
        self.assertIn("occupied", result["detail"])

    def test_unknown_start_uses_finite_recovery_radius(self):
        data = [
            0, 0, 0,
            0, -1, 0,
            0, 0, 0,
        ]
        result = plan_to_goal_with_status(
            data,
            width=3,
            height=3,
            robot_cell=(1, 1),
            goal_cell=(0, 0),
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], "success")
        self.assertTrue(result["start_recovery_used"])
        self.assertEqual(result["path"][-1], (0, 0))

    def test_occupied_start_does_not_recover(self):
        data = [0] * 9
        data[4] = 100
        result = plan_to_goal_with_status(
            data,
            width=3,
            height=3,
            robot_cell=(1, 1),
            goal_cell=(0, 0),
            start_recovery_radius_cells=1,
        )
        self.assertEqual(result["status"], "unreachable")

    def test_out_of_bounds_goal_is_unreachable(self):
        result = plan_to_goal_with_status(
            [0] * 9,
            width=3,
            height=3,
            robot_cell=(1, 1),
            goal_cell=(3, 3),
        )
        self.assertEqual(result["status"], "unreachable")
        self.assertIn("out of bounds", result["detail"])

    def test_region_uses_nearest_reachable_cell_when_exact_goal_is_occupied(self):
        data = [0] * 25
        data[2 * 5 + 2] = 100
        result = plan_to_goal_region_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(4, 4),
            goal_cell=(2, 2),
            goal_search_radius_cells=1,
        )
        self.assertEqual(result["status"], "success")
        self.assertTrue(result["goal_offset_used"])
        self.assertEqual(result["requested_goal_cell"], (2, 2))


class PlanToHeadingGoalTest(unittest.TestCase):
    def test_selects_furthest_reachable_cell_around_scattered_obstacle(self):
        data = [0] * 49
        data[3 * 7 + 3] = 100
        result = plan_to_heading_goal_with_status(
            data,
            width=7,
            height=7,
            robot_cell=(1, 3),
            heading_yaw=0.0,
            max_forward_cells=4.0,
            min_forward_cells=1.0,
            corridor_half_width_cells=2.0,
        )
        self.assertEqual(result["status"], "success")
        self.assertEqual(result["goal_cell"][0], 5)
        self.assertNotIn((3, 3), result["path"])

    def test_fixed_heading_reference_keeps_replans_on_anchor_corridor(self):
        data = [0] * 144

        local_result = plan_to_heading_goal_with_status(
            data,
            width=12,
            height=12,
            robot_cell=(2, 5),
            heading_yaw=0.0,
            max_forward_cells=4.0,
            min_forward_cells=1.0,
            corridor_half_width_cells=1.0,
        )
        anchored_result = plan_to_heading_goal_with_status(
            data,
            width=12,
            height=12,
            robot_cell=(2, 5),
            heading_yaw=0.0,
            max_forward_cells=4.0,
            min_forward_cells=1.0,
            corridor_half_width_cells=1.0,
            heading_reference_cell=(2, 2),
        )

        self.assertEqual(local_result["status"], "success")
        self.assertEqual(anchored_result["status"], "success")
        self.assertEqual(local_result["goal_cell"], (6, 5))
        self.assertEqual(anchored_result["goal_cell"], (6, 2))
        self.assertEqual(
            anchored_result["heading_reference_cell"],
            (2, 2),
        )

    def test_reports_unreachable_when_forward_corridor_is_unknown(self):
        data = [-1] * 25
        data[2 * 5 + 2] = 0
        result = plan_to_heading_goal_with_status(
            data,
            width=5,
            height=5,
            robot_cell=(2, 2),
            heading_yaw=0.0,
            max_forward_cells=2.0,
        )
        self.assertEqual(result["status"], "unreachable")


if __name__ == "__main__":
    unittest.main()
