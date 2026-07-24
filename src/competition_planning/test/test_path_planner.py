#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from path_planner import astar_path, inflate_obstacles  # noqa: E402


class AStarPathTest(unittest.TestCase):
    def test_finds_straight_path(self):
        data = [0] * 5

        path = astar_path(
            data,
            width=5,
            height=1,
            start=(0, 0),
            goal=(4, 0),
        )

        self.assertEqual(
            path,
            [(0, 0), (1, 0), (2, 0), (3, 0), (4, 0)],
        )

    def test_routes_around_obstacle(self):
        data = [
            0,   0, 0,
            0, 100, 0,
            0,   0, 0,
        ]

        path = astar_path(
            data,
            width=3,
            height=3,
            start=(0, 1),
            goal=(2, 1),
        )

        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 1))
        self.assertEqual(path[-1], (2, 1))
        self.assertNotIn((1, 1), path)
        self.assertEqual(len(path), 5)

    def test_returns_none_when_goal_is_unreachable(self):
        data = [
            0, 100, 0,
            0, 100, 0,
            0, 100, 0,
        ]

        path = astar_path(
            data,
            width=3,
            height=3,
            start=(0, 1),
            goal=(2, 1),
        )

        self.assertIsNone(path)

    def test_does_not_cross_unknown_cells(self):
        data = [0, -1, 0]

        path = astar_path(
            data,
            width=3,
            height=1,
            start=(0, 0),
            goal=(2, 0),
        )

        self.assertIsNone(path)

    def test_returns_single_cell_when_already_at_goal(self):
        path = astar_path(
            [0],
            width=1,
            height=1,
            start=(0, 0),
            goal=(0, 0),
        )

        self.assertEqual(path, [(0, 0)])

    def test_returns_none_when_start_or_goal_is_blocked(self):
        data = [100, 0, 100]

        self.assertIsNone(
            astar_path(
                data,
                width=3,
                height=1,
                start=(0, 0),
                goal=(1, 0),
            )
        )
        self.assertIsNone(
            astar_path(
                data,
                width=3,
                height=1,
                start=(1, 0),
                goal=(2, 0),
            )
        )

    def test_rejects_invalid_grid_length(self):
        with self.assertRaises(ValueError):
            astar_path(
                [0, 0, 0],
                width=2,
                height=2,
                start=(0, 0),
                goal=(1, 1),
            )

    def test_rejects_out_of_bounds_positions(self):
        data = [0] * 4

        with self.assertRaises(ValueError):
            astar_path(
                data,
                width=2,
                height=2,
                start=(-1, 0),
                goal=(1, 1),
            )

        with self.assertRaises(ValueError):
            astar_path(
                data,
                width=2,
                height=2,
                start=(0, 0),
                goal=(2, 1),
            )


class InflateObstaclesTest(unittest.TestCase):
    def test_inflates_obstacle_by_one_cell(self):
        data = [
            0, 0,   0,
            0, 100, 0,
            0, 0,   0,
        ]

        inflated = inflate_obstacles(
            data,
            width=3,
            height=3,
            radius_cells=1,
        )

        self.assertEqual(
            inflated,
            [
                0,   100, 0,
                100, 100, 100,
                0,   100, 0,
            ],
        )

    def test_zero_radius_keeps_original_shape(self):
        data = [
            0, 100,
            0,   0,
        ]

        inflated = inflate_obstacles(
            data,
            width=2,
            height=2,
            radius_cells=0,
        )

        self.assertEqual(inflated, data)
        self.assertIsNot(inflated, data)

    def test_handles_obstacle_at_map_edge(self):
        data = [
            100, 0,
            0,   0,
        ]

        inflated = inflate_obstacles(
            data,
            width=2,
            height=2,
            radius_cells=1,
        )

        self.assertEqual(
            inflated,
            [
                100, 100,
                100,   0,
            ],
        )

    def test_preserves_unknown_cells(self):
        data = [
            100, -1,
            0,    0,
        ]

        inflated = inflate_obstacles(
            data,
            width=2,
            height=2,
            radius_cells=1,
        )

        self.assertEqual(inflated[1], -1)

    def test_rejects_negative_radius(self):
        with self.assertRaises(ValueError):
            inflate_obstacles(
                [0],
                width=1,
                height=1,
                radius_cells=-1,
            )


if __name__ == "__main__":
    unittest.main()