#!/usr/bin/env python3
import math
import os
import sys
import unittest


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(PACKAGE_ROOT, "src"))

from competition_mapping.occupancy_grid_core import OccupancyGridCore, transform_point_3d


class OccupancyGridCoreTest(unittest.TestCase):
    def make_grid(self, **overrides):
        params = dict(
            resolution=1.0, width=10, height=10,
            origin_x=-5.0, origin_y=-5.0,
            min_range=0.0, max_range=20.0,
            min_height=-1.0, max_height=2.0,
        )
        params.update(overrides)
        return OccupancyGridCore(**params)

    def test_initial_map_is_unknown_and_copy_is_independent(self):
        grid = self.make_grid()
        data = grid.data_copy()
        self.assertEqual(100, len(data))
        self.assertTrue(all(value == -1 for value in data))
        data[0] = 100
        self.assertEqual(-1, grid.get_cell(0, 0))

    def test_world_to_grid_uses_origin_and_floor(self):
        grid = self.make_grid(resolution=0.5)
        self.assertEqual((0, 0), grid.world_to_grid(-5.0, -5.0))
        self.assertEqual((1, 1), grid.world_to_grid(-4.5, -4.5))
        self.assertIsNone(grid.world_to_grid(-5.01, 0.0))
        self.assertIsNone(grid.world_to_grid(float("nan"), 0.0))

    def test_ray_marks_free_cells_and_occupied_endpoint(self):
        grid = self.make_grid()
        stats = grid.integrate_scan(0.0, 0.0, 0.0, [(3.0, 0.0, 0.5)])
        self.assertEqual(1, stats["valid_points"])
        self.assertEqual(0, grid.get_cell(5, 5))
        self.assertEqual(0, grid.get_cell(6, 5))
        self.assertEqual(0, grid.get_cell(7, 5))
        self.assertEqual(100, grid.get_cell(8, 5))
        self.assertEqual(-1, grid.get_cell(5, 6))

    def test_planar_pose_rotation_and_translation(self):
        grid = self.make_grid()
        grid.integrate_scan(1.0, 1.0, math.pi / 2.0, [(2.0, 0.0, 0.0)])
        self.assertEqual(100, grid.get_cell(6, 8))

    def test_new_free_ray_clears_stale_obstacle(self):
        grid = self.make_grid()
        grid.integrate_scan(0.0, 0.0, 0.0, [(2.0, 0.0, 0.0)])
        self.assertEqual(100, grid.get_cell(7, 5))
        grid.integrate_scan(0.0, 0.0, 0.0, [(3.0, 0.0, 0.0)])
        self.assertEqual(0, grid.get_cell(7, 5))
        self.assertEqual(100, grid.get_cell(8, 5))

    def test_outside_endpoint_clears_only_to_map_edge(self):
        grid = self.make_grid(max_range=100.0)
        stats = grid.integrate_scan(0.0, 0.0, 0.0, [(50.0, 0.0, 0.0)])
        self.assertEqual(1, stats["valid_points"])
        self.assertEqual(0, stats["occupied_endpoints"])
        self.assertEqual(0, grid.get_cell(9, 5))

    def test_filter_rejects_range_height_and_nonfinite_points(self):
        grid = self.make_grid(min_range=1.0, max_range=4.0, min_height=0.0, max_height=1.0)
        points = [
            (0.5, 0.0, 0.5), (5.0, 0.0, 0.5), (2.0, 0.0, -0.1),
            (2.0, 0.0, 1.1), (float("nan"), 0.0, 0.5), (2.0, 0.0, 0.5),
        ]
        stats = grid.integrate_scan(0.0, 0.0, 0.0, points)
        self.assertEqual(6, stats["input_points"])
        self.assertEqual(6, stats["total_points"])
        self.assertEqual(1, stats["valid_points"])
        self.assertEqual(5, stats["rejected_points"])
        self.assertEqual(1, stats["non_finite_rejected"])
        self.assertEqual(2, stats["range_rejected"])
        self.assertEqual(2, stats["height_rejected"])

    def test_occupied_endpoint_wins_over_crossing_free_ray_in_same_scan(self):
        grid = self.make_grid()
        # The second ray crosses the endpoint of the first ray at (7, 5).
        grid.integrate_scan(0.0, 0.0, 0.0, [(2.0, 0.0, 0.0), (4.0, 0.0, 0.0)])
        self.assertEqual(100, grid.get_cell(7, 5))
        self.assertEqual(100, grid.get_cell(9, 5))

    def test_empty_scan_does_not_change_map(self):
        grid = self.make_grid()
        before = grid.data_copy()
        stats = grid.integrate_scan(0.0, 0.0, 0.0, [])
        self.assertEqual(before, grid.data_copy())
        self.assertEqual(0, stats["input_points"])

    def test_robot_outside_map_does_not_change_map(self):
        grid = self.make_grid()
        before = grid.data_copy()
        stats = grid.integrate_scan(100.0, 100.0, 0.0, [(1.0, 0.0, 0.0)])
        self.assertEqual(1, stats["robot_outside_map"])
        self.assertEqual(before, grid.data_copy())

    def test_invalid_parameters_and_pose_raise(self):
        with self.assertRaises(ValueError):
            self.make_grid(resolution=0.0)
        grid = self.make_grid()
        with self.assertRaises(ValueError):
            grid.integrate_scan(float("inf"), 0.0, 0.0, [])

    def test_3d_transform_applies_translation(self):
        result = transform_point_3d((1.0, 2.0, 3.0), (4.0, -2.0, 0.5), (0, 0, 0, 1))
        self.assertEqual((5.0, 0.0, 3.5), result)

    def test_3d_transform_applies_quaternion_rotation(self):
        half = math.sqrt(0.5)
        x, y, z = transform_point_3d((1.0, 0.0, 0.0), (0, 0, 0), (0, 0, half, half))
        self.assertAlmostEqual(0.0, x, places=7)
        self.assertAlmostEqual(1.0, y, places=7)
        self.assertAlmostEqual(0.0, z, places=7)


if __name__ == "__main__":
    unittest.main(verbosity=2)
