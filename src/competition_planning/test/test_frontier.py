#!/usr/bin/env python3

import sys
import unittest
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS_DIR))

from frontier import (  # noqa: E402
    cluster_frontier_cells,
    extract_frontier_cells,
    grid_cell_to_world,
    select_frontier_representative,
)


class ExtractFrontierCellsTest(unittest.TestCase):
    def test_extracts_free_cells_next_to_unknown(self):
        # 5×5 map:
        # Unknown cells surround a 3×3 free area.
        # The eight outside free cells are frontiers; the center is not.
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        frontiers = extract_frontier_cells(data, width=5, height=5)

        self.assertEqual(
            set(frontiers),
            {
                (1, 1), (2, 1), (3, 1),
                (1, 2),         (3, 2),
                (1, 3), (2, 3), (3, 3),
            },
        )

    def test_returns_empty_when_map_has_no_unknown_cells(self):
        data = [0] * 9

        frontiers = extract_frontier_cells(data, width=3, height=3)

        self.assertEqual(frontiers, [])

    def test_does_not_treat_obstacle_as_frontier(self):
        data = [
            -1, -1, -1,
            -1, 100, -1,
            -1, -1, -1,
        ]

        frontiers = extract_frontier_cells(data, width=3, height=3)

        self.assertEqual(frontiers, [])

    def test_rejects_invalid_grid_length(self):
        with self.assertRaises(ValueError):
            extract_frontier_cells([0, 0, 0], width=2, height=2)


class ClusterFrontierCellsTest(unittest.TestCase):
    def test_groups_adjacent_cells(self):
        cells = [(1, 1), (2, 1), (2, 2)]

        clusters = cluster_frontier_cells(cells)

        self.assertEqual(clusters, [[(1, 1), (2, 1), (2, 2)]])

    def test_uses_eight_connected_neighbors(self):
        cells = [(1, 1), (2, 2)]

        clusters = cluster_frontier_cells(cells)

        self.assertEqual(clusters, [[(1, 1), (2, 2)]])

    def test_keeps_separate_regions_separate(self):
        cells = [(1, 1), (2, 1), (10, 10), (11, 10)]

        clusters = cluster_frontier_cells(cells)

        self.assertEqual(
            clusters,
            [[(1, 1), (2, 1)], [(10, 10), (11, 10)]],
        )

    def test_filters_clusters_smaller_than_minimum(self):
        cells = [(1, 1), (2, 1), (10, 10)]

        clusters = cluster_frontier_cells(cells, min_size=2)

        self.assertEqual(clusters, [[(1, 1), (2, 1)]])

    def test_rejects_invalid_minimum_size(self):
        with self.assertRaises(ValueError):
            cluster_frontier_cells([], min_size=0)


class SelectFrontierRepresentativeTest(unittest.TestCase):
    def test_selects_cell_nearest_cluster_center(self):
        cluster = [(0, 0), (1, 0), (2, 0)]

        representative = select_frontier_representative(cluster)

        self.assertEqual(representative, (1, 0))

    def test_returns_the_only_cell(self):
        representative = select_frontier_representative([(4, 7)])

        self.assertEqual(representative, (4, 7))

    def test_result_is_an_actual_frontier_cell(self):
        cluster = [(0, 0), (2, 0), (2, 2)]

        representative = select_frontier_representative(cluster)

        self.assertIn(representative, cluster)

    def test_breaks_ties_deterministically(self):
        cluster = [(2, 0), (0, 0)]

        representative = select_frontier_representative(cluster)

        self.assertEqual(representative, (0, 0))

    def test_rejects_empty_cluster(self):
        with self.assertRaises(ValueError):
            select_frontier_representative([])


class GridCellToWorldTest(unittest.TestCase):
    def test_uses_cell_center(self):
        world_x, world_y = grid_cell_to_world(
            (0, 0),
            resolution=1.0,
        )

        self.assertAlmostEqual(world_x, 0.5)
        self.assertAlmostEqual(world_y, 0.5)

    def test_applies_map_resolution(self):
        world_x, world_y = grid_cell_to_world(
            (4, 7),
            resolution=0.5,
        )

        self.assertAlmostEqual(world_x, 2.25)
        self.assertAlmostEqual(world_y, 3.75)

    def test_applies_map_origin(self):
        world_x, world_y = grid_cell_to_world(
            (1, 2),
            resolution=1.0,
            origin_x=10.0,
            origin_y=-3.0,
        )

        self.assertAlmostEqual(world_x, 11.5)
        self.assertAlmostEqual(world_y, -0.5)

    def test_applies_map_origin_rotation(self):
        from math import pi

        world_x, world_y = grid_cell_to_world(
            (0, 0),
            resolution=2.0,
            origin_x=10.0,
            origin_y=20.0,
            origin_yaw=pi / 2.0,
        )

        self.assertAlmostEqual(world_x, 9.0)
        self.assertAlmostEqual(world_y, 21.0)

    def test_rejects_non_positive_resolution(self):
        with self.assertRaises(ValueError):
            grid_cell_to_world((0, 0), resolution=0.0)

if __name__ == "__main__":
    unittest.main()