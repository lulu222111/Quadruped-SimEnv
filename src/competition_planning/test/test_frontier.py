#!/usr/bin/env python3

import os
import sys
import unittest


PACKAGE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(PACKAGE_DIR, "scripts")
sys.path.insert(0, SCRIPTS_DIR)

from frontier import extract_frontier_cells


class ExtractFrontierCellsTest(unittest.TestCase):
    def test_extracts_free_cells_next_to_unknown(self):
        width = 5
        height = 5
        data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0, 100,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]

        result = set(extract_frontier_cells(data, width, height))

        expected = {
            (1, 1), (2, 1), (3, 1),
            (1, 2),         (3, 2),
            (1, 3), (2, 3), (3, 3),
        }
        self.assertEqual(result, expected)

    def test_returns_empty_when_map_has_no_unknown_cells(self):
        data = [
            0, 0, 0,
            0, 0, 0,
            0, 0, 0,
        ]

        result = extract_frontier_cells(data, width=3, height=3)

        self.assertEqual(result, [])

    def test_does_not_treat_obstacle_as_frontier(self):
        data = [
            -1, -1, -1,
            -1, 100, -1,
            -1, -1, -1,
        ]

        result = extract_frontier_cells(data, width=3, height=3)

        self.assertEqual(result, [])

    def test_rejects_invalid_grid_length(self):
        with self.assertRaises(ValueError):
            extract_frontier_cells([0, 0, 0], width=2, height=2)


if __name__ == "__main__":
    unittest.main()