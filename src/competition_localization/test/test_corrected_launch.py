#!/usr/bin/env python3
"""Static reproducibility checks for the experimental corrected launch."""

import unittest
import xml.etree.ElementTree as element_tree
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch" / "localization_corrected.launch"
PRODUCTION = PACKAGE.parent / "competition_bringup" / "launch" / "competition_system.launch"


class TestCorrectedLaunch(unittest.TestCase):
    def test_launch_contains_complete_experimental_chain(self):
        root = element_tree.parse(LAUNCH).getroot()
        nodes = {
            (node.attrib.get("pkg"), node.attrib.get("type"))
            for node in root.findall("node")
        }
        self.assertIn(("competition_localization", "filter_livox.py"), nodes)
        self.assertIn(("competition_localization", "leg_odometry.py"), nodes)
        self.assertIn(("fast_lio", "fastlio_mapping"), nodes)
        self.assertIn(("competition_localization", "localization_node"), nodes)

    def test_required_safety_parameters_are_exposed(self):
        root = element_tree.parse(LAUNCH).getroot()
        arguments = {arg.attrib["name"] for arg in root.findall("arg")}
        for required in (
            "correction_enabled", "correction_mode", "correction_timeout",
            "max_correction_skew", "filtered_cloud_topic", "leg_odom_topic",
            "max_imu_joint_skew",
        ):
            self.assertIn(required, arguments)

    def test_launch_is_marked_experimental(self):
        text = LAUNCH.read_text(encoding="utf-8")
        self.assertIn("EXPERIMENTAL / VALIDATION ONLY", text)
        self.assertIn("NOT YET APPROVED FOR FORMAL COMPETITION MODE", text)

    def test_production_launch_does_not_auto_enable_corrected_chain(self):
        text = PRODUCTION.read_text(encoding="utf-8")
        self.assertNotIn("localization_corrected.launch", text)
        self.assertNotIn("leg_odometry.py", text)
        self.assertNotIn("filter_livox.py", text)


if __name__ == "__main__":
    unittest.main()
