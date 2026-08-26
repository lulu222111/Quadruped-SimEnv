#!/usr/bin/env python3

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


class ReturningLaunchDefaultsTest(unittest.TestCase):
    def test_returning_is_explicitly_disabled_by_default(self):
        launch = (
            Path(__file__).resolve().parents[2]
            / "competition_bringup"
            / "launch"
            / "competition_system.launch"
        )
        root = ET.parse(str(launch)).getroot()
        args = {node.attrib["name"]: node.attrib for node in root.findall("arg")}
        self.assertEqual(args["return_on_first_detection"]["default"], "false")
        self.assertEqual(args["finish_on_first_detection"]["default"], "false")

    def test_returning_params_are_scoped_to_competition_loop(self):
        launch = (
            Path(__file__).resolve().parents[2]
            / "competition_bringup"
            / "launch"
            / "competition_system.launch"
        )
        root = ET.parse(str(launch)).getroot()
        loop_nodes = [
            node for node in root.findall("node")
            if node.attrib.get("name") == "competition_loop"
        ]
        self.assertEqual(len(loop_nodes), 1)
        params = {node.attrib["name"]: node.attrib for node in loop_nodes[0].findall("param")}
        self.assertEqual(params["return_on_first_detection"]["value"], "$(arg return_on_first_detection)")
        self.assertEqual(params["return_arrival_samples"]["value"], "$(arg return_arrival_samples)")


if __name__ == "__main__":
    unittest.main()
