#!/usr/bin/env python3

from pathlib import Path
import os
import xml.etree.ElementTree as ET
import unittest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
WORKSPACE_SRC = Path(os.environ.get(
    "COMPETITION_WORKSPACE_SRC", str(PACKAGE_ROOT.parent)))
COORDINATOR = PACKAGE_ROOT / "scripts" / "teleport_recovery_coordinator"
STRAIGHT_SOURCE = PACKAGE_ROOT / "scripts" / "straight_segment_path_source"
STRAIGHT_CORE = (
    PACKAGE_ROOT / "src" / "competition_recovery_testing" /
    "straight_segment.py"
)
STRAIGHT_LAUNCH = PACKAGE_ROOT / "launch" / "c2r7_straight_segment.launch"


class TestOnlyIsolationTest(unittest.TestCase):
    def test_production_bringup_never_references_recovery_package(self):
        production_launch = (
            WORKSPACE_SRC / "competition_bringup" / "launch" /
            "competition_system.launch"
        ).read_text()
        self.assertNotIn("competition_recovery_testing", production_launch)
        self.assertNotIn("set_model_state", production_launch)

    def test_gazebo_fall_read_is_explicitly_test_only(self):
        source = COORDINATOR.read_text()
        self.assertIn("if args.test_gazebo_fall_trigger:", source)
        self.assertIn('"/gazebo/model_states"', source)
        self.assertIn('"--test-gazebo-fall-trigger"', source)
        self.assertNotIn("get_model_state", source)
        self.assertIn("/gazebo/set_model_state", source)

    def test_coordinator_has_no_global_cleanup_commands(self):
        source = COORDINATOR.read_text()
        for forbidden in (
            "pkill", "killall", "rosnode kill -a", "docker system prune",
            "docker compose down",
        ):
            self.assertNotIn(forbidden, source)
        self.assertIn("os.killpg(process.pid", source)

    def test_full_stack_runtime_requires_auto_start_false(self):
        source = COORDINATOR.read_text()
        self.assertIn('"auto_start:=false" not in command_text', source)

    def test_straight_source_has_no_ground_truth_or_velocity_interface(self):
        source = STRAIGHT_SOURCE.read_text()
        core = STRAIGHT_CORE.read_text()
        combined = source + core
        for forbidden in (
            "/gazebo/", "/ground_truth/", "gazebo_msgs", "ModelStates",
            'Publisher("/cmd_vel"',
            'Publisher("/competition/raw_cmd_vel"',
        ):
            self.assertNotIn(forbidden, combined)
        self.assertIn('"/competition/path"', source)
        self.assertIn('"/competition/planning/status"', source)
        self.assertNotIn('"/competition/navigation_goal"', source)
        self.assertEqual(source.count("rospy.Publisher("), 2)

    def test_straight_source_checks_gate_before_constructing_publishers(self):
        source = STRAIGHT_SOURCE.read_text()
        main = source[source.index("def main():"):]
        self.assertLess(
            main.index("validate_runtime_gate("),
            main.index("StraightSegmentPathSource()"),
        )

    def test_straight_terminal_publication_cancels_path_before_status(self):
        source = STRAIGHT_SOURCE.read_text()
        method = source[
            source.index("    def _publish_cancel("):
            source.index("    @staticmethod\n    def _status_message(")
        ]
        self.assertLess(
            method.index("self._path_pub.publish(empty_path)"),
            method.index("self._status_pub.publish("),
        )

    def test_c2r7_launch_is_default_off_and_keeps_legal_chain(self):
        root = ET.fromstring(STRAIGHT_LAUNCH.read_text())
        args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual(args["test_only_acknowledged"], "false")
        include_args = {
            element.attrib["name"]: element.attrib["value"]
            for element in root.find("include").findall("arg")
        }
        self.assertEqual(include_args["competition_mode"], "false")
        self.assertEqual(include_args["use_mocks"], "false")
        self.assertEqual(include_args["planning_enabled"], "false")
        self.assertEqual(include_args["path_follower_max_angular"], "0.15")
        self.assertEqual(include_args["path_follower_path_timeout"], "1.0")
        launch_text = STRAIGHT_LAUNCH.read_text()
        self.assertNotIn("/cmd_vel", launch_text)
        self.assertNotIn("raw_cmd_vel", launch_text)
        self.assertNotIn("gazebo", launch_text.lower())

    def test_production_launch_defaults_remain_enabled_and_unbounded(self):
        launch_path = (
            WORKSPACE_SRC / "competition_bringup" / "launch" /
            "competition_system.launch"
        )
        root = ET.fromstring(launch_path.read_text())
        args = {
            element.attrib["name"]: element.attrib.get("default")
            for element in root.findall("arg")
        }
        self.assertEqual(args["planning_enabled"], "true")
        self.assertEqual(args["path_follower_max_angular"], "0.8")
        self.assertEqual(args["path_follower_path_timeout"], "5.0")

        planner = root.find(
            ".//group[@if=\"$(arg planning_enabled)\"]"
            "/node[@type='planning_node']"
        )
        self.assertIsNotNone(planner)
        follower = root.find(".//node[@type='path_follower_node']")
        follower_params = {
            element.attrib["name"]: element.attrib["value"]
            for element in follower.findall("param")
        }
        self.assertEqual(
            follower_params["max_angular"],
            "$(arg path_follower_max_angular)",
        )
        self.assertEqual(
            follower_params["path_timeout"],
            "$(arg path_follower_path_timeout)",
        )


if __name__ == "__main__":
    unittest.main()
