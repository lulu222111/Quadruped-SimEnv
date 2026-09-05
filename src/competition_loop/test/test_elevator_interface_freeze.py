#!/usr/bin/env python3
"""Regression guard for the V1 elevator public-data boundary."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import types
import unittest


REPO_ROOT = Path(__file__).resolve().parents[3]
FORBIDDEN_GEOMETRY_FILES = (
    "layout_metadata.json",
    "competition_scene.world",
    "building_config.json",
    "scene_manifest.json",
    "door_config.yaml",
    "elevator_config.yaml",
)
TEAM_SCENE_DECLARED_FORBIDDEN_FILES = (
    "layout_metadata.json",
    "building_config.json",
    "scene_manifest.json",
)
ANCHOR_FIELDS = {
    "approach_pose",
    "entry_pose",
    "cabin_pose",
    "exit_pose",
    "door_heading",
}


def _load_scene_generator():
    script = REPO_ROOT / "src" / "building_obstacles" / "scripts" / "generate_competition_scene.py"
    spec = importlib.util.spec_from_file_location("elevator_scene_generator", script)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


class ElevatorInterfaceFreezeTest(unittest.TestCase):
    def test_public_scene_builder_exports_only_semantic_elevator_fields(self):
        generator = _load_scene_generator()
        layout = types.SimpleNamespace(
            door_specs=[],
            elevator_specs=[
                types.SimpleNamespace(
                    id="public_elevator",
                    served_floors=[0, 1],
                    current_floor=0,
                )
            ],
        )

        scene = generator._build_team_scene_info(
            layout,
            {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0},
            Path("results/detected_danger.json"),
        )

        elevator = scene["public_scene"]["elevators"][0]
        self.assertEqual(
            set(elevator), {"id", "served_floors", "current_floor"}
        )
        self.assertTrue(ANCHOR_FIELDS.isdisjoint(elevator))
        declared_forbidden = scene["referee_only"]["forbidden_files"]
        for forbidden in TEAM_SCENE_DECLARED_FORBIDDEN_FILES:
            self.assertTrue(
                any(item.endswith(forbidden) for item in declared_forbidden),
                f"team scene omits forbidden-file declaration for {forbidden}",
            )

    def test_current_algorithm_nodes_do_not_name_forbidden_geometry_files(self):
        """Elevator V1 adds no algorithm path to hidden scene geometry."""
        algorithm_sources = (
            REPO_ROOT / "src" / "competition_loop" / "scripts" / "competition_loop_node",
            REPO_ROOT / "src" / "competition_planning" / "scripts" / "planning_node",
            REPO_ROOT / "src" / "competition_control" / "scripts" / "path_follower_node",
        )
        for source in algorithm_sources:
            text = source.read_text(encoding="utf-8")
            for forbidden in FORBIDDEN_GEOMETRY_FILES:
                self.assertNotIn(forbidden, text, f"{source} names {forbidden}")

    def test_elevator_messages_do_not_encode_navigation_anchor_fields(self):
        for filename in ("ElevatorMission.msg", "ElevatorStatus.msg"):
            text = (
                REPO_ROOT
                / "src"
                / "competition_interfaces"
                / "msg"
                / filename
            ).read_text(encoding="utf-8")
            for field in ANCHOR_FIELDS:
                self.assertNotIn(field, text, f"{filename} exposes {field}")


if __name__ == "__main__":
    unittest.main()
