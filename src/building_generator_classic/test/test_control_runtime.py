from __future__ import annotations

from types import SimpleNamespace
import unittest

from unittest.mock import Mock

from building_generator_classic.control_server import (
    _apply_initial_open_doors,
    _compose_world_pose,
    _pose_inside_elevator_car,
)
from building_generator_classic.control_runtime import BuildingControlRuntime


class BuildingControlRuntimeTest(unittest.TestCase):
    def test_applies_initial_open_doors_to_gazebo(self) -> None:
        runtime = BuildingControlRuntime(
            door_specs=[
                {
                    "id": "main_entrance",
                    "model_name": "dynamic_main_entrance",
                    "initial_open": True,
                    "closed_pose": [0.0, 0.0, 1.2, 0.0, 0.0, 1.5708],
                    "panel_poses": {
                        "left_closed": [0.0, -0.5, 0.0, 0.0, 0.0, 0.0],
                        "left_open": [0.0, -1.06, 0.0, 0.0, 0.0, 0.0],
                        "right_closed": [0.0, 0.5, 0.0, 0.0, 0.0, 0.0],
                        "right_open": [0.0, 1.06, 0.0, 0.0, 0.0, 0.0],
                    },
                },
                {
                    "id": "closed_door",
                    "model_name": "dynamic_closed_door",
                    "initial_open": False,
                },
            ],
            elevator_specs=[],
        )
        set_model_state = Mock()
        set_link_state = Mock()

        applied = _apply_initial_open_doors(runtime, set_model_state, set_link_state)

        self.assertEqual(applied, ("main_entrance",))
        self.assertEqual(set_model_state.call_count, 1)
        self.assertEqual(set_link_state.call_count, 2)
        self.assertEqual(set_model_state.call_args.args[0].model_name, "dynamic_main_entrance")
        self.assertEqual(
            {call.args[0].link_name for call in set_link_state.call_args_list},
            {
                "dynamic_main_entrance::left_panel",
                "dynamic_main_entrance::right_panel",
            },
        )

    def test_updates_door_and_elevator_state(self) -> None:
        runtime = BuildingControlRuntime(
            door_specs=[
                {
                    "id": "elevator_floor_0",
                    "kind": "elevator",
                    "initial_open": True,
                    "motion_duration": 25.0,
                    "panel_poses": {
                        "left_closed": [0.0, -0.35, 0.0, 0.0, 0.0, 0.0],
                        "left_open": [0.0, -1.2, 0.0, 0.0, 0.0, 0.0],
                        "right_closed": [0.0, 0.35, 0.0, 0.0, 0.0, 0.0],
                        "right_open": [0.0, 1.2, 0.0, 0.0, 0.0, 0.0],
                    },
                }
            ],
            elevator_specs=[
                {
                    "id": "elevator_main",
                    "current_floor": 0,
                    "served_floors": [0, 1, 2],
                    "car_size": [1.9, 2.1, 2.3],
                    "floor_poses": {
                        "0": [2.76, 2.6, 1.03, 0.0, 0.0, -1.5708],
                        "1": [2.76, 2.6, 3.63, 0.0, 0.0, -1.5708],
                        "2": [2.76, 2.6, 6.23, 0.0, 0.0, -1.5708],
                    },
                }
            ],
        )

        door_result = runtime.set_door_state("elevator_floor_0", False)
        elevator_result = runtime.call_elevator("elevator_main", 2, True)

        self.assertEqual(door_result["state"], "closed")
        self.assertEqual(door_result["motion_duration"], 25.0)
        self.assertEqual(door_result["start_panel_poses"]["left_panel"], [0.0, -1.2, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(door_result["panel_poses"]["left_panel"], [0.0, -0.35, 0.0, 0.0, 0.0, 0.0])
        self.assertEqual(elevator_result["current_floor"], 2)
        self.assertEqual(elevator_result["previous_floor"], 0)
        self.assertEqual(elevator_result["previous_pose"], [2.76, 2.6, 1.03, 0.0, 0.0, -1.5708])
        self.assertEqual(elevator_result["car_size"], [1.9, 2.1, 2.3])
        self.assertEqual(elevator_result["target_door_id"], "elevator_floor_2")
        self.assertEqual(elevator_result["state"], "door_open")

    def test_compose_world_pose_rotates_local_offsets(self) -> None:
        pose = _compose_world_pose(
            [1.0, 2.0, 0.5, 0.0, 0.0, 3.141592653589793 / 2.0],
            [0.0, 1.0, 0.2, 0.0, 0.0, 0.0],
        )

        self.assertAlmostEqual(pose[0], 0.0, places=6)
        self.assertAlmostEqual(pose[1], 2.0, places=6)
        self.assertAlmostEqual(pose[2], 0.7, places=6)

    def test_detects_robot_inside_elevator_car(self) -> None:
        elevator_pose = [2.76, 2.6, 1.03, 0.0, 0.0, -1.5708]
        car_size = [1.9, 2.1, 2.3]

        inside_pose = SimpleNamespace(position=SimpleNamespace(x=2.76, y=2.6, z=0.55))
        outside_pose = SimpleNamespace(position=SimpleNamespace(x=2.76, y=5.4, z=0.55))

        self.assertTrue(_pose_inside_elevator_car(inside_pose, elevator_pose, car_size))
        self.assertFalse(_pose_inside_elevator_car(outside_pose, elevator_pose, car_size))


if __name__ == "__main__":
    unittest.main()
