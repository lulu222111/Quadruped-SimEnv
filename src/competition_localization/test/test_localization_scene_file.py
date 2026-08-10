#!/usr/bin/env python3
"""Unit tests for the strict scene-file validation (fail-safe) in
localization_node.

Requirement: `generated_building/team_scene_info.json` must pass a strict
schema check. On ANY failure the node is set to FAILED (valid=false) and valid
localization publishing is cut off. There is NO silent fallback to (0,0,0,0).
"""

from __future__ import annotations

import json
import math
import os
import sys
import tempfile
import unittest
from pathlib import Path


# ---------------------------------------------------------------------------
# Minimal ROS mocks so we can import the node script without ROS runtime.
# ---------------------------------------------------------------------------

class _MockDuration:
    def __init__(self, secs: float):
        self._s = secs

    def to_sec(self) -> float:
        return self._s


class _MockTimeInstance:
    __slots__ = ("_sec",)

    def __init__(self, s: float = 0.0):
        self._sec = s


class _MockTime:
    _now = 0.0

    @classmethod
    def now(cls):
        return _MockTimeInstance(cls._now)


class _MockLog:
    @staticmethod
    def info(*args, **kwargs):
        pass

    @staticmethod
    def warn(*args, **kwargs):
        pass

    @staticmethod
    def loginfo(*args, **kwargs):
        pass

    @staticmethod
    def logwarn(*args, **kwargs):
        pass

    @staticmethod
    def logerr(*args, **kwargs):
        pass

    @staticmethod
    def logfatal(*args, **kwargs):
        pass

    @staticmethod
    def warn_throttle(*args, **kwargs):
        pass


class _MockRospy:
    Time = _MockTime
    Duration = _MockDuration

    def __getattr__(self, name):
        return getattr(_MockLog, name, _MockLog.info)

    def init_node(self, *args, **kwargs):
        pass

    def get_param(self, name, default=None):
        return default

    def Publisher(self, *args, **kwargs):
        return type("P", (), {"publish": lambda self, m: None})()

    def Subscriber(self, *args, **kwargs):
        return None

    def Rate(self, hz):
        return type("R", (), {"sleep": lambda: None})()

    def is_shutdown(self):
        return True


sys.modules["rospy"] = _MockRospy()


class _TransformBroadcaster:
    def sendTransform(self, *args, **kwargs):
        pass


class _StaticTransformBroadcaster:
    def sendTransform(self, *args, **kwargs):
        pass


class _MockTf2Ros:
    TransformBroadcaster = _TransformBroadcaster
    StaticTransformBroadcaster = _StaticTransformBroadcaster


sys.modules["tf2_ros"] = _MockTf2Ros()


class _Point:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.x = x
        self.y = y
        self.z = z


class _Quaternion:
    def __init__(self, x=0.0, y=0.0, z=0.0, w=1.0):
        self.x = x
        self.y = y
        self.z = z
        self.w = w


class _Pose:
    def __init__(self, position=None, orientation=None):
        self.position = position or _Point()
        self.orientation = orientation or _Quaternion()


class _PoseWithCovariance:
    def __init__(self, pose=None):
        self.pose = pose or _Pose()
        self.covariance = [0.0] * 36


class _Header:
    def __init__(self, frame_id="", stamp=None):
        self.frame_id = frame_id
        self.stamp = stamp or _MockTimeInstance()


class _Odometry:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.pose = _PoseWithCovariance()
        self.twist = type("TwistWCov", (), {"twist": type("Twist", (), {
            "linear": _Point(), "angular": _Point()})(),
            "covariance": [0.0] * 36})()


class _NavMsgsMsg:
    Odometry = _Odometry


class _NavMsgs:
    msg = _NavMsgsMsg()


sys.modules["nav_msgs"] = _NavMsgs()
sys.modules["nav_msgs.msg"] = _NavMsgsMsg()


class _TransformStamped:
    def __init__(self):
        self.header = _Header()
        self.child_frame_id = ""
        self.transform = type("Transform", (), {
            "translation": _Point(), "rotation": _Quaternion()})()


class _GeometryMsgsMsg:
    TransformStamped = _TransformStamped
    Quaternion = _Quaternion


class _GeometryMsgs:
    msg = _GeometryMsgsMsg()


sys.modules["geometry_msgs"] = _GeometryMsgs()
sys.modules["geometry_msgs.msg"] = _GeometryMsgsMsg()


class _ModuleStatus:
    UNKNOWN = 0
    STARTING = 1
    READY = 2
    RUNNING = 3
    SUCCEEDED = 4
    BLOCKED = 5
    TIMEOUT = 6
    FAILED = 7

    def __init__(self):
        self.header = _Header()
        self.state = 0
        self.valid = False
        self.confidence = 0.0
        self.detail = ""


class _CompetitionInterfacesMsg:
    ModuleStatus = _ModuleStatus


class _CompetitionInterfaces:
    msg = _CompetitionInterfacesMsg()


sys.modules["competition_interfaces"] = _CompetitionInterfaces()
sys.modules["competition_interfaces.msg"] = _CompetitionInterfacesMsg()


# ---------------------------------------------------------------------------
# Import the node under test (pure functions only, strip the __main__ block).
# ---------------------------------------------------------------------------

_SRC = str(Path(__file__).resolve().parents[1] / "scripts")
_NODE_PATH = f"{_SRC}/localization_node"

mod = type("mod", (), {})()
with open(_NODE_PATH, "r", encoding="utf-8") as f:
    source = f.read()
_source_no_main = source.split("if __name__")[0]
exec(compile(_source_no_main, _NODE_PATH, "exec"), mod.__dict__)
mod.__file__ = _NODE_PATH  # needed by _load_start_pose's repo-root fallback


def _adapter(path: str, competition_mode: bool = True) -> object:
    """Real LocalizationAdapter instance (without __init__) exposing the
    attributes used by `_load_start_pose` / `_fail_start_pose`."""
    adapter = mod.LocalizationAdapter.__new__(mod.LocalizationAdapter)
    adapter.team_scene_info_path = path
    adapter.competition_mode = competition_mode
    adapter._start_pose_valid = False
    adapter._start_x = 0.0
    adapter._start_y = 0.0
    adapter._start_z = 0.0
    adapter._start_yaw = 0.0
    adapter._state = _ModuleStatus.STARTING
    adapter._valid = False
    adapter._detail = ""
    return adapter


def _write(tmpdir: str, content) -> str:
    """Write `content` (str or dict) to a temp file and return its path."""
    path = os.path.join(tmpdir, "team_scene_info.json")
    if isinstance(content, str):
        with open(path, "w", encoding="utf-8") as f:
            f.write(content)
    else:
        with open(path, "w", encoding="utf-8") as f:
            json.dump(content, f)
    return path


def _valid_scene(start=None) -> dict:
    return {
        "schema": "team_scene_info_v1",
        "coordinate_frame": "world",
        "robot_start": start or {"x": 0.0, "y": -3.2, "z": 0.6, "yaw": 1.5708},
    }


class TestSceneFileValidation(unittest.TestCase):
    """Strict validation + fail-safe for team_scene_info.json."""

    def test_valid_scene_accepted(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, _valid_scene())
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertTrue(adapter._start_pose_valid)
            self.assertNotEqual(adapter._state, _ModuleStatus.FAILED)
            self.assertAlmostEqual(adapter._start_x, 0.0)
            self.assertAlmostEqual(adapter._start_y, -3.2)
            self.assertAlmostEqual(adapter._start_z, 0.6)
            self.assertAlmostEqual(adapter._start_yaw, 1.5708)

    def test_missing_file_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "does_not_exist.json")
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)
            self.assertFalse(adapter._valid)
            self.assertIn("invalid", adapter._detail)

    def test_invalid_json_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, "{ this is not json")
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)
            self.assertFalse(adapter._valid)

    def test_root_not_object_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = _write(tmp, [1, 2, 3])
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)

    def test_wrong_coordinate_frame_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = _valid_scene()
            scene["coordinate_frame"] = "odom"  # forbidden frame
            path = _write(tmp, scene)
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)

    def test_missing_robot_start_fields_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = _valid_scene(start={"x": 0.0, "y": -3.2, "yaw": 1.5708})  # no z
            path = _write(tmp, scene)
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)

    def test_non_numeric_robot_start_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = _valid_scene(
                start={"x": "a", "y": -3.2, "z": 0.6, "yaw": 1.5708}
            )
            path = _write(tmp, scene)
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)

    def test_non_finite_robot_start_fails(self):
        with tempfile.TemporaryDirectory() as tmp:
            scene = _valid_scene(
                start={"x": float("nan"), "y": -3.2, "z": 0.6, "yaw": 1.5708}
            )
            path = _write(tmp, scene)
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)

    def test_no_silent_fallback_to_origin(self):
        """A failed load must NOT look like a valid origin start."""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "missing.json")
            adapter = _adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            # Even though start_* fall back to 0.0, the module must be marked
            # invalid so the node refuses to publish odometry.
            self.assertFalse(adapter._start_pose_valid)
            self.assertFalse(adapter._valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)


if __name__ == "__main__":
    unittest.main()
