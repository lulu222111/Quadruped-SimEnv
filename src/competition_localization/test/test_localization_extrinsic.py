#!/usr/bin/env python3
"""Unit tests for the base <-> IMU extrinsic compensation in localization_node.

FAST-LIO's `body` is the IMU frame (livox_imu_link). The adapter must remove the
fixed extrinsic base -> livox_imu_link so the output describes the BASE pose:

    T_world_to_base = T_world_to_body * (T_base_to_body)^-1

These tests prove the matrix multiplication order using non-zero translation
(xyz = [1, 0, 0]) and non-zero rotation (yaw = 90 deg) extrinsic values.
"""

from __future__ import annotations

import math
import threading
import sys
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

    def to_sec(self) -> float:
        return self._sec


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


class _MockBuffer:
    def lookup_transform(self, *args, **kwargs):
        raise RuntimeError("no TF available in unit test")


class _MockTransformListener:
    def __init__(self, *args, **kwargs):
        pass


class _MockTf2Ros:
    TransformBroadcaster = _TransformBroadcaster
    StaticTransformBroadcaster = _StaticTransformBroadcaster
    Buffer = _MockBuffer
    TransformListener = _MockTransformListener


sys.modules["tf2_ros"] = _MockTf2Ros()


# Minimal nav_msgs / geometry_msgs / competition_interfaces mocks.
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


# Convenience: build a 4x4 transform from translation + yaw-only rotation.
def _T(tx: float, ty: float, tz: float, yaw: float):
    return mod._transform_from_translation_quat(
        (tx, ty, tz), mod._quat_from_euler_rpy(0.0, 0.0, yaw)
    )


class TestExtrinsicComposition(unittest.TestCase):
    """Verify T_world_to_base = T_world_to_body * (T_base_to_body)^-1."""

    def test_translation_only_extrinsic_xyz_100(self):
        """base->body = [1,0,0] (IMU 1 m ahead of base), identity rotation.

        body (IMU) sits at the world origin, so base must be 1 m BEHIND the
        IMU along base -x: world->base translation == (-1, 0, 0).
        """
        T_base_to_body = _T(1.0, 0.0, 0.0, 0.0)
        T_world_to_body = mod._identity_transform()
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        t, _ = mod._transform_to_translation_quat(T_world_to_base)
        self.assertAlmostEqual(t[0], -1.0, places=9)
        self.assertAlmostEqual(t[1], 0.0, places=9)
        self.assertAlmostEqual(t[2], 0.0, places=9)

    def test_rotation_only_extrinsic_yaw_90(self):
        """base->body = yaw +90 deg, zero translation.

        world->base orientation = inv(RotZ(+90)) = RotZ(-90). The base x-axis
        (forward) must point along world -y: RotZ(-90)*[1,0,0] = [0,-1,0].
        """
        T_base_to_body = _T(0.0, 0.0, 0.0, math.pi / 2.0)
        T_world_to_body = mod._identity_transform()
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        _, q = mod._transform_to_translation_quat(T_world_to_base)
        v = mod._quat_rotate(q, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(v[0], 0.0, places=9)
        self.assertAlmostEqual(v[1], -1.0, places=9)
        self.assertAlmostEqual(v[2], 0.0, places=9)

    def test_translation_and_rotation_combined_order(self):
        """base->body = t[1,0,0] AND yaw +90 deg.

        inv([R t; 0 1]) = [R^T  -R^T t; 0 1]:
          R^T = RotZ(-90),  -R^T*[1,0,0] = -[0,-1,0] = [0,1,0].
        So world->base translation == (0, 1, 0) and yaw == -90 deg.
        This proves the multiply order is T_world_body * T_body_base (NOT the
        naive T_world_body * T_base_body).
        """
        T_base_to_body = _T(1.0, 0.0, 0.0, math.pi / 2.0)
        T_world_to_body = mod._identity_transform()
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        t, q = mod._transform_to_translation_quat(T_world_to_base)
        # Translation.
        self.assertAlmostEqual(t[0], 0.0, places=9)
        self.assertAlmostEqual(t[1], 1.0, places=9)
        self.assertAlmostEqual(t[2], 0.0, places=9)
        # Orientation: base forward points along world -y.
        v = mod._quat_rotate(q, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(v[0], 0.0, places=9)
        self.assertAlmostEqual(v[1], -1.0, places=9)
        self.assertAlmostEqual(v[2], 0.0, places=9)

    def test_round_trip_invariant(self):
        """T_world_to_base * T_base_to_body must equal T_world_to_body."""
        T_base_to_body = mod._transform_from_translation_quat(
            (1.0, 0.0, 0.0), mod._quat_from_euler_rpy(0.1, -0.2, 0.7)
        )
        T_world_to_body = mod._transform_from_translation_quat(
            (3.0, -2.0, 1.5), mod._quat_from_euler_rpy(-0.3, 0.4, 1.2)
        )
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        recomposed = mod._transform_multiply(T_world_to_base, T_base_to_body)
        for i in range(16):
            self.assertAlmostEqual(recomposed[i], T_world_to_body[i], places=10)

    def test_odom_message_frames_and_pose(self):
        """The output odom must be frame_id=world / child_frame_id=base and
        carry exactly the composed world->base pose."""
        T_base_to_body = _T(1.0, 0.0, 0.0, math.pi / 2.0)
        T_world_to_body = mod._transform_from_translation_quat(
            (5.0, 1.0, 0.5), mod._quat_from_euler_rpy(0.0, 0.0, 0.4)
        )
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        out = mod.odom_from_transform(
            T_world_to_base, _MockTimeInstance(7.0), "world", "base"
        )
        self.assertEqual(out.header.frame_id, "world")
        self.assertEqual(out.child_frame_id, "base")

        t, _ = mod._transform_to_translation_quat(T_world_to_base)
        self.assertAlmostEqual(out.pose.pose.position.x, t[0], places=9)
        self.assertAlmostEqual(out.pose.pose.position.y, t[1], places=9)
        self.assertAlmostEqual(out.pose.pose.position.z, t[2], places=9)

    def test_alignment_with_extrinsic_reaches_start_pose(self):
        """Full chain: alignment + extrinsic compose must yield the start pose
        at t0 even when the IMU is offset from the base.

        Exercises the real node method `_compute_alignment(first_pose)`.
        """
        start = {"x": 0.0, "y": -3.2, "z": 0.6, "yaw": math.pi / 2.0}
        T_base_to_imu = _T(1.0, 0.0, 0.0, 0.0)  # IMU 1 m ahead of base

        # Minimal adapter (real instance without __init__) exercising the
        # actual node method _compute_alignment.
        adapter = mod.LocalizationAdapter.__new__(mod.LocalizationAdapter)
        adapter._start_x = start["x"]
        adapter._start_y = start["y"]
        adapter._start_z = start["z"]
        adapter._start_yaw = start["yaw"]
        adapter._T_base_to_imu = T_base_to_imu
        adapter._T_world_to_camera_init = None
        # Startup vertical references: a level reported attitude and a floor
        # datum equal to the scene start z reproduce the pre-existing vertical
        # contract, so this test keeps pinning the extrinsic composition.
        adapter._initial_roll = 0.0
        adapter._initial_pitch = 0.0
        adapter._initial_floor_height = start["z"]
        adapter._initial_cloud = None
        adapter._initial_lock = threading.Lock()
        adapter._initial_attitude_sub = None
        adapter._initial_floor_sub = None

        # t0: FAST-LIO body (IMU) pose in camera_init is identity.
        P0 = mod._identity_transform()
        mod.LocalizationAdapter._compute_alignment(adapter, P0)
        T_wc = adapter._T_world_to_camera_init
        self.assertIsNotNone(T_wc)

        # world->base at t0 = T_wc * (T_base_imu)^-1 must equal T_ws.
        T_world_to_base_0 = mod.compose_world_to_base(T_wc, T_base_to_imu)
        t0, _ = mod._transform_to_translation_quat(T_world_to_base_0)
        self.assertAlmostEqual(t0[0], start["x"], places=6)
        self.assertAlmostEqual(t0[1], start["y"], places=6)
        self.assertAlmostEqual(t0[2], start["z"], places=6)

        # A later body pose must still transform consistently (round trip).
        T_cb_later = mod._transform_from_translation_quat(
            (0.5, 0.0, 0.0), mod._quat_from_euler_rpy(0.0, 0.0, 0.3)
        )
        T_world_to_body = mod._transform_multiply(T_wc, T_cb_later)
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_imu)
        recomposed = mod._transform_multiply(T_world_to_base, T_base_to_imu)
        for i in range(16):
            self.assertAlmostEqual(recomposed[i], T_world_to_body[i], places=10)


if __name__ == "__main__":
    unittest.main()
