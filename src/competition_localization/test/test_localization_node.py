#!/usr/bin/env python3
"""Unit tests for localization_node — pure Python, no ROS infrastructure needed.

Tests: quaternion identity, transform inverse, start-pose alignment,
extrinsic composition (item 2), first-frame latching (item 3), scene-file
Safe-Fail (items 4/5), frame BLOCKED (item 6), timestamp Safe-Fail (item 7),
and rejection of NaN/Inf/degenerate odometry inputs.
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
    def __init__(self, secs: float):  # noqa: D401
        self._s = secs

    def to_sec(self) -> float:
        return self._s


class _MockTimeInstance:
    __slots__ = ("_sec",)

    def __init__(self, s: float = 0.0):
        self._sec = s

    def to_sec(self) -> float:
        return self._sec

    def __sub__(self, other):
        if isinstance(other, _MockTimeInstance):
            return _MockDuration(self._sec - other._sec)
        return NotImplemented


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

    def loginfo(self, *args, **kwargs):
        pass

    def logwarn(self, *args, **kwargs):
        pass

    def logerr(self, *args, **kwargs):
        pass


sys.modules["rospy"] = _MockRospy()


class _TransformBroadcaster:
    def sendTransform(self, *args, **kwargs):
        pass


class _StaticTransformBroadcaster:
    def sendTransform(self, *args, **kwargs):
        pass


class _RecordingBroadcaster:
    """Records the last TransformStamped passed to sendTransform."""

    def __init__(self):
        self.sent = []

    def sendTransform(self, tf_msg):
        self.sent.append(tf_msg)


class _RecordingPublisher:
    """Records every message passed to publish()."""

    def __init__(self):
        self.published = []

    def publish(self, msg):
        self.published.append(msg)


class _TfStore:
    """Minimal in-memory TF tree: records sendTransform() calls and answers
    lookup_transform("world", "base", stamp) queries at or before a stamp."""

    def __init__(self):
        self.sent = []
        self._by_stamp = {}

    def sendTransform(self, tf_msg):
        self.sent.append(tf_msg)
        self._by_stamp[tf_msg.header.stamp.to_sec()] = tf_msg

    def lookup_transform(self, target, source, time, timeout=None):
        t = time.to_sec()
        avail = sorted(s for s in self._by_stamp if s <= t)
        if not avail:
            raise RuntimeError(f"no TF {target}->{source} available at t={t}")
        return self._by_stamp[avail[-1]]


class _MockTf2Ros:
    TransformBroadcaster = _TransformBroadcaster
    StaticTransformBroadcaster = _StaticTransformBroadcaster


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


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_odom(x=0.0, y=0.0, z=0.0, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
               frame_id="camera_init", child_frame_id="body", stamp=None) -> _Odometry:
    """Build a mock Odometry message with a valid non-zero stamp by default."""
    odom = _Odometry()
    odom.header.frame_id = frame_id
    odom.header.stamp = stamp if stamp is not None else _MockTimeInstance(1.0)
    odom.child_frame_id = child_frame_id
    odom.pose.pose.position = _Point(x, y, z)
    odom.pose.pose.orientation = _Quaternion(qx, qy, qz, qw)
    return odom


def _adapter_with_start(start: dict) -> object:
    """Create a LocalizationAdapter instance (without __init__) with a given
    robot_start, so class methods like _odom_callback / _compute_alignment are
    bound."""
    adapter = mod.LocalizationAdapter.__new__(mod.LocalizationAdapter)
    adapter._start_x = float(start.get("x", 0.0))
    adapter._start_y = float(start.get("y", 0.0))
    adapter._start_z = float(start.get("z", 0.0))
    adapter._start_yaw = float(start.get("yaw", 0.0))
    adapter._T_world_to_camera_init = None
    adapter._T_base_to_imu = mod._identity_transform()  # no extrinsic (base == body)
    adapter._aligned = False
    adapter._start_pose_valid = True
    adapter._first_valid_pose = None
    adapter._first_valid_pose_time = None
    adapter._last_input_stamp = None
    adapter.start_alignment_delay = 2.0
    adapter.input_frame_id = "camera_init"
    adapter.input_child_frame_id = "body"
    adapter.input_timeout = 0.5
    adapter._max_future_stamp_skew = 1.0
    adapter.competition_mode = False
    adapter._state = _ModuleStatus.STARTING
    adapter._valid = False
    adapter._detail = ""
    adapter._last_odom_time = None
    adapter._odom_msg = None
    adapter._odom_pub = type("P", (), {"publish": lambda self, m: None})()
    adapter._status_pub = type("P", (), {"publish": lambda self, m: None})()
    adapter._br = None
    adapter.publish_tf_enabled = False
    adapter._lock = type("L", (), {"acquire": lambda self: None, "release": lambda self: None, "__enter__": lambda self: self, "__exit__": lambda *args: None})()
    return adapter


class TestLocalizationNode(unittest.TestCase):
    """Pure-math tests for the FAST-LIO localization adapter."""

    def test_identity_quaternion_rotation(self):
        """Unit quaternion should leave any vector unchanged."""
        identity = (0.0, 0.0, 0.0, 1.0)
        v = (1.0, -2.0, 3.0)
        result = mod._quat_rotate(identity, v)
        self.assertAlmostEqual(result[0], v[0])
        self.assertAlmostEqual(result[1], v[1])
        self.assertAlmostEqual(result[2], v[2])

    def test_transform_inverse_identity(self):
        """T * inv(T) should be the identity transform."""
        q = mod._quat_from_euler_rpy(0.1, -0.2, 0.7)
        t = (1.0, -2.0, 0.5)
        T = mod._transform_from_translation_quat(t, q)
        T_inv = mod._transform_inverse(T)
        T_identity = mod._transform_multiply(T, T_inv)

        eye = mod._identity_transform()
        for i in range(16):
            self.assertAlmostEqual(T_identity[i], eye[i], places=10)

    def test_coerce_float_list(self):
        """Extrinsic params may arrive as lists or as string lists (launch
        <param value=\"[...]\"> stores a string); both must parse to floats."""
        self.assertEqual(
            mod._coerce_float_list([0.223404, -0.023290, 0.118985]),
            [0.223404, -0.023290, 0.118985],
        )
        self.assertEqual(
            mod._coerce_float_list((0.0, 0.785, 0.0)),
            [0.0, 0.785, 0.0],
        )
        self.assertEqual(
            mod._coerce_float_list("[0.0, 0.785, 0.0]"),
            [0.0, 0.785, 0.0],
        )
        self.assertEqual(
            mod._coerce_float_list("0.0, 0.785, 0.0"),
            [0.0, 0.785, 0.0],
        )
        with self.assertRaises(ValueError):
            mod._coerce_float_list("[0.0, abc, 0.0]")

    def test_start_alignment(self):
        """Identity FAST-LIO pose should map to robot_start in world frame."""
        start = {"x": 0.0, "y": -3.2, "z": 0.6, "yaw": math.pi / 2.0}
        adapter = _adapter_with_start(start)
        odom = _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)

        mod.LocalizationAdapter._compute_alignment(adapter, mod.transform_from_odom(odom))
        self.assertIsNotNone(adapter._T_world_to_camera_init)

        T_body = mod.transform_from_odom(odom)
        T_world_body = mod._transform_multiply(adapter._T_world_to_camera_init, T_body)
        out = mod.odom_from_transform(
            T_world_body, _MockTimeInstance(), "world", "base"
        )

        self.assertAlmostEqual(out.pose.pose.position.x, start["x"], places=6)
        self.assertAlmostEqual(out.pose.pose.position.y, start["y"], places=6)
        self.assertAlmostEqual(out.pose.pose.position.z, start["z"], places=6)

        _, q = mod._transform_to_translation_quat(T_world_body)
        # Compare yaw by rotating a unit x-vector.
        v_world = mod._quat_rotate(q, (1.0, 0.0, 0.0))
        v_expected = mod._quat_rotate(
            mod._quat_from_euler_rpy(0.0, 0.0, start["yaw"]), (1.0, 0.0, 0.0)
        )
        for i in range(3):
            self.assertAlmostEqual(v_world[i], v_expected[i], places=6)

    def test_nan_position_rejected(self):
        """NaN position values should mark the module as FAILED."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        odom = _make_odom(float("nan"), 0.0, 0.0)
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)

    def test_inf_orientation_rejected(self):
        """Inf orientation values should mark the module as FAILED."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        odom = _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, float("inf"))
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)

    def test_degenerate_quaternion_rejected(self):
        """Zero quaternion should be rejected and mark the module as FAILED."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        odom = _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)

    def test_publish_tf_world_to_base_matches_odom(self):
        """Dynamic TF must be world -> base and match the odometry message.

        The `odom` frame is removed from the system; the localization node is
        the only dynamic world -> base TF publisher.
        """
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        br = _RecordingBroadcaster()
        adapter._br = br

        stamp = _MockTimeInstance(123.0)
        odom = _make_odom(1.0, -2.0, 0.5, 0.0, 0.0, 0.383, 0.924)
        odom.header.stamp = stamp

        mod.LocalizationAdapter._publish_tf(adapter, odom)

        self.assertEqual(len(br.sent), 1)
        tf_msg = br.sent[0]
        self.assertEqual(tf_msg.header.frame_id, "world")
        self.assertEqual(tf_msg.child_frame_id, "base")
        self.assertIs(tf_msg.header.stamp, stamp)
        # TF values are taken verbatim from the odometry message.
        self.assertAlmostEqual(tf_msg.transform.translation.x, odom.pose.pose.position.x)
        self.assertAlmostEqual(tf_msg.transform.translation.y, odom.pose.pose.position.y)
        self.assertAlmostEqual(tf_msg.transform.translation.z, odom.pose.pose.position.z)
        self.assertAlmostEqual(tf_msg.transform.rotation.x, odom.pose.pose.orientation.x)
        self.assertAlmostEqual(tf_msg.transform.rotation.y, odom.pose.pose.orientation.y)
        self.assertAlmostEqual(tf_msg.transform.rotation.z, odom.pose.pose.orientation.z)
        self.assertAlmostEqual(tf_msg.transform.rotation.w, odom.pose.pose.orientation.w)

    def test_no_static_world_to_odom(self):
        """The node must no longer contain any `odom` frame publishing logic.

        The whole system removed the `odom` frame; the localization node must
        only publish the dynamic world -> base TF.
        """
        self.assertNotIn("_publish_static_world_to_odom", source)
        self.assertNotIn("_static_br", source)
        self.assertNotIn('"odom"', source)

    def test_first_frame_latching_uses_p0_not_pdelay(self):
        """Alignment must use the latched FIRST pose P0, never the pose seen
        after the stabilization delay (Pdelay != P0).

        Regression test for the start-alignment drift bug: previously the pose
        at the delay moment (possibly after the robot moved) was used, biasing
        the world frame.
        """
        adapter = _adapter_with_start({"x": 0.0, "y": -3.2, "z": 0.6, "yaw": math.pi / 2.0})
        # Non-identity extrinsic keeps the check meaningful.
        adapter._T_base_to_imu = mod._transform_from_translation_quat(
            (1.0, 0.0, 0.0), mod._quat_from_euler_rpy(0.0, 0.0, 0.0)
        )
        adapter.start_alignment_delay = 1.0

        # t0: first valid pose P0 arrives immediately.
        _MockTime._now = 0.0
        P0 = _make_odom(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        mod.LocalizationAdapter._odom_callback(adapter, P0)
        self.assertIsNotNone(adapter._first_valid_pose)
        self.assertFalse(adapter._aligned)  # 0s < 1s delay -> not aligned yet

        # Robot moves; at t=3s a different pose arrives (Pdelay != P0).
        _MockTime._now = 3.0
        Pdelay = _make_odom(5.0, 4.0, 0.0, 0.0, 0.0, 0.707, 0.707)
        mod.LocalizationAdapter._odom_callback(adapter, Pdelay)
        self.assertTrue(adapter._aligned)

        # Alignment must be T_start * T_base_imu * (P0)^-1.
        q_ws = mod._quat_from_euler_rpy(0.0, 0.0, math.pi / 2.0)
        T_ws = mod._transform_from_translation_quat((0.0, -3.2, 0.6), q_ws)
        expected = mod._transform_multiply(
            mod._transform_multiply(T_ws, adapter._T_base_to_imu),
            mod._transform_inverse(mod.transform_from_odom(P0)),
        )
        for i in range(16):
            self.assertAlmostEqual(
                adapter._T_world_to_camera_init[i], expected[i], places=6
            )

        # Explicitly prove it is NOT based on Pdelay.
        wrong = mod._transform_multiply(
            mod._transform_multiply(T_ws, adapter._T_base_to_imu),
            mod._transform_inverse(mod.transform_from_odom(Pdelay)),
        )
        self.assertNotAlmostEqual(
            adapter._T_world_to_camera_init[3], wrong[3], places=6
        )

    def test_frame_mismatch_sets_blocked(self):
        """Input frames != configured camera_init/body -> BLOCKED, no publish."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.start_alignment_delay = 0.0
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        odom = _make_odom(0.0, 0.0, 0.0,
                          frame_id="odom", child_frame_id="base",
                          stamp=_MockTimeInstance(1.0))
        mod.LocalizationAdapter._odom_callback(adapter, odom)

        self.assertEqual(adapter._state, _ModuleStatus.BLOCKED)
        self.assertFalse(adapter._valid)
        self.assertEqual(len(pub.published), 0)  # no valid odom published

    def test_zero_timestamp_rejected(self):
        """A zero input timestamp must be rejected (FAILED)."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        odom = _make_odom(0.0, 0.0, 0.0, stamp=_MockTimeInstance(0.0))
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)

    def test_timestamp_jump_back_rejected(self):
        """A timestamp going backwards must be rejected (FAILED)."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(0.0, 0.0, 0.0, stamp=_MockTimeInstance(2.0))
        )
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(0.0, 0.0, 0.0, stamp=_MockTimeInstance(1.0))
        )
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)

    def test_future_timestamp_rejected(self):
        """A timestamp beyond max_future_stamp_skew ahead of now() is rejected."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter._max_future_stamp_skew = 1.0
        _MockTime._now = 10.0
        odom = _make_odom(1.0, 0.0, 0.0, stamp=_MockTimeInstance(12.5))
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        self.assertEqual(adapter._state, _ModuleStatus.FAILED)
        self.assertFalse(adapter._valid)
        self.assertIn("future", adapter._detail)

    def test_future_timestamp_within_tolerance_accepted(self):
        """A small future skew within max_future_stamp_skew is accepted."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.start_alignment_delay = 0.0
        adapter._max_future_stamp_skew = 1.0
        adapter._T_base_to_imu = mod._identity_transform()
        _MockTime._now = 10.0
        odom = _make_odom(1.0, 0.0, 0.0, stamp=_MockTimeInstance(10.5))
        pub = _RecordingPublisher()
        adapter._odom_pub = pub
        mod.LocalizationAdapter._odom_callback(adapter, odom)
        # Should be accepted — 10.5 is within 10.0 + 1.0
        self.assertEqual(adapter._state, _ModuleStatus.RUNNING)
        self.assertTrue(adapter._valid)

    def test_output_uses_input_original_timestamp(self):
        """The output odometry must carry the INPUT original timestamp, not the
        reception wall-clock time."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.start_alignment_delay = 0.0
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        _MockTime._now = 10.0  # wall clock at reception
        odom = _make_odom(1.0, 2.0, 3.0, 0.0, 0.0, 0.0, 1.0,
                          stamp=_MockTimeInstance(7.5))
        mod.LocalizationAdapter._odom_callback(adapter, odom)

        self.assertEqual(len(pub.published), 1)
        out = pub.published[0]
        self.assertEqual(out.header.stamp.to_sec(), 7.5)

    def test_watchdog_timeout_sets_timedout(self):
        """No valid input for longer than input_timeout -> TIMEOUT, valid=false."""
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.input_timeout = 0.5
        _MockTime._now = 0.0
        adapter._last_odom_time = _MockTimeInstance(0.0)

        _MockTime._now = 2.0
        mod.LocalizationAdapter._check_timeout(adapter)
        self.assertEqual(adapter._state, _ModuleStatus.TIMEOUT)
        self.assertFalse(adapter._valid)

    def test_static_stability_no_drift(self):
        """Continuous static input must not drift position or heading.

        (Part 10.1) With a latched static pose the output must stay exactly at
        the start pose across many frames — no X/Y/Z drift, no yaw drift.
        """
        start = {"x": 1.0, "y": 2.0, "z": 0.5, "yaw": 0.7}
        adapter = _adapter_with_start(start)
        adapter.start_alignment_delay = 0.0
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        expected_q = mod._quat_from_euler_rpy(0.0, 0.0, start["yaw"])
        for i in range(50):
            _MockTime._now = 10.0 + i * 0.05
            odom = _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                              stamp=_MockTimeInstance(10.0 + i * 0.05))
            mod.LocalizationAdapter._odom_callback(adapter, odom)

        self.assertEqual(len(pub.published), 50)
        for out in pub.published:
            self.assertAlmostEqual(out.pose.pose.position.x, start["x"], places=6)
            self.assertAlmostEqual(out.pose.pose.position.y, start["y"], places=6)
            self.assertAlmostEqual(out.pose.pose.position.z, start["z"], places=6)
            o = out.pose.pose.orientation
            for j, c in enumerate((o.x, o.y, o.z, o.w)):
                self.assertAlmostEqual(c, expected_q[j], places=6)

    def test_motion_direction_and_turning(self):
        """Forward motion along robot X and right-handed yaw turns.

        (Part 10.2) Start yaw = +90 deg, so the robot X axis points along world
        +Y: moving +1 m along robot X must increase world Y (not X). A +90 deg
        body turn (CCW about +Z) must add +90 deg yaw -> world yaw 180 deg.
        """
        start = {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": math.pi / 2.0}
        adapter = _adapter_with_start(start)
        adapter.start_alignment_delay = 0.0
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        _MockTime._now = 0.0
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                stamp=_MockTimeInstance(1.0)))  # latch P0

        # Forward: +1 m along robot X.
        _MockTime._now = 0.1
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                stamp=_MockTimeInstance(1.1)))
        forward = pub.published[-1]
        self.assertAlmostEqual(forward.pose.pose.position.x, 0.0, places=6)
        self.assertAlmostEqual(forward.pose.pose.position.y, 1.0, places=6)

        # Turn: +90 deg about +Z (right-hand rule).
        _MockTime._now = 0.2
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(1.0, 0.0, 0.0, 0.0, 0.0,
                                math.sin(math.pi / 4.0), math.cos(math.pi / 4.0),
                                stamp=_MockTimeInstance(1.2)))
        turned = pub.published[-1]
        o = turned.pose.pose.orientation
        yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y),
                         1.0 - 2.0 * (o.y * o.y + o.z * o.z))
        self.assertAlmostEqual(yaw, math.pi, places=6)

    def test_tf_continuously_queryable(self):
        """world -> base TF must be published at high frequency with no gaps
        and stay queryable at every odometry timestamp.

        (Part 10.3) Simulates 50 frames at 100 Hz-equivalent spacing and
        verifies a world -> base TF exists for EVERY odometry (no dropout) and
        matches the odometry pose.
        """
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.start_alignment_delay = 0.0
        store = _TfStore()
        adapter._br = store
        adapter.publish_tf_enabled = True
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        n = 50
        base_stamp = 100.0
        for i in range(n):
            _MockTime._now = base_stamp + i * 0.01
            mod.LocalizationAdapter._odom_callback(
                adapter,
                _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                           stamp=_MockTimeInstance(base_stamp + i * 0.01)),
            )

        self.assertEqual(len(pub.published), n)
        self.assertEqual(len(store.sent), n)  # a TF for EVERY odometry (no gaps)
        for out in pub.published:
            t = out.header.stamp.to_sec()
            tf = store.lookup_transform("world", "base", _MockTimeInstance(t))
            self.assertEqual(tf.header.frame_id, "world")
            self.assertEqual(tf.child_frame_id, "base")
            self.assertTrue(math.isfinite(tf.transform.translation.x))
            self.assertTrue(math.isfinite(tf.transform.translation.y))
            self.assertTrue(math.isfinite(tf.transform.translation.z))
            self.assertAlmostEqual(
                tf.transform.translation.x, out.pose.pose.position.x, places=6)
            self.assertAlmostEqual(
                tf.transform.translation.y, out.pose.pose.position.y, places=6)
            self.assertAlmostEqual(
                tf.transform.translation.z, out.pose.pose.position.z, places=6)

    def test_timeout_stops_valid_odom(self):
        """After an input interruption (> watchdog period) the node goes
        TIMEOUT, valid=false and stops publishing valid odometry.

        (Part 10.4)
        """
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.start_alignment_delay = 0.0
        adapter.input_timeout = 0.5
        pub = _RecordingPublisher()
        adapter._odom_pub = pub

        _MockTime._now = 0.0
        mod.LocalizationAdapter._odom_callback(
            adapter, _make_odom(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                stamp=_MockTimeInstance(1.0)))
        self.assertEqual(len(pub.published), 1)

        # Input interrupted for longer than the watchdog period.
        _MockTime._now = 3.0
        mod.LocalizationAdapter._check_timeout(adapter)
        self.assertEqual(adapter._state, _ModuleStatus.TIMEOUT)
        self.assertFalse(adapter._valid)
        # No further valid odometry published during the gap.
        self.assertEqual(len(pub.published), 1)

    def test_compliance_no_forbidden_interfaces(self):
        """The node must not use forbidden ROS interfaces or illegal libraries
        (no Gazebo/ground-truth services, no subprocess, no numpy/scipy).

        (Part 10.5)
        """
        for tok in ("rospy.ServiceProxy", "get_model_state", "gazebo_msgs",
                    "subprocess", "os.system", "import numpy", "import scipy"):
            self.assertNotIn(tok, source, f"forbidden token {tok!r} in node source")
        # The only input subscriber uses the legitimate /Odometry topic.
        self.assertEqual(source.count("rospy.Subscriber("), 2)
        self.assertIn("correction_enabled", source)
        self.assertIn("/c2r28/proprio_fastlio_shadow/odom", source)
        self.assertIn("self.odom_input_topic", source)
        self.assertIn('"/Odometry"', source)
        # Output frames are hard-wired to world/base (never odom).
        self.assertNotIn('child_frame_id = "odom"', source)

    # ------------------------------------------------------------------
    # Item 2 — non-zero body->base extrinsic transform matrix
    # ------------------------------------------------------------------

    def test_extrinsic_nonzero_body_to_base_transform(self):
        """(Item 2) A non-zero base->body extrinsic must be removed correctly.

        T_world_to_base = T_world_to_body * (T_base_to_body)^-1.
        base->body = translation [1,0,0] AND yaw +90 deg; world->body = identity
        => world->base translation == (0, 1, 0) and yaw == -90 deg, proving the
        multiplication order (body->base, not base->body).
        """
        T_base_to_body = mod._transform_from_translation_quat(
            (1.0, 0.0, 0.0), mod._quat_from_euler_rpy(0.0, 0.0, math.pi / 2.0)
        )
        T_world_to_body = mod._identity_transform()
        T_world_to_base = mod.compose_world_to_base(T_world_to_body, T_base_to_body)

        t, q = mod._transform_to_translation_quat(T_world_to_base)
        self.assertAlmostEqual(t[0], 0.0, places=6)
        self.assertAlmostEqual(t[1], 1.0, places=6)
        self.assertAlmostEqual(t[2], 0.0, places=6)
        v = mod._quat_rotate(q, (1.0, 0.0, 0.0))
        self.assertAlmostEqual(v[0], 0.0, places=6)
        self.assertAlmostEqual(v[1], -1.0, places=6)
        self.assertAlmostEqual(v[2], 0.0, places=6)

    # ------------------------------------------------------------------
    # Items 4 & 5 — scene file Safe-Fail (missing / structure / frame)
    # ------------------------------------------------------------------

    def _scene_adapter(self, path: str) -> object:
        adapter = _adapter_with_start({"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0})
        adapter.team_scene_info_path = path
        adapter.competition_mode = False
        return adapter

    @staticmethod
    def _write_scene(path: str, content) -> None:
        if isinstance(content, str):
            with open(path, "w", encoding="utf-8") as f:
                f.write(content)
        else:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(content, f)

    def test_scene_file_missing_safe_fail(self):
        """(Item 4) Missing scene file -> FAILED, valid=false, no silent origin."""
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "does_not_exist.json")
            adapter = self._scene_adapter(path)
            mod.LocalizationAdapter._load_start_pose(adapter)
            self.assertFalse(adapter._start_pose_valid)
            self.assertEqual(adapter._state, _ModuleStatus.FAILED)
            self.assertFalse(adapter._valid)
            self.assertIn("invalid", adapter._detail)

    def test_scene_json_structure_and_frame_safe_fail(self):
        """(Item 5) Bad JSON structure / wrong frame -> FAILED, valid=false."""
        bad_scenes = [
            "{ this is not json",                                   # parse error
            "[1, 2, 3]",                                            # root not object
            {"coordinate_frame": "odom",                            # wrong frame
             "robot_start": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw": 0.0}},
            {"coordinate_frame": "world"},                          # robot_start missing
            {"coordinate_frame": "world",                           # missing z field
             "robot_start": {"x": 0.0, "y": 0.0, "yaw": 0.0}},
            {"coordinate_frame": "world",                           # non-finite value
             "robot_start": {"x": float("nan"), "y": 0.0, "z": 0.0, "yaw": 0.0}},
        ]
        with tempfile.TemporaryDirectory() as tmp:
            for i, scene in enumerate(bad_scenes):
                path = os.path.join(tmp, f"scene_{i}.json")
                self._write_scene(path, scene)
                adapter = self._scene_adapter(path)
                mod.LocalizationAdapter._load_start_pose(adapter)
                self.assertFalse(adapter._start_pose_valid, f"case {i}")
                self.assertEqual(adapter._state, _ModuleStatus.FAILED, f"case {i}")
                self.assertFalse(adapter._valid, f"case {i}")


if __name__ == "__main__":
    unittest.main()
