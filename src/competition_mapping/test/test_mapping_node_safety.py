#!/usr/bin/env python3
"""Deterministic unit tests for MappingNode validation, race guards and
generation-based invalidation/recovery."""

import importlib.util
import importlib.machinery
import os
import threading
import time
import unittest
from unittest import mock

import rospy
import sensor_msgs.point_cloud2 as pc2
from competition_interfaces.msg import ModuleStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointField
from std_msgs.msg import Header


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


PACKAGE_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODE_PATH = os.path.join(PACKAGE_ROOT, "scripts", "mapping_node")
SPEC = importlib.util.spec_from_loader(
    "competition_mapping_node",
    importlib.machinery.SourceFileLoader("competition_mapping_node", NODE_PATH),
)
NODE_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(NODE_MODULE)
MappingNode = NODE_MODULE.MappingNode


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class BlockingPublisher(FakePublisher):
    def __init__(self):
        super().__init__()
        self.entered = threading.Event()
        self.release = threading.Event()

    def publish(self, msg):
        self.entered.set()
        self.release.wait(2.0)
        super().publish(msg)


def make_bare_node():
    node = MappingNode.__new__(MappingNode)
    node._lock = threading.RLock()
    node._map_frame = "world"
    node._expected_base_frame = "base"
    node._expected_cloud_frame = "laser_livox"
    node._quaternion_tolerance = 0.1
    node._future_tolerance = 0.2
    node._max_input_age = 2.0
    node._time_rollback_tolerance = 0.001
    node._last_ros_time = None
    node._input_epoch = 4
    node._map_version = 7
    node._fault_active = False
    node._fault_state = ModuleStatus.BLOCKED
    node._fault_detail = ""
    node._healthy_inputs_since = None
    node._localization_valid = True
    node._status_epoch = 4
    node._last_status_received_at = rospy.Time(10.0)
    node._last_status_stamp = rospy.Time(10.0)
    node._odom_valid = True
    node._odom_epoch = 4
    node._last_odom_received_at = rospy.Time(10.0)
    node._last_odom_stamp = rospy.Time(10.0)
    node._latest_pose = (0.0, 0.0, 0.0)
    node._accepted_pose = (0.0, 0.0, 0.0)
    node._pending_jump_pose = None
    node._jump_stable_count = 0
    node._jump_fault_active = False
    node._max_pose_jump = 2.0
    node._max_yaw_jump = 1.57
    node._pose_recovery_samples = 3
    node._reset_map_on_pose_jump = True
    node._pending_cloud = None
    node._last_cloud_received_at = rospy.Time(10.0)
    node._last_cloud_accepted_stamp = rospy.Time(10.0)
    node._last_cloud_integrated_stamp = rospy.Time(10.0)
    node._last_cloud_integrated_at = rospy.Time(10.0)
    node._last_integrated_epoch = 4
    node._map_initialized = True
    node._map_load_time = rospy.Time(9.0)
    node._origin_mode = "fixed"
    node._origin_ready = True
    node._origin_x = -10.0
    node._origin_y = -10.0
    node._resolution = 1.0
    node._map_width = 20
    node._map_height = 20
    node._status_timeout = 1.0
    node._odom_timeout = 1.0
    node._pointcloud_timeout = 1.0
    node._last_scan_stats = {
        "valid_points": 1, "occupied_endpoints": 1, "changed_cells": 2
    }
    node._last_reported_status = None
    node._status_pub = FakePublisher()
    node._map_pub = FakePublisher()
    node._grid = mock.Mock()
    node._grid.data_copy.return_value = [-1] * 400
    node._grid.reset = mock.Mock()
    node._grid.integrate_scan.return_value = {
        "input_points": 1, "total_points": 1, "non_finite_rejected": 0,
        "range_rejected": 0, "height_rejected": 0, "valid_points": 1,
        "rejected_points": 0, "occupied_endpoints": 1,
        "free_cells_updated": 1, "changed_cells": 2, "robot_outside_map": 0,
    }
    node._point_stride = 1
    node._max_points = 50000
    node._tf_timeout = 0.1
    node._tf_buffer = make_tf_buffer()
    node._cloud_metrics = {
        "clouds_received": 0,
        "clouds_replaced": 0,
        "clouds_rejected": 0,
        "clouds_integrated": 0,
        "last_integration_duration": 0.0,
        "last_valid_point_count": 0,
    }
    node._pointcloud_failure_state = ModuleStatus.RUNNING
    node._pointcloud_detail = "point cloud integrated"
    return node


def make_odom(stamp=10.0, frame="world", child="base"):
    msg = Odometry()
    msg.header.stamp = rospy.Time(stamp)
    msg.header.frame_id = frame
    msg.child_frame_id = child
    msg.pose.pose.orientation.w = 1.0
    return msg


def make_status(stamp=10.1, valid=True, state=ModuleStatus.RUNNING,
                detail="localization valid"):
    msg = ModuleStatus()
    msg.header.stamp = rospy.Time(stamp)
    msg.header.frame_id = "world"
    msg.valid = valid
    msg.state = state
    msg.confidence = 1.0 if valid else 0.0
    msg.detail = detail
    return msg


def make_cloud(stamp=10.3, frame="laser_livox"):
    header = Header(stamp=rospy.Time(stamp), frame_id=frame)
    return pc2.create_cloud(header, FIELDS, [(3.0, 0.0, 0.5)])


def make_tf_buffer():
    """Fake tf2 buffer returning identity transforms at the requested stamp."""

    def lookup(frame_id, child_id, stamp, timeout):
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = frame_id
        transform.child_frame_id = child_id
        transform.transform.rotation.w = 1.0
        return transform

    buffer = mock.Mock()
    buffer.lookup_transform.side_effect = lookup
    return buffer


class MappingNodeSafetyTest(unittest.TestCase):
    def test_odometry_frame_contract(self):
        node = make_bare_node()
        valid, state, detail, _ = node._validate_odometry_message(
            make_odom(frame="odom")
        )
        self.assertFalse(valid)
        self.assertEqual(ModuleStatus.BLOCKED, state)
        self.assertIn("frame 'odom'", detail)

        valid, state, detail, _ = node._validate_odometry_message(
            make_odom(child="base_link")
        )
        self.assertFalse(valid)
        self.assertEqual(ModuleStatus.BLOCKED, state)
        self.assertIn("child frame 'base_link'", detail)

    def test_zero_future_stale_duplicate_and_out_of_order_stamps(self):
        node = make_bare_node()
        now = rospy.Time(10.0)
        cases = [
            (rospy.Time(), None, None, "zero"),
            (rospy.Time(10.21), None, None, "future"),
            (rospy.Time(7.99), None, None, "old"),
            (rospy.Time(9.0), rospy.Time(9.0), None, "duplicate"),
            (rospy.Time(8.9), rospy.Time(9.0), None, "out of order"),
        ]
        for stamp, previous, max_age, expected in cases:
            valid, detail = node._validate_stamp_locked(
                "input", stamp, now, previous, max_age=max_age
            )
            self.assertFalse(valid)
            self.assertIn(expected, detail)

    def test_time_rollback_clears_epoch_inputs_and_map(self):
        node = make_bare_node()
        node._last_ros_time = rospy.Time(10.0)
        node._check_ros_time_locked(rospy.Time(9.0))
        self.assertEqual(5, node._input_epoch)
        self.assertFalse(node._localization_valid)
        self.assertFalse(node._odom_valid)
        self.assertIsNone(node._pending_cloud)
        self.assertFalse(node._map_initialized)
        self.assertEqual(8, node._map_version)
        node._grid.reset.assert_called_once_with()
        self.assertIn("moved backwards", node._fault_detail)

    def test_map_snapshot_is_not_published_after_epoch_changes(self):
        node = make_bare_node()
        build_entered = threading.Event()
        release_build = threading.Event()

        def blocked_build(*args):
            build_entered.set()
            release_build.wait(2.0)
            return mock.Mock(header=mock.Mock(stamp=None))

        node._build_map_message = blocked_build
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.1)):
            worker = threading.Thread(target=node._map_timer_callback, args=(None,))
            worker.start()
            self.assertTrue(build_entered.wait(1.0))
            with node._lock:
                node._enter_fault_locked("injected fault")
            release_build.set()
            worker.join(2.0)

        self.assertFalse(worker.is_alive())
        self.assertEqual([], node._map_pub.messages)

    def test_invalid_callback_cannot_complete_before_valid_status_publish(self):
        node = make_bare_node()
        publisher = BlockingPublisher()
        node._status_pub = publisher
        invalid_done = threading.Event()

        def invalidate():
            with node._lock:
                node._enter_fault_locked("invalid")
            invalid_done.set()

        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.1)):
            status_thread = threading.Thread(
                target=node._status_timer_callback, args=(None,)
            )
            status_thread.start()
            self.assertTrue(publisher.entered.wait(1.0))
            invalid_thread = threading.Thread(target=invalidate)
            invalid_thread.start()
            time.sleep(0.05)
            self.assertFalse(invalid_done.is_set())
            publisher.release.set()
            status_thread.join(2.0)
            invalid_thread.join(2.0)

        self.assertTrue(invalid_done.is_set())
        self.assertEqual(1, len(publisher.messages))
        self.assertTrue(publisher.messages[0].valid)


class MappingNodeRecoveryContractTest(unittest.TestCase):
    """Deterministic regression tests for the v0.2.0 generation contract.

    Covers: single generation transition on invalidation, candidate caching
    while invalid, order-independent recovery, pending-cloud freshness
    recheck, valid-only usability, fault state/detail following the latest
    fault report, and odometry faults invalidating cached candidates.
    """

    def test_repeated_invalid_heartbeats_open_only_one_generation(self):
        node = make_bare_node()
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.5)):
            node._localization_status_callback(
                make_status(stamp=10.1, valid=False, state=ModuleStatus.BLOCKED)
            )
        self.assertEqual(5, node._input_epoch)
        self.assertTrue(node._fault_active)
        self.assertFalse(node._localization_valid)

        # Current-generation candidates cached after the transition must
        # survive continued invalid heartbeats.
        with node._lock:
            node._odom_valid = True
            node._odom_epoch = node._input_epoch
            node._pending_cloud = (make_cloud(stamp=10.2), node._input_epoch)

        for offset in range(5):
            with mock.patch.object(
                rospy.Time, "now", return_value=rospy.Time(10.6 + offset)
            ):
                node._localization_status_callback(
                    make_status(
                        stamp=10.2 + offset,
                        valid=False,
                        state=ModuleStatus.BLOCKED,
                    )
                )
            self.assertEqual(5, node._input_epoch)
        self.assertFalse(node._localization_valid)
        self.assertTrue(node._odom_valid)
        self.assertEqual(5, node._odom_epoch)
        self.assertIsNotNone(node._pending_cloud)
        self.assertEqual(5, node._pending_cloud[1])

    def test_recovery_order_cloud_cached_then_localization_valid(self):
        node = make_bare_node()
        clock = {"now": 10.5}
        with mock.patch.object(
            rospy.Time, "now", side_effect=lambda: rospy.Time(clock["now"])
        ):
            node._localization_status_callback(make_status(stamp=10.1, valid=False))
            self.assertEqual(5, node._input_epoch)

            # Order 1: current-generation odometry and cloud arrive while
            # localization is still invalid.
            node._odom_callback(make_odom(stamp=10.2))
            self.assertTrue(node._odom_valid)
            self.assertEqual(5, node._odom_epoch)
            self.assertFalse(node._localization_valid)

            node._pointcloud_callback(make_cloud(stamp=10.3))
            self.assertIsNotNone(node._pending_cloud)
            self.assertEqual(5, node._pending_cloud[1])

            # The integration timer must not consume the candidate while
            # localization is invalid.
            node._integration_callback(None)
            self.assertIsNotNone(node._pending_cloud)
            node._grid.integrate_scan.assert_not_called()
            self.assertTrue(node._fault_active)

            # Localization recovers: the already cached candidates complete
            # the recovery without any further messages.
            node._localization_status_callback(
                make_status(stamp=10.4, valid=True, state=ModuleStatus.RUNNING)
            )
            node._integration_callback(None)

        self.assertFalse(node._fault_active)
        node._grid.integrate_scan.assert_called_once()
        self.assertEqual(5, node._last_integrated_epoch)
        state, valid, _confidence, _detail = node._evaluate_status_locked(
            rospy.Time(10.5)
        )
        self.assertTrue(valid)
        self.assertEqual(ModuleStatus.RUNNING, state)

    def test_recovery_order_localization_valid_then_new_inputs(self):
        node = make_bare_node()
        clock = {"now": 10.5}
        with mock.patch.object(
            rospy.Time, "now", side_effect=lambda: rospy.Time(clock["now"])
        ):
            node._localization_status_callback(make_status(stamp=10.1, valid=False))
            self.assertEqual(5, node._input_epoch)

            # Order 2: localization becomes valid first, then fresh
            # odometry and cloud arrive.
            node._localization_status_callback(
                make_status(stamp=10.2, valid=True, state=ModuleStatus.RUNNING)
            )
            self.assertTrue(node._localization_valid)
            self.assertTrue(node._fault_active)

            node._odom_callback(make_odom(stamp=10.3))
            self.assertTrue(node._odom_valid)
            self.assertEqual(5, node._odom_epoch)

            node._pointcloud_callback(make_cloud(stamp=10.4))
            node._integration_callback(None)

        self.assertFalse(node._fault_active)
        node._grid.integrate_scan.assert_called_once()
        self.assertEqual(5, node._last_integrated_epoch)

    def test_stale_candidate_cloud_is_not_fused_after_recovery(self):
        node = make_bare_node()
        clock = {"now": 10.0}
        with mock.patch.object(
            rospy.Time, "now", side_effect=lambda: rospy.Time(clock["now"])
        ):
            node._localization_status_callback(make_status(stamp=10.05, valid=False))
            self.assertEqual(5, node._input_epoch)

            clock["now"] = 10.1
            node._pointcloud_callback(make_cloud(stamp=10.1))
            self.assertIsNotNone(node._pending_cloud)

            # Long past the pointcloud freshness timeout, localization
            # recovers.  The cached candidate must not be fused.
            clock["now"] = 11.3
            node._localization_status_callback(
                make_status(stamp=11.2, valid=True, state=ModuleStatus.RUNNING)
            )
            node._odom_callback(make_odom(stamp=11.25))
            node._integration_callback(None)

            self.assertIsNone(node._pending_cloud)
            node._grid.integrate_scan.assert_not_called()
            self.assertTrue(node._fault_active)
            self.assertIn("old", node._pointcloud_detail)

            # A fresh current-generation cloud completes the recovery.
            clock["now"] = 11.4
            node._pointcloud_callback(make_cloud(stamp=11.4))
            clock["now"] = 11.5
            node._integration_callback(None)

        self.assertFalse(node._fault_active)
        node._grid.integrate_scan.assert_called_once()
        self.assertEqual(rospy.Time(11.4), node._last_cloud_integrated_stamp)

    def test_non_running_state_with_valid_true_stays_usable(self):
        node = make_bare_node()
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.5)):
            node._localization_status_callback(
                make_status(
                    stamp=10.1,
                    valid=True,
                    state=ModuleStatus.READY,
                    detail="localization ready",
                )
            )
        self.assertTrue(node._localization_valid)
        self.assertFalse(node._fault_active)
        state, valid, _confidence, _detail = node._evaluate_status_locked(
            rospy.Time(10.5)
        )
        self.assertTrue(valid)
        self.assertEqual(ModuleStatus.RUNNING, state)

    def test_fault_state_and_detail_follow_latest_fault_report(self):
        node = make_bare_node()
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.5)):
            node._localization_status_callback(
                make_status(
                    stamp=10.1,
                    valid=False,
                    state=ModuleStatus.TIMEOUT,
                    detail="localization timed out internally",
                )
            )
        self.assertEqual(ModuleStatus.TIMEOUT, node._fault_state)
        self.assertEqual(
            "localization unavailable: localization timed out internally",
            node._fault_detail,
        )

        # A later BLOCKED heartbeat updates state and detail as a pair: the
        # previously reported TIMEOUT must not persist alongside the new
        # BLOCKED detail (and vice versa).
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.6)):
            node._localization_status_callback(
                make_status(
                    stamp=10.2,
                    valid=False,
                    state=ModuleStatus.BLOCKED,
                    detail="localization blocked by user",
                )
            )
        self.assertEqual(ModuleStatus.BLOCKED, node._fault_state)
        self.assertEqual(
            "localization unavailable: localization blocked by user",
            node._fault_detail,
        )

        # A FAILED report likewise replaces both fields.
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.7)):
            node._localization_status_callback(
                make_status(
                    stamp=10.3,
                    valid=False,
                    state=ModuleStatus.FAILED,
                    detail="sensor failure",
                )
            )
        self.assertEqual(ModuleStatus.FAILED, node._fault_state)
        self.assertEqual(
            "localization unavailable: sensor failure",
            node._fault_detail,
        )

        # Without a new fault event the state/detail pair persists unchanged
        # and is reported as-is (never squashed to BLOCKED).
        state, valid, _confidence, detail = node._evaluate_status_locked(
            rospy.Time(10.8)
        )
        self.assertEqual(ModuleStatus.FAILED, state)
        self.assertFalse(valid)
        self.assertEqual(node._fault_detail, detail)

        # Repeated heartbeats never opened extra generations.
        self.assertEqual(5, node._input_epoch)

    def test_odom_fault_invalidates_cached_candidate_and_forces_new_generation(self):
        """Deterministic regression: invalid generation -> valid Odom A ->
        invalid Odom B -> localization valid.

        Old candidate A must never satisfy recovery: the odometry fault has
        to invalidate A and force a new generation, and recovery must wait for
        a fresh valid odometry.  Covers invalid timestamp, invalid frame,
        non-finite pose and pose-jump odometry faults.
        """
        variants = [
            ("invalid timestamp", lambda msg: setattr(
                msg.header, "stamp", rospy.Time())),
            ("invalid frame", lambda msg: setattr(
                msg.header, "frame_id", "odom")),
            ("non-finite pose", lambda msg: setattr(
                msg.pose.pose.position, "x", float("nan"))),
            ("pose jump", lambda msg: setattr(
                msg.pose.pose.position, "x", 5.0)),
        ]
        for label, mutate in variants:
            node = make_bare_node()
            clock = {"now": 10.5}
            with mock.patch.object(
                rospy.Time, "now", side_effect=lambda: rospy.Time(clock["now"])
            ):
                node._localization_status_callback(
                    make_status(stamp=10.1, valid=False)
                )
                self.assertEqual(5, node._input_epoch, label)

                # Valid Odom A cached in the invalid generation.
                node._odom_callback(make_odom(stamp=10.2))
                self.assertTrue(node._odom_valid, label)
                self.assertEqual(5, node._odom_epoch, label)
                self.assertEqual((0.0, 0.0, 0.0), node._latest_pose, label)

                # Invalid Odom B must kill candidate A and force a new
                # generation even though a fault generation is active.
                bad = make_odom(stamp=10.3)
                mutate(bad)
                node._odom_callback(bad)
                self.assertFalse(node._odom_valid, label)
                self.assertIsNone(node._latest_pose, label)
                self.assertEqual(6, node._input_epoch, label)

                # Localization recovers: old A must not satisfy recovery.
                clock["now"] = 10.5
                node._localization_status_callback(
                    make_status(stamp=10.4, valid=True, state=ModuleStatus.RUNNING)
                )
                allowed, reason = node._integration_allowed_locked(
                    rospy.Time(10.6), node._input_epoch
                )
                self.assertFalse(allowed, label)
                self.assertIn("odometry", reason, label)

                # A fresh valid odometry completes the odometry half of
                # recovery.  Pose jumps need _pose_recovery_samples stable
                # samples before the guard accepts the new pose.
                samples = 3 if label == "pose jump" else 1
                for offset in range(samples):
                    clock["now"] = 10.6 + 0.1 * offset
                    node._odom_callback(
                        make_odom(stamp=10.5 + 0.1 * offset)
                    )
                self.assertTrue(node._odom_valid, label)
                self.assertEqual(6, node._odom_epoch, label)
                allowed, reason = node._integration_allowed_locked(
                    rospy.Time(clock["now"] + 0.05), node._input_epoch
                )
                self.assertTrue(allowed, "{}: {}".format(label, reason))

    def test_odom_timeout_invalidates_cached_candidate(self):
        node = make_bare_node()
        # Healthy node with a cached odometry candidate.  Status stays fresh
        # so the odometry timeout branch is reached.
        node._last_status_received_at = rospy.Time(11.0)
        node._last_status_stamp = rospy.Time(11.0)
        self.assertTrue(node._odom_valid)
        self.assertEqual((0.0, 0.0, 0.0), node._latest_pose)

        state, valid, _confidence, detail = node._evaluate_status_locked(
            rospy.Time(11.6)
        )
        self.assertEqual(ModuleStatus.TIMEOUT, state)
        self.assertFalse(valid)
        self.assertIn("odometry timed out", detail)
        # The timeout forced a new generation and killed the candidate.
        self.assertEqual(5, node._input_epoch)
        self.assertFalse(node._odom_valid)
        self.assertIsNone(node._latest_pose)
        self.assertTrue(node._fault_active)

        # A fresh odometry is required again.
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(11.7)):
            node._odom_callback(make_odom(stamp=11.6))
        self.assertTrue(node._odom_valid)
        self.assertEqual(5, node._odom_epoch)

    def test_stale_cached_odom_does_not_satisfy_recovery(self):
        """Mid-fault odometry timeout: a cached candidate whose freshness
        expired must not satisfy recovery even though _odom_valid is True."""
        node = make_bare_node()
        clock = {"now": 10.5}
        with mock.patch.object(
            rospy.Time, "now", side_effect=lambda: rospy.Time(clock["now"])
        ):
            node._localization_status_callback(make_status(stamp=10.1, valid=False))
            node._odom_callback(make_odom(stamp=10.2))
            self.assertTrue(node._odom_valid)

            # Odometry stops arriving; time passes beyond odom_timeout.
            clock["now"] = 11.5
            node._localization_status_callback(
                make_status(stamp=11.4, valid=True, state=ModuleStatus.RUNNING)
            )
            allowed, reason = node._integration_allowed_locked(
                rospy.Time(11.6), node._input_epoch
            )
            self.assertFalse(allowed)
            self.assertIn("odometry timed out", reason)

            # A fresh odometry restores the odometry half of recovery.
            clock["now"] = 11.7
            node._odom_callback(make_odom(stamp=11.6))
            allowed, reason = node._integration_allowed_locked(
                rospy.Time(11.8), node._input_epoch
            )
            self.assertTrue(allowed, reason)

    def test_integration_clears_old_generation_pending_cloud(self):
        node = make_bare_node()
        with node._lock:
            node._pending_cloud = (make_cloud(stamp=10.3), node._input_epoch - 1)
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.5)):
            node._integration_callback(None)
        self.assertIsNone(node._pending_cloud)
        node._grid.integrate_scan.assert_not_called()
        self.assertEqual(1, node._cloud_metrics["clouds_rejected"])

    def test_pending_cloud_remains_latest_only(self):
        node = make_bare_node()
        with mock.patch.object(rospy.Time, "now", return_value=rospy.Time(10.5)):
            node._pointcloud_callback(make_cloud(stamp=10.2))
            first = node._pending_cloud
            node._pointcloud_callback(make_cloud(stamp=10.3))
        self.assertIsNot(node._pending_cloud, first)
        self.assertEqual(rospy.Time(10.3), node._pending_cloud[0].header.stamp)
        self.assertEqual(1, node._cloud_metrics["clouds_replaced"])


if __name__ == "__main__":
    unittest.main(verbosity=2)
