#!/usr/bin/env python3
"""Deterministic unit tests for MappingNode validation and race guards."""

import importlib.util
import importlib.machinery
import os
import threading
import time
import unittest
from unittest import mock

import rospy
from competition_interfaces.msg import ModuleStatus
from nav_msgs.msg import Odometry


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
    return node


def make_odom(stamp=10.0, frame="world", child="base"):
    msg = Odometry()
    msg.header.stamp = rospy.Time(stamp)
    msg.header.frame_id = frame
    msg.child_frame_id = child
    msg.pose.pose.orientation.w = 1.0
    return msg


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
                node._enter_fault_locked("injected fault", force_new_epoch=True)
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
                node._enter_fault_locked("invalid", force_new_epoch=True)
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


if __name__ == "__main__":
    unittest.main(verbosity=2)
