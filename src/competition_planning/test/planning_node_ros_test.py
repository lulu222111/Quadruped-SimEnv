#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from competition_interfaces.msg import ModuleStatus
from nav_msgs.msg import OccupancyGrid, Odometry, Path


class PlanningNodeContractRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.last_status = None
        self.last_path = None
        self.status_history = []
        self.path_history = []

        self.map_pub = rospy.Publisher(
            "/competition/map",
            OccupancyGrid,
            queue_size=1,
            latch=True,
        )
        self.mapping_status_pub = rospy.Publisher(
            "/competition/mapping/status",
            ModuleStatus,
            queue_size=1,
            latch=True,
        )
        self.odom_pub = rospy.Publisher(
            "/competition/localization/odom",
            Odometry,
            queue_size=1,
            latch=True,
        )
        self.localization_status_pub = rospy.Publisher(
            "/competition/localization/status",
            ModuleStatus,
            queue_size=1,
            latch=True,
        )

        self.status_sub = rospy.Subscriber(
            "/competition/planning/status",
            ModuleStatus,
            self.status_callback,
            queue_size=20,
        )
        self.path_sub = rospy.Subscriber(
            "/competition/path",
            Path,
            self.path_callback,
            queue_size=20,
        )

        self.wait_for_input_connections()

    def tearDown(self):
        self.status_sub.unregister()
        self.path_sub.unregister()

    def status_callback(self, msg):
        with self.lock:
            self.last_status = msg
            self.status_history.append(msg)

    def path_callback(self, msg):
        with self.lock:
            self.last_path = msg
            self.path_history.append(msg)

    def clear_received_outputs(self):
        with self.lock:
            self.last_status = None
            self.last_path = None
            self.status_history = []
            self.path_history = []

    def wait_for_input_connections(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if (
                self.map_pub.get_num_connections() > 0
                and self.mapping_status_pub.get_num_connections() > 0
                and self.odom_pub.get_num_connections() > 0
                and self.localization_status_pub.get_num_connections() > 0
            ):
                return
            rospy.sleep(0.05)

        self.fail(
            "planning_node did not subscribe to all four formal inputs"
        )

    def wait_for(self, predicate, description, timeout=6.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                status = self.last_status
                path = self.last_path
            if predicate(status, path):
                return status, path
            rospy.sleep(0.02)
        self.fail("timed out waiting for " + description)

    def wait_for_status_history(self, predicate, description, timeout=6.0):
        """Wait until any status received in this phase matches.

        Ingress/stamp rejections are transient: the node reports the
        fault and then falls back to waiting for current-generation
        data, so the fault status may already be overwritten by the
        time the latest status is polled. The full history makes the
        rejection observable deterministically.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                statuses = list(self.status_history)
            if predicate(statuses):
                return
            rospy.sleep(0.02)
        self.fail("timed out waiting for " + description)

    def assert_no_valid_status(self, duration=0.25):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            with self.lock:
                status = self.last_status
            if status is not None and status.valid:
                self.fail(
                    "planning recovered without current-generation data"
                )
            rospy.sleep(0.02)

    def assert_no_valid_or_nonempty_output(self, duration=0.75):
        """Ensure an invalidation cannot be followed by a stale revival."""
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            with self.lock:
                statuses = list(self.status_history)
                paths = list(self.path_history)
            self.assertFalse(
                any(status.valid for status in statuses),
                "a valid status was published after invalidation",
            )
            self.assertFalse(
                any(len(path.poses) > 0 for path in paths),
                "a non-empty path was published after invalidation",
            )
            rospy.sleep(0.02)

    @staticmethod
    def make_status(valid, detail):
        msg = ModuleStatus()
        msg.header.stamp = rospy.Time.now()
        msg.state = (
            ModuleStatus.RUNNING
            if valid
            else ModuleStatus.BLOCKED
        )
        msg.valid = valid
        msg.confidence = 1.0 if valid else 0.0
        msg.detail = detail
        return msg

    @staticmethod
    def make_map(frame_id="world"):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.info.resolution = 1.0
        msg.info.width = 5
        msg.info.height = 5
        msg.info.origin.orientation.w = 1.0
        msg.data = [
            -1, -1, -1, -1, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1,  0,  0,  0, -1,
            -1, -1, -1, -1, -1,
        ]
        return msg

    @staticmethod
    def make_large_map(size=600):
        """Create enough work to overlap planning and invalidation."""
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.info.resolution = 0.05
        msg.info.width = size
        msg.info.height = size
        msg.info.origin.orientation.w = 1.0
        data = [0] * (size * size)
        for x in range(size):
            data[x] = -1
            data[(size - 1) * size + x] = -1
        for y in range(size):
            data[y * size] = -1
            data[y * size + size - 1] = -1
        msg.data = data
        return msg

    @staticmethod
    def make_odom(frame_id="world", child_frame_id="base"):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id
        msg.pose.pose.position.x = 2.5
        msg.pose.pose.position.y = 2.5
        msg.pose.pose.orientation.w = 1.0
        return msg

    def publish_valid_statuses(self):
        self.mapping_status_pub.publish(
            self.make_status(True, "mapping ready")
        )
        self.localization_status_pub.publish(
            self.make_status(True, "localization ready")
        )

    def publish_valid_generation(self):
        self.publish_valid_statuses()
        self.map_pub.publish(self.make_map())
        self.odom_pub.publish(self.make_odom())

    @staticmethod
    def is_valid_path(status, path):
        return (
            status is not None
            and status.state == ModuleStatus.RUNNING
            and status.valid
            and path is not None
            and path.header.frame_id == "world"
            and len(path.poses) > 0
            and all(
                pose.header.frame_id == "world"
                for pose in path.poses
            )
        )

    def test_contract_gating_generation_frames_and_timeout(self):
        # No planning is allowed before both upstream statuses and both
        # current-generation data messages have arrived.
        initial_status, initial_path = self.wait_for(
            lambda status, path: (
                status is not None
                and not status.valid
                and path is not None
                and len(path.poses) == 0
            ),
            "initial invalid status and empty path",
        )
        self.assertFalse(initial_status.valid)
        self.assertEqual(initial_path.header.frame_id, "world")

        # A complete valid generation produces a usable world-frame path.
        self.clear_received_outputs()
        self.publish_valid_generation()
        success_status, success_path = self.wait_for(
            self.is_valid_path,
            "a valid path from four valid inputs",
        )
        self.assertTrue(success_status.valid)
        self.assertGreater(len(success_path.poses), 0)

        # Upstream invalidation immediately withdraws the latched path.
        self.clear_received_outputs()
        self.mapping_status_pub.publish(
            self.make_status(False, "mapping unavailable")
        )
        invalid_status, empty_path = self.wait_for(
            lambda status, path: (
                status is not None
                and not status.valid
                and "mapping status is invalid" in status.detail
                and path is not None
                and len(path.poses) == 0
            ),
            "mapping invalidation and path withdrawal",
        )
        self.assertFalse(invalid_status.valid)
        self.assertEqual(empty_path.header.frame_id, "world")

        # A valid status alone must not revive pre-invalidation data.
        self.clear_received_outputs()
        self.publish_valid_statuses()
        self.assert_no_valid_status()

        # Recovery requires new Map and Odometry in the new generation.
        self.clear_received_outputs()
        self.map_pub.publish(self.make_map())
        self.odom_pub.publish(self.make_odom())
        recovered_status, recovered_path = self.wait_for(
            self.is_valid_path,
            "recovery with current-generation map and odometry",
        )
        self.assertTrue(recovered_status.valid)
        self.assertGreater(len(recovered_path.poses), 0)

        # Formal frames are fixed to world/base, not merely equal strings.
        self.clear_received_outputs()
        self.publish_valid_statuses()
        self.map_pub.publish(self.make_map(frame_id="map"))
        self.odom_pub.publish(self.make_odom())
        frame_status, frame_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.FAILED
                and not status.valid
                and "must be 'world'" in status.detail
                and path is not None
                and len(path.poses) == 0
            ),
            "frame rejection and path withdrawal",
        )
        self.assertFalse(frame_status.valid)
        self.assertEqual(len(frame_path.poses), 0)

        # A freshly received message with an old header timestamp must
        # still be rejected by message-level freshness protection. Keep
        # the current generation alive first so the last generation
        # boundary is comfortably older than the deliberately aged
        # stamp below; otherwise the pre-generation admission check
        # would drop the message before freshness protection sees it.
        self.clear_received_outputs()
        keepalive_end = time.monotonic() + 2.8
        while time.monotonic() < keepalive_end:
            self.publish_valid_generation()
            rospy.sleep(0.2)
        self.clear_received_outputs()
        self.publish_valid_statuses()
        stale_map = self.make_map()
        stale_map.header.stamp = (
            rospy.Time.now() - rospy.Duration(2.5)
        )
        self.map_pub.publish(stale_map)
        self.odom_pub.publish(self.make_odom())
        # The rejection report is transient: after invalidating the
        # stale data the node falls back to waiting for
        # current-generation inputs, so scan the full history.
        self.wait_for_status_history(
            lambda statuses: any(
                status.state == ModuleStatus.FAILED
                and not status.valid
                and "map timestamp is" in status.detail
                and "old; limit" in status.detail
                for status in statuses
            ),
            "old map header timestamp rejection",
        )
        stale_status, stale_path = self.wait_for(
            lambda status, path: (
                status is not None
                and not status.valid
                and path is not None
                and len(path.poses) == 0
            ),
            "invalid status and empty path after stale map",
        )
        self.assertFalse(stale_status.valid)
        self.assertEqual(len(stale_path.poses), 0)
        self.assert_no_valid_or_nonempty_output(duration=0.4)

        # A future-stamped message is rejected before entering the cache.
        # Merely waiting for ROS time to catch up must not make it usable.
        self.clear_received_outputs()
        self.publish_valid_statuses()
        future_map = self.make_map()
        future_map.header.stamp = (
            rospy.Time.now() + rospy.Duration(1.0)
        )
        self.map_pub.publish(future_map)
        self.odom_pub.publish(self.make_odom())
        # The ingress rejection is reported transiently before the node
        # falls back to waiting for current-generation data.
        self.wait_for_status_history(
            lambda statuses: any(
                status.state == ModuleStatus.FAILED
                and not status.valid
                and "map timestamp is" in status.detail
                and "in the future" in status.detail
                for status in statuses
            ),
            "future map rejection before cache insertion",
        )
        future_status, future_path = self.wait_for(
            lambda status, path: (
                status is not None
                and not status.valid
                and path is not None
                and len(path.poses) == 0
            ),
            "invalid status and empty path after future map",
        )
        self.assertFalse(future_status.valid)
        self.assertEqual(len(future_path.poses), 0)

        self.clear_received_outputs()
        rospy.sleep(1.1)
        self.publish_valid_statuses()
        self.assert_no_valid_status(duration=0.35)

        # Recovery requires a newly received legal Map and Odometry.
        self.clear_received_outputs()
        self.map_pub.publish(self.make_map())
        self.odom_pub.publish(self.make_odom())
        self.wait_for(
            self.is_valid_path,
            "recovery after a new legal message",
        )

        # Invalidate while a large-map calculation may be running. Once the
        # empty path is observed, no stale non-empty path or valid status may
        # be published afterwards.
        self.clear_received_outputs()
        self.publish_valid_statuses()
        self.map_pub.publish(self.make_large_map())
        large_odom = self.make_odom()
        large_odom.pose.pose.position.x = 15.0
        large_odom.pose.pose.position.y = 15.0
        self.odom_pub.publish(large_odom)
        rospy.sleep(0.08)
        self.mapping_status_pub.publish(
            self.make_status(False, "mapping invalid during planning")
        )
        self.wait_for(
            lambda status, path: (
                status is not None
                and not status.valid
                and path is not None
                and len(path.poses) == 0
            ),
            "invalidation during planning",
        )
        self.clear_received_outputs()
        self.assert_no_valid_or_nonempty_output()

        # Recover once more, then stop status heartbeats. Status timeout
        # must happen before the longer Map/Odometry timeout.
        self.clear_received_outputs()
        self.publish_valid_generation()
        self.wait_for(
            self.is_valid_path,
            "valid path before upstream status timeout",
        )
        self.clear_received_outputs()
        timeout_status, timeout_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.TIMEOUT
                and not status.valid
                and "upstream status timeout" in status.detail
                and path is not None
                and len(path.poses) == 0
            ),
            "upstream status timeout and path withdrawal",
            timeout=3.0,
        )
        self.assertFalse(timeout_status.valid)
        self.assertEqual(timeout_path.header.frame_id, "world")


if __name__ == "__main__":
    rospy.init_node("planning_node_ros_test", anonymous=True)
    rostest.rosrun(
        "competition_planning",
        "planning_node_ros_test",
        PlanningNodeContractRosTest,
    )
