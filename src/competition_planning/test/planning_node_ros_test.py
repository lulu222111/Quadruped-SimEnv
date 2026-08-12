#!/usr/bin/env python3

import threading
import time
import unittest

import rospy
import rostest
from competition_interfaces.msg import ModuleStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from math import atan2, cos, sin


class PlanningNodeRosTest(unittest.TestCase):
    def setUp(self):
        self.lock = threading.Lock()
        self.last_status = None
        self.last_path = None

        self.map_pub = rospy.Publisher(
            "/competition/map",
            OccupancyGrid,
            queue_size=1,
            latch=True,
        )
        self.odom_pub = rospy.Publisher(
            "/competition/localization/odom",
            Odometry,
            queue_size=1,
            latch=True,
        )

        self.status_sub = rospy.Subscriber(
            "/competition/planning/status",
            ModuleStatus,
            self.status_callback,
            queue_size=10,
        )
        self.path_sub = rospy.Subscriber(
            "/competition/path",
            Path,
            self.path_callback,
            queue_size=10,
        )

        self.wait_for_input_connections()

    def status_callback(self, msg):
        with self.lock:
            self.last_status = msg

    def path_callback(self, msg):
        with self.lock:
            self.last_path = msg

    def wait_for_input_connections(self, timeout=5.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            if (
                self.map_pub.get_num_connections() > 0
                and self.odom_pub.get_num_connections() > 0
            ):
                return

            rospy.sleep(0.05)

        self.fail(
            "planning_node did not subscribe to map and odometry"
        )

    def wait_for(self, predicate, description, timeout=5.0):
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            with self.lock:
                status = self.last_status
                path = self.last_path

            if predicate(status, path):
                return status, path

            rospy.sleep(0.05)

        self.fail("timed out waiting for " + description)

    def clear_received_outputs(self):
        with self.lock:
            self.last_status = None
            self.last_path = None

    @staticmethod
    def make_map(width, height, data, frame_id="map"):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.info.resolution = 1.0
        msg.info.width = width
        msg.info.height = height
        msg.info.origin.orientation.w = 1.0
        msg.data = data
        return msg

    @staticmethod
    def make_odom(x, y, frame_id="map"):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = frame_id
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.w = 1.0
        return msg

    def test_invalid_plan_clears_outputs_and_rejects_frame_mismatch(
        self,
    ):
        reachable_map = self.make_map(
            5,
            5,
            [
                -1, -1, -1, -1, -1,
                -1,  0,  0,  0, -1,
                -1,  0,  0,  0, -1,
                -1,  0,  0,  0, -1,
                -1, -1, -1, -1, -1,
            ],
        )

        self.map_pub.publish(reachable_map)
        self.odom_pub.publish(
            self.make_odom(2.5, 2.5)
        )

        success_status, success_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.RUNNING
                and status.valid
                and path is not None
                and len(path.poses) > 0
            ),
            "a valid non-empty path",
        )

        self.assertTrue(success_status.valid)
        self.assertGreater(len(success_path.poses), 0)

        self.clear_received_outputs()

        self.map_pub.publish(reachable_map)
        self.odom_pub.publish(
            self.make_odom(
                2.5,
                2.5,
                frame_id="odom",
            )
        )

        mismatch_status, mismatch_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.FAILED
                and not status.valid
                and "frame mismatch" in status.detail
                and path is not None
                and len(path.poses) == 0
            ),
            "FAILED status and empty path for frame mismatch",
        )

        self.assertFalse(mismatch_status.valid)
        self.assertIn(
            "map uses 'map'",
            mismatch_status.detail,
        )
        self.assertIn(
            "localization uses 'odom'",
            mismatch_status.detail,
        )
        self.assertEqual(len(mismatch_path.poses), 0)

        self.clear_received_outputs()

        unreachable_map = self.make_map(
            7,
            7,
            [
                -1, -1, -1, -1, -1, -1, -1,
                -1,  0,  0,  0,  0,  0, -1,
                -1,  0, 100, 100, 100,  0, -1,
                -1,  0, 100,   0, 100,  0, -1,
                -1,  0, 100, 100, 100,  0, -1,
                -1,  0,  0,  0,  0,  0, -1,
                -1, -1, -1, -1, -1, -1, -1,
            ],
        )

        self.map_pub.publish(unreachable_map)
        self.odom_pub.publish(
            self.make_odom(3.5, 3.5)
        )

        blocked_status, cleared_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.BLOCKED
                and not status.valid
                and path is not None
                and len(path.poses) == 0
            ),
            "BLOCKED status and an empty path",
        )

        self.assertFalse(blocked_status.valid)
        self.assertEqual(len(cleared_path.poses), 0)

        for pose_stamped in success_path.poses:
            orientation = (
                pose_stamped.pose.orientation
            )
            quaternion_norm = (
                orientation.z * orientation.z
                + orientation.w * orientation.w
            )

            self.assertAlmostEqual(
                quaternion_norm,
                1.0,
                places=6,
            )

        for index in range(
            len(success_path.poses) - 1
        ):
            current = success_path.poses[index]
            following = success_path.poses[
                index + 1
            ]

            expected_yaw = atan2(
                (
                    following.pose.position.y
                    - current.pose.position.y
                ),
                (
                    following.pose.position.x
                    - current.pose.position.x
                ),
            )

            orientation = (
                current.pose.orientation
            )
            actual_yaw = 2.0 * atan2(
                orientation.z,
                orientation.w,
            )

            angle_error = atan2(
                sin(actual_yaw - expected_yaw),
                cos(actual_yaw - expected_yaw),
            )

            self.assertAlmostEqual(
                angle_error,
                0.0,
                places=6,
            )

        if len(success_path.poses) > 1:
            previous_orientation = (
                success_path.poses[-2].pose.orientation
            )
            final_orientation = (
                success_path.poses[-1].pose.orientation
            )

            self.assertAlmostEqual(
                final_orientation.z,
                previous_orientation.z,
                places=6,
            )
            self.assertAlmostEqual(
                final_orientation.w,
                previous_orientation.w,
                places=6,
            )

        late_goal_received = threading.Event()

        self.clear_received_outputs()

        self.map_pub.publish(reachable_map)
        self.odom_pub.publish(
            self.make_odom(2.5, 2.5)
        )

        recovered_status, recovered_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.RUNNING
                and status.valid
                and path is not None
                and len(path.poses) > 0
            ),
            "a recovered valid path before timeout",
        )

        self.assertTrue(recovered_status.valid)
        self.assertGreater(
            len(recovered_path.poses),
            0,
        )

        self.clear_received_outputs()

        timeout_status, timeout_path = self.wait_for(
            lambda status, path: (
                status is not None
                and status.state == ModuleStatus.TIMEOUT
                and not status.valid
                and "stale input timeout" in status.detail
                and path is not None
                and len(path.poses) == 0
            ),
            "TIMEOUT status and empty path for stale inputs",
        )

        self.assertFalse(timeout_status.valid)
        self.assertIn(
            "map",
            timeout_status.detail,
        )
        self.assertIn(
            "localization",
            timeout_status.detail,
        )
        self.assertEqual(
            len(timeout_path.poses),
            0,
        )

        late_goal_sub = rospy.Subscriber(
            "/competition/navigation_goal",
            PoseStamped,
            lambda _msg: late_goal_received.set(),
            queue_size=1,
        )

        rospy.sleep(0.5)

        self.assertFalse(
            late_goal_received.is_set(),
            "a late subscriber received a stale navigation goal",
        )

        late_goal_sub.unregister()


if __name__ == "__main__":
    rospy.init_node(
        "planning_node_ros_test",
        anonymous=True,
    )
    rostest.rosrun(
        "competition_planning",
        "planning_node_ros_test",
        PlanningNodeRosTest,
    )