#!/usr/bin/env python3
"""Black-box fault injection and automatic-recovery tests."""

import threading
import time
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
from competition_interfaces.msg import ModuleStatus
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


class MappingFaultsTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._latest_status = None
        self._latest_map = None
        self._status_history = []
        self._map_count = 0
        self._publish_status = True
        self._publish_odom = True
        self._publish_cloud = True
        self._status_valid = True
        self._cloud_frame = "laser_livox"
        self._send_sensor_tf = False
        self._stop = False

        self._localization_status_pub = rospy.Publisher(
            "/competition/localization/status", ModuleStatus, queue_size=1
        )
        self._odom_pub = rospy.Publisher(
            "/competition/localization/odom", Odometry, queue_size=1
        )
        self._cloud_pub = rospy.Publisher(
            "/mapping_faults/points", PointCloud2, queue_size=1
        )
        self._status_sub = rospy.Subscriber(
            "/mapping_faults/status", ModuleStatus, self._status_cb
        )
        self._map_sub = rospy.Subscriber(
            "/mapping_faults/map", OccupancyGrid, self._map_cb
        )
        self._tf = tf2_ros.TransformBroadcaster()
        self._static_tf = tf2_ros.StaticTransformBroadcaster()
        self._thread = threading.Thread(target=self._input_loop)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._stop = True
        self._thread.join(1.0)

    def _status_cb(self, msg):
        with self._lock:
            self._latest_status = msg
            self._status_history.append(msg)

    def _map_cb(self, msg):
        with self._lock:
            self._latest_map = msg
            self._map_count += 1

    def _publish_sensor_tf(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "base"
        transform.child_frame_id = "laser_livox"
        transform.transform.rotation.w = 1.0
        self._static_tf.sendTransform(transform)

    def _input_loop(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown() and not self._stop:
            stamp = rospy.Time.now()
            if self._send_sensor_tf:
                self._publish_sensor_tf()
                self._send_sensor_tf = False

            world_base = TransformStamped()
            world_base.header.stamp = stamp
            world_base.header.frame_id = "world"
            world_base.child_frame_id = "base"
            world_base.transform.rotation.w = 1.0
            self._tf.sendTransform(world_base)

            if self._publish_status:
                status = ModuleStatus()
                status.header.stamp = stamp
                status.state = (
                    ModuleStatus.RUNNING if self._status_valid else ModuleStatus.BLOCKED
                )
                status.valid = self._status_valid
                status.confidence = 1.0 if self._status_valid else 0.0
                status.detail = "healthy" if self._status_valid else "injected invalid"
                self._localization_status_pub.publish(status)

            if self._publish_odom:
                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "world"
                odom.child_frame_id = "base"
                odom.pose.pose.orientation.w = 1.0
                self._odom_pub.publish(odom)

            if self._publish_cloud:
                cloud = pc2.create_cloud(
                    Header(stamp=stamp, frame_id=self._cloud_frame),
                    FIELDS, [(3.0, 0.0, 0.5)],
                )
                self._cloud_pub.publish(cloud)
            rate.sleep()

    def _wait_status(self, predicate, description, timeout=5.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                status = self._latest_status
                if status is not None and predicate(status):
                    return status
            time.sleep(0.02)
        with self._lock:
            detail = None if self._latest_status is None else self._latest_status.detail
        self.fail("{}; latest status={!r}".format(description, detail))

    def _wait_running(self):
        return self._wait_status(
            lambda s: bool(s.valid) and s.state == ModuleStatus.RUNNING,
            "mapping did not reach RUNNING",
        )

    def _wait_map(self, predicate, description, timeout=5.0):
        """Bounded wait on the lock-protected _latest_map.

        Status and map are asynchronous publish/subscribe paths: RUNNING may
        be observed up to one map publish period before the map arrives.
        Poll the predicate instead of asserting immediately.
        """
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                current_map = self._latest_map
                if current_map is not None and predicate(current_map):
                    return current_map
            time.sleep(0.02)
        with self._lock:
            received = self._latest_map is not None
        self.fail("{}; map received={}".format(description, received))

    def _recover(self):
        self._publish_status = True
        self._publish_odom = True
        self._publish_cloud = True
        self._status_valid = True
        self._cloud_frame = "laser_livox"
        return self._wait_running()

    def test_faults_and_recovery(self):
        # A valid frame name is not enough: timestamped base->laser_livox TF
        # must exist before any cloud can be integrated.
        missing_tf = self._wait_status(
            lambda s: not s.valid and "TF unavailable" in s.detail,
            "missing sensor TF was not reported",
        )
        self.assertFalse(missing_tf.valid)

        self._send_sensor_tf = True
        self._recover()
        first_map = self._wait_map(
            lambda m: 100 in m.data,
            "first map with occupied cells was not published",
        )
        self.assertIn(100, first_map.data)

        self._status_valid = False
        invalid = self._wait_status(
            lambda s: not s.valid and "injected invalid" in s.detail,
            "invalid localization was not propagated",
        )
        self.assertFalse(invalid.valid)
        self._recover()

        self._cloud_frame = "wrong_lidar"
        wrong_frame = self._wait_status(
            lambda s: not s.valid and "point cloud" in s.detail and "timed out" in s.detail,
            "wrong point-cloud frames were not rejected (map stayed valid)",
        )
        self.assertFalse(wrong_frame.valid)
        self._cloud_frame = "laser_livox"
        self._recover()

        self._publish_status = False
        status_timeout = self._wait_status(
            lambda s: not s.valid and "status timed out" in s.detail,
            "localization status timeout was not reported",
        )
        self.assertEqual(ModuleStatus.TIMEOUT, status_timeout.state)
        self._recover()

        self._publish_odom = False
        odom_timeout = self._wait_status(
            lambda s: not s.valid and "odometry timed out" in s.detail,
            "odometry timeout was not reported",
        )
        self.assertEqual(ModuleStatus.TIMEOUT, odom_timeout.state)
        self._recover()

        self._publish_cloud = False
        cloud_timeout = self._wait_status(
            lambda s: not s.valid and "point cloud" in s.detail and "timed out" in s.detail,
            "point-cloud timeout was not reported",
        )
        self.assertEqual(ModuleStatus.TIMEOUT, cloud_timeout.state)
        with self._lock:
            maps_after_cloud_timeout = self._map_count
        time.sleep(0.25)
        with self._lock:
            self.assertEqual(maps_after_cloud_timeout, self._map_count)
        self._recover()


if __name__ == "__main__":
    rospy.init_node("mapping_faults_test")
    rostest.rosrun("competition_mapping", "mapping_faults", MappingFaultsTest)
