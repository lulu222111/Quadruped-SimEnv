#!/usr/bin/env python3
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


class MappingIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._map = None
        self._status = None
        self._status_valid = True
        self._publish_cloud = True
        self._stop = False
        self._status_pub = rospy.Publisher(
            "/competition/localization/status", ModuleStatus, queue_size=1, latch=True
        )
        self._odom_pub = rospy.Publisher(
            "/competition/localization/odom", Odometry, queue_size=1
        )
        self._cloud_pub = rospy.Publisher("/test/points", PointCloud2, queue_size=1)
        self._map_sub = rospy.Subscriber("/competition/map", OccupancyGrid, self._map_cb)
        self._mapping_status_sub = rospy.Subscriber(
            "/competition/mapping/status", ModuleStatus, self._status_cb
        )
        self._tf = tf2_ros.TransformBroadcaster()
        self._static_tf = tf2_ros.StaticTransformBroadcaster()
        self._publish_static_tf()
        self._thread = threading.Thread(target=self._publish_inputs)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._stop = True
        self._thread.join(timeout=1.0)

    def _map_cb(self, msg):
        with self._lock:
            self._map = msg

    def _status_cb(self, msg):
        with self._lock:
            self._status = msg

    def _publish_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = "base"
        transform.child_frame_id = "laser_livox"
        transform.transform.rotation.w = 1.0
        self._static_tf.sendTransform(transform)

    def _publish_inputs(self):
        rate = rospy.Rate(10)
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        while not rospy.is_shutdown() and not self._stop:
            stamp = rospy.Time.now()

            status = ModuleStatus()
            status.header.stamp = stamp
            status.state = ModuleStatus.RUNNING if self._status_valid else ModuleStatus.BLOCKED
            status.valid = self._status_valid
            status.confidence = 1.0 if self._status_valid else 0.0
            status.detail = "synthetic valid localization" if self._status_valid else "injected invalid"
            self._status_pub.publish(status)

            odom = Odometry()
            odom.header.stamp = stamp
            odom.header.frame_id = "world"
            odom.child_frame_id = "base"
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)

            # Deliberately differs from Odometry x=0. The expected occupied cell
            # proves Mapping uses the point-cloud timestamp TF pose (x=1).
            dynamic_tf = TransformStamped()
            dynamic_tf.header.stamp = stamp
            dynamic_tf.header.frame_id = "world"
            dynamic_tf.child_frame_id = "base"
            dynamic_tf.transform.translation.x = 1.0
            dynamic_tf.transform.rotation.w = 1.0
            self._tf.sendTransform(dynamic_tf)

            if self._publish_cloud:
                header = Header(stamp=stamp, frame_id="laser_livox")
                cloud = pc2.create_cloud(header, fields, [(3.0, 0.0, 0.5)])
                self._cloud_pub.publish(cloud)
            rate.sleep()

    def _wait_for(self, predicate, timeout=8.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate(self._map, self._status):
                    return
            time.sleep(0.05)
        self.fail("condition not reached within {:.1f}s".format(timeout))

    def test_timestamped_tf_metadata_and_epoch_recovery(self):
        self._wait_for(lambda m, s: m is not None and s is not None and s.valid)
        with self._lock:
            first_map = self._map
            first_data = list(first_map.data)
        self.assertEqual("world", first_map.header.frame_id)
        self.assertEqual(1.0, first_map.info.resolution)
        self.assertEqual(20, first_map.info.width)
        self.assertEqual(20, first_map.info.height)
        self.assertEqual(-10.0, first_map.info.origin.position.x)
        self.assertIn(100, first_data)
        self.assertIn(0, first_data)
        self.assertIn(-1, first_data)
        self.assertEqual(100, first_data[10 * 20 + 14])
        self.assertNotEqual(100, first_data[10 * 20 + 13])

        self._status_valid = False
        self._wait_for(lambda _m, s: s is not None and not s.valid)
        time.sleep(0.4)
        with self._lock:
            # While localization is invalid the old latched map must not be
            # wrongly fused into: its published cells stay unchanged.
            self.assertEqual(first_data, list(self._map.data))

        # Data-first recovery (generation contract): current-generation
        # odometry and clouds arriving while localization is invalid are
        # cached as candidates but not consumed.  Stop the cloud publisher and
        # let localization recover; no further cloud is sent, yet the mapping
        # must recover valid=true by consuming the cached fresh
        # current-generation candidate.
        self._publish_cloud = False
        self._status_valid = True
        self._wait_for(lambda _m, s: s is not None and s.valid)


if __name__ == "__main__":
    rospy.init_node("mapping_integration_test")
    rostest.rosrun("competition_mapping", "mapping_integration", MappingIntegrationTest)
