#!/usr/bin/env python3
import threading
import time
import unittest

import rospy
import rostest
import sensor_msgs.point_cloud2 as pc2
from competition_interfaces.msg import ModuleStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


class MappingIntegrationTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._map = None
        self._status = None
        self._status_valid = True
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
            odom.header.frame_id = "odom"
            odom.child_frame_id = "base_link"
            odom.pose.pose.orientation.w = 1.0
            self._odom_pub.publish(odom)

            header = Header(stamp=stamp, frame_id="base_link")
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

    def test_map_metadata_encoding_and_localization_failure_guard(self):
        self._wait_for(lambda m, s: m is not None and s is not None and s.valid)
        with self._lock:
            first_map = self._map
            first_data = list(first_map.data)
        self.assertEqual("odom", first_map.header.frame_id)
        self.assertEqual(1.0, first_map.info.resolution)
        self.assertEqual(20, first_map.info.width)
        self.assertEqual(20, first_map.info.height)
        self.assertIn(100, first_data)
        self.assertIn(0, first_data)
        self.assertIn(-1, first_data)

        self._status_valid = False
        self._wait_for(lambda _m, s: s is not None and not s.valid and s.state == ModuleStatus.BLOCKED)
        time.sleep(0.5)
        with self._lock:
            self.assertEqual(first_data, list(self._map.data))


if __name__ == "__main__":
    rospy.init_node("mapping_integration_test")
    rostest.rosrun("competition_mapping", "mapping_integration", MappingIntegrationTest)
