#!/usr/bin/env python3
"""Prove rospy.Timer callbacks survive a simulated-time rollback."""

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
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
]


class MappingClockRollbackTest(unittest.TestCase):
    def setUp(self):
        self._lock = threading.Lock()
        self._sim_time = 100.0
        self._publish_inputs = True
        self._stop = False
        self._statuses = []
        self._maps = []
        self._clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        self._status_pub = rospy.Publisher(
            "/competition/localization/status", ModuleStatus, queue_size=1
        )
        self._odom_pub = rospy.Publisher(
            "/competition/localization/odom", Odometry, queue_size=1
        )
        self._cloud_pub = rospy.Publisher(
            "/mapping_clock/points", PointCloud2, queue_size=1
        )
        self._status_sub = rospy.Subscriber(
            "/mapping_clock/status", ModuleStatus, self._status_cb
        )
        self._map_sub = rospy.Subscriber(
            "/mapping_clock/map", OccupancyGrid, self._map_cb
        )
        self._tf = tf2_ros.TransformBroadcaster()
        self._static_tf = tf2_ros.StaticTransformBroadcaster()
        self._send_static_tf()
        self._thread = threading.Thread(target=self._loop)
        self._thread.daemon = True
        self._thread.start()

    def tearDown(self):
        self._stop = True
        self._thread.join(1.0)

    def _status_cb(self, msg):
        with self._lock:
            self._statuses.append(msg)

    def _map_cb(self, msg):
        with self._lock:
            self._maps.append(msg)

    def _send_static_tf(self):
        transform = TransformStamped()
        transform.header.stamp = rospy.Time(0)
        transform.header.frame_id = "base"
        transform.child_frame_id = "laser_livox"
        transform.transform.rotation.w = 1.0
        self._static_tf.sendTransform(transform)

    def _loop(self):
        while not rospy.is_shutdown() and not self._stop:
            with self._lock:
                self._sim_time += 0.02
                value = self._sim_time
                publish_inputs = self._publish_inputs
            stamp = rospy.Time.from_sec(value)
            self._clock_pub.publish(Clock(clock=stamp))
            if publish_inputs:
                status = ModuleStatus()
                status.header.stamp = stamp
                status.state = ModuleStatus.RUNNING
                status.valid = True
                status.confidence = 1.0
                status.detail = "clock test localization"
                self._status_pub.publish(status)

                odom = Odometry()
                odom.header.stamp = stamp
                odom.header.frame_id = "world"
                odom.child_frame_id = "base"
                odom.pose.pose.orientation.w = 1.0
                self._odom_pub.publish(odom)

                dynamic_tf = TransformStamped()
                dynamic_tf.header.stamp = stamp
                dynamic_tf.header.frame_id = "world"
                dynamic_tf.child_frame_id = "base"
                dynamic_tf.transform.rotation.w = 1.0
                self._tf.sendTransform(dynamic_tf)

                cloud = pc2.create_cloud(
                    Header(stamp=stamp, frame_id="laser_livox"),
                    FIELDS, [(3.0, 0.0, 0.5)],
                )
                self._cloud_pub.publish(cloud)
            time.sleep(0.01)

    def _wait_for(self, predicate, description, timeout=6.0):
        deadline = time.time() + timeout
        while time.time() < deadline and not rospy.is_shutdown():
            with self._lock:
                if predicate(self._statuses, self._maps):
                    return
            time.sleep(0.02)
        self.fail(description)

    def test_timers_and_mapping_recover_after_clock_rollback(self):
        self._wait_for(
            lambda statuses, maps: maps and statuses and statuses[-1].valid,
            "initial epoch never reached RUNNING",
        )
        with self._lock:
            maps_before = len(self._maps)
            history_start = len(self._statuses)
            self._publish_inputs = False
            self._sim_time = 10.0

        self._wait_for(
            lambda statuses, _maps: any(
                (not item.valid) and "moved backwards" in item.detail
                for item in statuses[history_start:]
            ),
            "rollback invalidation was not published",
        )

        with self._lock:
            self._publish_inputs = True
        self._wait_for(
            lambda statuses, maps: (
                len(maps) > maps_before
                and statuses
                and statuses[-1].valid
                and maps[-1].header.stamp.to_sec() < 20.0
            ),
            "timers did not resume or mapping did not recover in the new epoch",
        )


if __name__ == "__main__":
    rospy.init_node("mapping_clock_rollback_test")
    rostest.rosrun(
        "competition_mapping", "mapping_clock_rollback", MappingClockRollbackTest
    )
