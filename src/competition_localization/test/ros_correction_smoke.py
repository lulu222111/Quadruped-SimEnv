#!/usr/bin/env python3
"""Short live-ROS smoke for correction fail-close and fresh recovery."""

import math
import time

import rospy
import tf2_ros
from competition_interfaces.msg import ModuleStatus
from nav_msgs.msg import Odometry


class CorrectionSmoke:
    def __init__(self):
        self.status = None
        self.odometry = []
        self.fast_pub = rospy.Publisher("/test/fastlio", Odometry, queue_size=10)
        self.correction_pub = rospy.Publisher(
            "/test/correction", Odometry, queue_size=10
        )
        rospy.Subscriber(
            "/competition/localization/status", ModuleStatus,
            self._status_callback, queue_size=10,
        )
        rospy.Subscriber(
            "/competition/localization/odom", Odometry,
            self._odom_callback, queue_size=20,
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def _status_callback(self, message):
        self.status = message

    def _odom_callback(self, message):
        self.odometry.append(message)

    @staticmethod
    def _message(stamp, correction=False, x=0.0):
        message = Odometry()
        message.header.stamp = stamp
        message.header.frame_id = (
            "c2r28_fastlio_local" if correction else "camera_init"
        )
        message.child_frame_id = "c2r28_base" if correction else "body"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        return message

    def publish_fast(self, x=0.0):
        stamp = rospy.Time.now()
        self.fast_pub.publish(self._message(stamp, x=x))
        return stamp

    def publish_correction(self, x=0.0):
        stamp = rospy.Time.now()
        self.correction_pub.publish(self._message(stamp, correction=True, x=x))
        return stamp

    @staticmethod
    def wait_for(predicate, timeout, detail):
        deadline = time.monotonic() + timeout
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if predicate():
                return
            rospy.sleep(0.02)
        raise RuntimeError("smoke timeout: " + detail)

    def run(self):
        self.wait_for(
            lambda: self.fast_pub.get_num_connections() > 0
            and self.correction_pub.get_num_connections() > 0,
            5.0, "publishers did not connect",
        )

        # A: correction missing -> STARTING/invalid and no competition odom.
        for _ in range(3):
            self.publish_fast()
            rospy.sleep(0.05)
        self.wait_for(
            lambda: self.status is not None
            and self.status.state == ModuleStatus.STARTING
            and not self.status.valid,
            3.0, "missing correction did not fail closed",
        )
        if self.odometry:
            raise RuntimeError("competition odom published without correction")

        # B: fresh correction + fresh FAST-LIO -> RUNNING/valid.
        self.publish_correction(x=1.0)
        rospy.sleep(0.03)
        self.publish_fast(x=0.1)
        self.wait_for(
            lambda: self.status is not None
            and self.status.state == ModuleStatus.RUNNING
            and self.status.valid and len(self.odometry) > 0,
            3.0, "fresh correction did not reach RUNNING",
        )

        # C/D: stop correction while FAST-LIO continues.  Once TIMEOUT is
        # observed, further raw frames must not reuse the stale x/y/yaw.
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            self.publish_fast(x=2.0)
            rospy.sleep(0.08)
        self.wait_for(
            lambda: self.status is not None
            and self.status.state == ModuleStatus.TIMEOUT
            and not self.status.valid,
            3.0, "correction interruption did not reach TIMEOUT",
        )
        count_at_timeout = len(self.odometry)
        for _ in range(3):
            self.publish_fast(x=5.0)
            rospy.sleep(0.08)
        if len(self.odometry) != count_at_timeout:
            raise RuntimeError("stale correction was reused after timeout")

        # E: only a new legal correction plus a compatible FAST-LIO frame can
        # recover the chain.
        self.publish_correction(x=3.0)
        rospy.sleep(0.03)
        self.publish_fast(x=3.0)
        self.wait_for(
            lambda: self.status is not None
            and self.status.state == ModuleStatus.RUNNING
            and self.status.valid and len(self.odometry) > count_at_timeout,
            3.0, "fresh correction recovery did not reach RUNNING",
        )

        output = self.odometry[-1]
        if output.header.frame_id != "world" or output.child_frame_id != "base":
            raise RuntimeError("output frame contract mismatch")
        transform = self.tf_buffer.lookup_transform(
            "world", "base", output.header.stamp, rospy.Duration(2.0)
        )
        if not math.isclose(
            transform.header.stamp.to_sec(),
            output.header.stamp.to_sec(),
            abs_tol=1e-9,
        ):
            raise RuntimeError(
                "TF/odom timestamp mismatch: {:.9f} != {:.9f}".format(
                    transform.header.stamp.to_sec(), output.header.stamp.to_sec()
                )
            )
        if not math.isclose(
            transform.transform.translation.x,
            output.pose.pose.position.x,
            abs_tol=1e-9,
        ):
            raise RuntimeError("TF/odom pose mismatch")

        print("CORRECTION_MISSING_FAIL_CLOSED=True")
        print("CORRECTION_FRESH_RUNNING=True")
        print("CORRECTION_TIMEOUT=True")
        print("STALE_CORRECTION_REUSE=False")
        print("CORRECTION_FRESH_RECOVERY=True")
        print("TF_ODOM_CONSISTENT=True")


def main():
    rospy.init_node("pr16_correction_smoke")
    CorrectionSmoke().run()


if __name__ == "__main__":
    main()
