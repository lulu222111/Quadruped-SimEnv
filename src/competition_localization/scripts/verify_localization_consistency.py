#!/usr/bin/env python3
"""verify_localization_consistency — real TF query for the localization adapter.

Replaces the old `grep odom` on the /tf topic with an explicit TF lookup using
tf2_ros.Buffer.lookup_transform("world", "base", time, timeout) plus strict
assertions:

  - the lookup must succeed within a bounded timeout;
  - translation and quaternion values must be finite;
  - quaternion norm must be ~1.0;
  - the odometry stamp must be valid, and the TF pose must agree with
    /competition/localization/odom position at the SAME stamp to < 1 mm.

Exit code 0 = PASS, 1 = FAIL. Used by scripts/verify_localization.sh.
"""

from __future__ import annotations

import math
import sys

import rospy
import tf2_ros
from nav_msgs.msg import Odometry

LOOKUP_TIMEOUT_SEC = 1.0
ODOM_WAIT_TIMEOUT_SEC = 5.0
MAX_POSITION_ERROR_M = 0.001  # 1 mm


def _fail(msg: str) -> None:
    print(f"FAIL: {msg}")
    sys.exit(1)


def main() -> None:
    rospy.init_node("verify_localization_consistency", anonymous=True)

    buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(buffer)

    received = {"odom": None}

    def odom_cb(msg: Odometry) -> None:
        received["odom"] = msg

    rospy.Subscriber("/competition/localization/odom", Odometry, odom_cb, queue_size=1)

    # --- Wait for a valid odometry message (bounded) ---
    deadline = rospy.Time.now() + rospy.Duration(ODOM_WAIT_TIMEOUT_SEC)
    while received["odom"] is None and rospy.Time.now() < deadline:
        rospy.sleep(0.1)
    if received["odom"] is None:
        _fail("no /competition/localization/odom received within timeout")
    odom = received["odom"]

    stamp = odom.header.stamp
    if stamp.to_sec() <= 0.0:
        _fail(f"odometry has invalid (zero) timestamp: {stamp}")

    # --- Real TF lookup: world -> base at the odometry stamp (bounded) ---
    try:
        tf = buffer.lookup_transform(
            "world", "base", stamp, rospy.Duration(LOOKUP_TIMEOUT_SEC)
        )
    except Exception as exc:  # noqa: BLE001
        _fail(f"lookup_transform(world -> base, t={stamp}) failed: {exc}")

    pos = tf.transform.translation
    rot = tf.transform.rotation
    values = [pos.x, pos.y, pos.z, rot.x, rot.y, rot.z, rot.w]
    if not all(math.isfinite(v) for v in values):
        _fail(f"TF contains non-finite values: {values}")

    q_norm = math.sqrt(rot.x**2 + rot.y**2 + rot.z**2 + rot.w**2)
    if abs(q_norm - 1.0) > 1e-3:
        _fail(f"quaternion norm {q_norm:.6f} != 1.0")

    # --- TF vs odom position agreement at the SAME stamp (< 1 mm) ---
    odom_pos = odom.pose.pose.position
    err = math.sqrt(
        (pos.x - odom_pos.x) ** 2
        + (pos.y - odom_pos.y) ** 2
        + (pos.z - odom_pos.z) ** 2
    )
    if err >= MAX_POSITION_ERROR_M:
        _fail(
            f"TF vs odom position error {err * 1000.0:.3f} mm >= 1 mm "
            f"(TF=({pos.x:.4f},{pos.y:.4f},{pos.z:.4f}) "
            f"odom=({odom_pos.x:.4f},{odom_pos.y:.4f},{odom_pos.z:.4f}))"
        )

    print(
        f"PASS: TF world -> base at t={stamp} is finite, quaternion norm "
        f"{q_norm:.6f}, position agrees with odom within "
        f"{err * 1000.0:.3f} mm (< 1 mm)"
    )
    sys.exit(0)


if __name__ == "__main__":
    main()
