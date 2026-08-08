#!/usr/bin/env bash
set -euo pipefail

# verify_localization.sh — Verify the FAST-LIO localization adapter inside the
# simenv container.  Must be run from the workspace root AFTER catkin_make.
#
# The old `grep odom` over /tf is REMOVED. TF is verified with a real
# tf2_ros.Buffer.lookup_transform("world", "base", ...) via
# competition_localization/scripts/verify_localization_consistency.py
# (finite values, unit quaternion, timestamp validity, <1mm odom agreement).

: "${ROS_DISTRO:=noetic}"

# Avoid unbound-variable failures when sourcing ROS setup under set -u.
: "${ROS_MASTER_URI:=}"
: "${ROS_HOSTNAME:=$(hostname)}"

echo "=== Sourcing ROS environment ==="
# Standard SimEnv layout: /opt/ros/<distro>/setup.bash
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
    source "${ROS_SETUP}"
else
    echo "FAIL: ROS setup not found at ${ROS_SETUP}" >&2
    echo "Run this script inside the SimEnv container AFTER catkin_make." >&2
    exit 1
fi
if [[ -f "devel/setup.bash" ]]; then
    source "devel/setup.bash"
fi

# Fail fast if required ROS tools are missing.
for tool in roscore rosrun rostopic rosnode rosparam; do
    if ! command -v "$tool" >/dev/null 2>&1; then
        echo "FAIL: required ROS tool '$tool' not found on PATH" >&2
        exit 1
    fi
done

# Pick a free port so multiple users / runs do not collide on 11311.
PORT=$(python3 -c 'import socket; s=socket.socket(); s.bind(("", 0)); print(s.getsockname()[1]); s.close()')
export ROS_MASTER_URI="http://localhost:${PORT}"
echo "=== Using ROS_MASTER_URI=${ROS_MASTER_URI} ==="

echo "=== Starting roscore on port ${PORT} ==="
roscore --port "${PORT}" &
ROSCORE_PID=$!

cleanup() {
    echo "=== Cleaning up processes (only those created by this script) ==="
    if [[ -n "${PUB_PID:-}" ]]; then
        kill "${PUB_PID}" 2>/dev/null || true
        wait "${PUB_PID}" 2>/dev/null || true
    fi
    if [[ -n "${CHECK_PID:-}" ]]; then
        kill "${CHECK_PID}" 2>/dev/null || true
        wait "${CHECK_PID}" 2>/dev/null || true
    fi
    if [[ -n "${NODE_PID:-}" ]]; then
        kill "${NODE_PID}" 2>/dev/null || true
        wait "${NODE_PID}" 2>/dev/null || true
    fi
    if [[ -n "${ROSCORE_PID:-}" ]]; then
        kill "${ROSCORE_PID}" 2>/dev/null || true
        wait "${ROSCORE_PID}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# Poll for the ROS master with roscore PID liveness check (bounded).
wait_for_master() {
    local deadline=$((SECONDS + 30))
    while [ "$SECONDS" -lt "$deadline" ]; do
        if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
            echo "FAIL: roscore died during startup" >&2
            return 1
        fi
        if timeout 1 rosnode list >/dev/null 2>&1; then
            return 0
        fi
        sleep 0.25
    done
    echo "FAIL: timed out waiting for ROS master" >&2
    return 1
}

if ! wait_for_master; then
    exit 1
fi
echo "=== ROS master is up ==="

echo "=== Setting workspace root for localization_node ==="
rosparam set /competition_loop/workspace_root "$(pwd)"
# Standalone dev verification runs in DEBUG mode (lenient on scene-file /
# extrinsic failures so the test can run from any directory).
rosparam set /competition_loop/competition_mode false

# Fixed extrinsic base -> livox_imu_link (from a1_description URDF chain).
# Ensures the extrinsic-compensation path is exercised even without
# robot_state_publisher running in this standalone test.
rosparam set /localization/base_to_imu_translation "[0.223404, -0.023290, 0.118985]"
rosparam set /localization/base_to_imu_rotation_rpy "[0.0, 0.785, 0.0]"

echo "=== Starting localization_node in background ==="
# __name:=localization aligns the private namespace (~base_to_imu_*, ...) with
# the /localization/* params set below (the rosrun default node name would be
# localization_node and the extrinsic params would be missed).
rosrun competition_localization localization_node __name:=localization &
NODE_PID=$!
sleep 3

echo "=== Publishing mock FAST-LIO /Odometry ==="
# Publish continuously at 10 Hz with a FRESH timestamp per message (rostopic
# pub -r reuses one message/stamp, which floods tf2 with TF_REPEATED_DATA).
rosrun competition_localization mock_fast_lio_odom.py &
PUB_PID=$!

# Wait for alignment window (default ~2s) plus margin.
sleep 4

echo "=== Checking /competition/localization/odom ==="
if timeout 5 rostopic echo -n 1 /competition/localization/odom >/dev/null 2>&1; then
    echo "OK: /competition/localization/odom is publishing"
    timeout 5 rostopic echo -n 1 /competition/localization/odom
else
    echo "FAIL: /competition/localization/odom is not publishing"
    exit 1
fi

echo "=== Checking /competition/localization/status ==="
if timeout 5 rostopic echo -n 1 /competition/localization/status >/dev/null 2>&1; then
    echo "OK: /competition/localization/status is publishing"
    timeout 5 rostopic echo -n 1 /competition/localization/status
else
    echo "FAIL: /competition/localization/status is not publishing"
    exit 1
fi

echo "=== Checking TF world -> base (real lookup_transform) + odom consistency ==="
# The localization module publishes a single dynamic TF world -> base; the
# odom frame is removed per the team contract. The checker performs an actual
# tf2_ros lookup and validates finite values, unit quaternion, valid timestamp
# and < 1 mm agreement with /competition/localization/odom.
if timeout 30 rosrun competition_localization verify_localization_consistency.py; then
    echo "OK: TF world -> base is consistent with odom"
else
    echo "FAIL: TF world -> base consistency check failed"
    exit 1
fi

echo "=== [Item 9] Watchdog: stop input >1s -> TIMEOUT & valid=false ==="
# Stop the mock FAST-LIO input so the adapter stops receiving messages.
kill "${PUB_PID}" 2>/dev/null || true
wait "${PUB_PID}" 2>/dev/null || true

# Wait beyond the watchdog period (default 1s) plus margin, then read status.
sleep 3
STATUS="$(timeout 5 rostopic echo -n 1 /competition/localization/status 2>/dev/null || true)"
echo "$STATUS"
# ModuleStatus.state == 6 (TIMEOUT) and valid == false.
if echo "$STATUS" | grep -q "state: 6" && echo "$STATUS" | grep -q "valid: False"; then
    echo "OK: watchdog fired -> TIMEOUT (state=6), valid=false"
else
    echo "FAIL: watchdog did not produce TIMEOUT / valid=false after input stop"
    exit 1
fi

echo "=== All checks passed ==="
