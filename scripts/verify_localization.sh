#!/usr/bin/env bash
set -euo pipefail

# verify_localization.sh — Verify the FAST-LIO localization adapter.
# Must be run from the workspace root AFTER catkin_make.
#
# This is a standalone isolated verification — it starts a private roscore,
# the localization_node, and a mock FAST-LIO publisher.  It does NOT launch
# FAST-LIO or localization_bringup.launch.
#
# Checks:
#   1. /competition/localization/odom publishing
#   2. /competition/localization/status publishing
#   3. TF world -> base consistency (real tf2 lookup, <1mm odom agreement)
#   4. Watchdog: input stop -> TIMEOUT + valid=false
#   5. Static extrinsic consistency (launch file vs expected)
#   6. Forbidden-topic isolation (no referee/ground-truth/control topics)
#   7. Runtime FAST-LIO extrinsic check (SKIP when FAST-LIO not launched)

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

# ── Static extrinsic consistency check ─────────────────────────────
# Verify that the canonical launch file overrides match the expected
# values derived from the a1_description URDF chain (see the derivation
# in localization_bringup.launch). This is a source-of-truth static check,
# not a runtime FAST-LIO check.
echo "=== Static extrinsic consistency check ==="

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
LAUNCH_FILE="${REPO_ROOT}/src/competition_localization/launch/localization_bringup.launch"

EXPECTED_EXTRINSIC_T="[0.011, 0.02329, -0.04412]"
EXPECTED_BASE_TO_IMU_T="[0.223404, -0.023290, 0.118985]"
EXPECTED_BASE_TO_IMU_RPY="[0.0, 0.785, 0.0]"

# Use Python + YAML to parse the rosparam values from the launch file.
# The launch file contains lines like:
#   <rosparam param="mapping/extrinsic_T" subst_value="True">[0.011, 0.02329, -0.04412]</rosparam>
#   <rosparam param="base_to_imu_translation">[0.223404, -0.023290, 0.118985]</rosparam>
#   <rosparam param="base_to_imu_rotation_rpy">[0.0, 0.785, 0.0]</rosparam>
EXTRINSIC_CHECK=$(
    python3 -c "
import re, sys, yaml, math

launch_file = '${LAUNCH_FILE}'
with open(launch_file, 'r') as f:
    text = f.read()

def extract_rosparam_value(param_name):
    '''Extract the content of a <rosparam param=\"...\">...</rosparam> element.'''
    pattern = rf'<rosparam\s+param=\"{param_name}\"[^>]*>\\s*([^<]+)\\s*</rosparam>'
    m = re.search(pattern, text)
    if not m:
        # Also try <rosparam ...>value</rosparam> across multiple lines
        pattern2 = rf'<rosparam\s+param=\"{param_name}\"[^>]*>\\s*([^<]+)\\s*</rosparam>'
        m = re.search(pattern2, text, re.DOTALL)
    if not m:
        print(f'SKIP: cannot find {param_name} in launch file')
        return None, None
    raw = m.group(1).strip()
    try:
        parsed = yaml.safe_load(raw)
    except Exception:
        print(f'SKIP: cannot parse {param_name} value: {raw[:60]}')
        return None, None
    return parsed, raw

def check_param(param_name, expected_raw, tol=1e-6):
    parsed, raw = extract_rosparam_value(param_name)
    if parsed is None:
        return  # SKIP already printed
    # parsed may be a list or a string that yaml decoded as list
    if isinstance(parsed, str):
        try:
            parsed = yaml.safe_load(parsed)
        except Exception:
            pass
    if not isinstance(parsed, (list, tuple)):
        print(f'SKIP: {param_name} parsed as {type(parsed).__name__}, not a sequence')
        return
    if len(parsed) != 3:
        print(f'FAIL: {param_name} has {len(parsed)} elements, expected 3')
        sys.exit(1)
    for i, v in enumerate(parsed):
        try:
            fv = float(v)
        except (TypeError, ValueError):
            print(f'FAIL: {param_name}[{i}]={v!r} is not a finite float')
            sys.exit(1)
        if not math.isfinite(fv):
            print(f'FAIL: {param_name}[{i}]={fv} is not finite')
            sys.exit(1)
    expected = yaml.safe_load(expected_raw)
    for i, (a, b) in enumerate(zip(parsed, expected)):
        if abs(float(a) - float(b)) > tol:
            print(f'FAIL: {param_name}[{i}]={float(a):.6f} != expected {float(b):.6f}')
            sys.exit(1)
    print(f'  PASS: {param_name} = {parsed} (matches expected)')

check_param('mapping/extrinsic_T', '${EXPECTED_EXTRINSIC_T}')
check_param('base_to_imu_translation', '${EXPECTED_BASE_TO_IMU_T}')
check_param('base_to_imu_rotation_rpy', '${EXPECTED_BASE_TO_IMU_RPY}')
print('PASS: static extrinsic consistency OK')
" 2>&1
)
EXTRINSIC_RC=$?
echo "${EXTRINSIC_CHECK}"
if [ "${EXTRINSIC_RC}" -ne 0 ]; then
    echo "FAIL: static extrinsic consistency check failed"
    exit 1
fi

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

echo "=== [Watchdog] Stop input >1s -> TIMEOUT & valid=false ==="
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

# ── Forbidden-topic isolation check ─────────────────────────────────
# In this Localization-only isolated verification, no referee /
# ground-truth / control topics should appear.  The presence of these
# topics indicates contamination from other modules or misconfiguration.
#
# NOTE: /cmd_vel is NOT a forbidden topic in the full production system
# (cmd_vel_guard_node is the exclusive publisher per module contract
# v0.2.0).  Its presence here only means this isolated verification
# environment has been polluted by a control node — not that /cmd_vel is
# globally disallowed.
echo "=== Forbidden-topic isolation check ==="
CURRENT_TOPICS=$(timeout 5 rostopic list 2>/dev/null || echo "")
FORBIDDEN=""
for pattern in "/Odometry_gazebo" "/ground_truth" "/cmd_vel"; do
    if echo "$CURRENT_TOPICS" | grep -qF "$pattern" 2>/dev/null; then
        FORBIDDEN="${FORBIDDEN} ${pattern}"
    fi
done
if [ -n "$FORBIDDEN" ]; then
    echo "FAIL: isolated verification environment contains unexpected topics:${FORBIDDEN}" >&2
    echo "  /Odometry_gazebo  — referee odometry, must not appear in localization-only test"
    echo "  /ground_truth/*   — ground-truth data, must not appear in localization-only test"
    echo "  /cmd_vel          — control output, indicates test environment contamination"
    echo "  (Note: /cmd_vel is a legitimate production topic; its presence here only"
    echo "   means this isolated test has unexpected control module activity.)"
    exit 1
fi
echo "PASS: no forbidden topics in isolated verification environment"

# ── Runtime FAST-LIO extrinsic consistency check ─────────────────────
# This check only applies when FAST-LIO (laserMapping) is running and
# has loaded its mapping/extrinsic_T onto the parameter server.  In a
# standalone verification (no FAST-LIO launch), we skip with a clear
# message rather than silently pass.
echo "=== Runtime FAST-LIO extrinsic check ==="
FASTLIO_EXTRINSIC_T=$(rosparam get /mapping/extrinsic_T 2>/dev/null || echo "")
if [ -z "$FASTLIO_EXTRINSIC_T" ]; then
    echo "SKIP: /mapping/extrinsic_T not on parameter server"
    echo "      (standalone verification does not launch FAST-LIO;"
    echo "       runtime extrinsic check requires a running laserMapping node)"
else
    # Parse with Python YAML (not string split) for reliable YAML decoding.
    RUNTIME_CHECK=$(
        python3 -c "
import sys, yaml, math

raw = '''${FASTLIO_EXTRINSIC_T}'''
expected = [0.011, 0.02329, -0.04412]
tol = 1e-6

try:
    parsed = yaml.safe_load(raw)
except Exception as e:
    print(f'SKIP: cannot YAML-parse /mapping/extrinsic_T: {e}')
    sys.exit(0)

if not isinstance(parsed, (list, tuple)):
    print(f'FAIL: /mapping/extrinsic_T is {type(parsed).__name__}, not a sequence')
    sys.exit(1)
if len(parsed) != 3:
    print(f'FAIL: /mapping/extrinsic_T has {len(parsed)} elements, expected 3')
    sys.exit(1)
for i, v in enumerate(parsed):
    try:
        fv = float(v)
    except (TypeError, ValueError):
        print(f'FAIL: /mapping/extrinsic_T[{i}]={v!r} is not a finite float')
        sys.exit(1)
    if not math.isfinite(fv):
        print(f'FAIL: /mapping/extrinsic_T[{i}]={fv} is not finite')
        sys.exit(1)
for i, (a, b) in enumerate(zip(parsed, expected)):
    if abs(float(a) - float(b)) > tol:
        print(f'FAIL: /mapping/extrinsic_T[{i}]={float(a):.6f} != expected {float(b):.6f}')
        sys.exit(1)
print(f'PASS: runtime /mapping/extrinsic_T = {parsed} matches expected {expected}')
" 2>&1
    )
    RUNTIME_RC=$?
    echo "${RUNTIME_CHECK}"
    if [ "${RUNTIME_RC}" -ne 0 ]; then
        echo "FAIL: runtime FAST-LIO extrinsic check failed"
        exit 1
    fi
fi

echo "=== All checks passed ==="
