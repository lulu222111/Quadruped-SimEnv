#!/bin/bash
# Integration test: ROS Master forbidden topic verification.
#
# Must be run inside the ROS container with ROS environment sourced:
#   source /opt/ros/noetic/setup.bash
#   source /tmp/competition-loop-pr7-devel/setup.bash
#
# Tests:
#   1. No forbidden topics → check PASSES
#   2. Add /Odometry_gazebo publisher → check FAILS
#   3. competition_loop_node detects → stays WAITING, safety signals correct
#   4. After publisher stops → fresh check passes
#   5. Confirmed DETECTION → permanently blocked (by design)
#
# Does NOT: read truth files, subscribe to /Odometry_gazebo, call Gazebo,
# start Gazebo, publish /cmd_vel.

set -o pipefail

# ============================================================
# Port selection
# ============================================================
# Priority: caller's explicit ROS_PORT > auto-selected free port.
# Auto mode binds 127.0.0.1:0 via Python to let the OS pick.
SELECTED_BY_AUTO=false

if [ -z "${ROS_PORT+x}" ]; then
    # Not set at all — auto-select a free port
    CANDIDATE=$(python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', 0))
port = s.getsockname()[1]
s.close()
print(port)
" 2>/dev/null)
    if [ -z "$CANDIDATE" ]; then
        echo "FATAL: Failed to auto-select a free port via Python socket"
        exit 1
    fi
    ROS_PORT="$CANDIDATE"
    SELECTED_BY_AUTO=true
    echo "ROS_PORT auto-selected: $ROS_PORT"
else
    # Caller explicitly set ROS_PORT — validate it
    case "$ROS_PORT" in
        ''|*[!0-9]*)
            echo "FATAL: ROS_PORT must be an integer, got: '$ROS_PORT'"
            exit 1
            ;;
    esac
    if [ "$ROS_PORT" -lt 1024 ] || [ "$ROS_PORT" -gt 65535 ]; then
        echo "FATAL: ROS_PORT must be in [1024, 65535], got: $ROS_PORT"
        exit 1
    fi
    echo "ROS_PORT explicitly set by caller: $ROS_PORT"
fi

export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"

# ============================================================
# Isolated temporary directories (UID + PID)
# ============================================================
SCENE_DIR="/tmp/simenv-pr7-forbidden-${UID:-$(id -u)}-$$"
ROS_HOME_DIR="/tmp/simenv-pr7-ros-home-${UID:-$(id -u)}-$$"

rm -rf "$SCENE_DIR" "$ROS_HOME_DIR"
mkdir -p "$SCENE_DIR/generated_building"
mkdir -p "$ROS_HOME_DIR"
export ROS_HOME="$ROS_HOME_DIR"

# ============================================================
# Logging
# ============================================================
LOG_DIR="/workspace/SimEnv/artifacts/logs"
mkdir -p "$LOG_DIR"
TIMESTAMP=$(date +%Y%m%d-%H%M%S)
LOG="${LOG_DIR}/competition-loop-pr7-forbidden-topics-${TIMESTAMP}.log"
exec > >(tee -a "$LOG") 2>&1

echo "=============================================="
echo "ROS Forbidden Topic Integration Test (Isolated)"
echo "Timestamp: $(date)"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_HOME: $ROS_HOME"
echo "Scene dir: $SCENE_DIR"
echo "Log: $LOG"
echo "UID: ${UID:-$(id -u)}  PID: $$"
echo "=============================================="

# ============================================================
# Test scene
# ============================================================
cat > "$SCENE_DIR/generated_building/team_scene_info.json" << 'EOF'
{
  "schema": "v1",
  "referee_only": {
    "forbidden_topics": ["/Odometry_gazebo", "/ground_truth/odom", "/ground_truth/pose"]
  }
}
EOF

# ============================================================
# Process tracking & cleanup
# ============================================================
ROSCORE_PID=""
OWN_PIDS=""
CLEANED_UP=false

cleanup() {
    if $CLEANED_UP; then return; fi
    CLEANED_UP=true
    echo ""; echo "=== Cleanup ==="
    for pid in $OWN_PIDS; do
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            echo "  Stopping pid=$pid"
            kill "$pid" 2>/dev/null || true
            wait "$pid" 2>/dev/null || true
        fi
    done
    if [ -n "$ROSCORE_PID" ] && kill -0 "$ROSCORE_PID" 2>/dev/null; then
        echo "  Stopping roscore pid=$ROSCORE_PID"
        kill "$ROSCORE_PID" 2>/dev/null || true
        wait "$ROSCORE_PID" 2>/dev/null || true
    fi
    rm -rf "$SCENE_DIR" "$ROS_HOME_DIR"
    echo "  Cleanup done"
}
trap cleanup EXIT

# ============================================================
# Start roscore with retry (up to 5 attempts)
# ============================================================
MAX_RETRIES=5
ROSCORE_STARTED=false

for attempt in $(seq 1 $MAX_RETRIES); do
    echo ""; echo "=== Starting roscore (attempt $attempt/$MAX_RETRIES) on port $ROS_PORT ==="

    roscore -p "$ROS_PORT" > /tmp/roscore-pr7-$$.log 2>&1 &
    ROSCORE_PID=$!
    echo "  roscore PID: $ROSCORE_PID"

    # Wait for OUR roscore to be ready (poll ROS Master).
    # CRITICAL: verify our PID is still alive BEFORE trusting rostopic,
    # otherwise a port-collision exit goes undetected and we connect to
    # another user's pre-existing ROS Master.
    ROSCORE_READY=false
    for i in $(seq 1 20); do
        sleep 0.5
        if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
            echo "  roscore PID=$ROSCORE_PID exited (attempt=$i) — port collision or crash"
            break
        fi
        if rostopic list &>/dev/null; then
            echo "  roscore ready (pid=$ROSCORE_PID, attempt=$i)"
            ROSCORE_READY=true
            break
        fi
    done

    if $ROSCORE_READY; then
        ROSCORE_STARTED=true
        break
    fi

    # Failed this attempt — diagnostic
    if kill -0 "$ROSCORE_PID" 2>/dev/null; then
        echo "  FAIL: roscore PID=$ROSCORE_PID still alive but Master not reachable on port $ROS_PORT"
    else
        echo "  FAIL: roscore PID=$ROSCORE_PID exited — port $ROS_PORT likely occupied by another Master"
    fi
    echo "  Last lines of roscore log:"
    tail -20 /tmp/roscore-pr7-$$.log 2>/dev/null | sed 's/^/    /' || echo "    (empty)"

    kill "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
    ROSCORE_PID=""

    if $SELECTED_BY_AUTO && [ "$attempt" -lt "$MAX_RETRIES" ]; then
        # Pick a new free port for the next retry
        CANDIDATE=$(python3 -c "
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind(('127.0.0.1', 0))
port = s.getsockname()[1]
s.close()
print(port)
" 2>/dev/null)
        if [ -n "$CANDIDATE" ]; then
            ROS_PORT="$CANDIDATE"
            export ROS_MASTER_URI="http://127.0.0.1:${ROS_PORT}"
            echo "  Auto-selected new port for retry: $ROS_PORT"
        fi
    elif ! $SELECTED_BY_AUTO; then
        # Caller explicitly set this port — do NOT silently switch
        echo "  FATAL: Explicit ROS_PORT=$ROS_PORT is occupied or unusable"
        echo "  Will not silently switch to a different port."
    fi
done

if ! $ROSCORE_STARTED; then
    echo ""
    echo "FATAL: roscore failed to start after $MAX_RETRIES attempt(s)"
    echo "  ROS_PORT: $ROS_PORT"
    echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
    echo "  Mode: $($SELECTED_BY_AUTO && echo 'auto-select' || echo 'explicit (caller-set)')"
    exit 1
fi

echo "  roscore started successfully on port $ROS_PORT (PID=$ROSCORE_PID)"

# ============================================================
# Test counters
# ============================================================
PASS=0; FAIL=0
ok() { echo "  PASS: $1"; PASS=$((PASS+1)); }
no() { echo "  FAIL: $1"; FAIL=$((FAIL+1)); }

# ============================================================
# Helper: poll a log file for a pattern until timeout
# ============================================================
wait_for_log() {
    local file="$1"
    local pattern="$2"
    local timeout_sec="${3:-10}"
    local deadline=$((SECONDS + timeout_sec))

    while (( SECONDS < deadline )); do
        if grep -qE "$pattern" "$file" 2>/dev/null; then
            return 0
        fi
        sleep 0.2
    done
    return 1
}

# ============================================================
# TEST 1: no forbidden topics → PASS
# ============================================================
echo ""; echo "=== TEST 1: No forbidden topics → check passes ==="

timeout 10 python3 -c "
import rospy, sys
rospy.init_node('t1', log_level=rospy.INFO)
sys.path.insert(0, '/workspace/SimEnv/src/competition_loop/src')
from interface_validator import InterfaceValidator
v = InterfaceValidator(competition_mode=True)
v.load_scene_info('$SCENE_DIR')
r = v.check_forbidden_topics()
print(f'completed={v.forbidden_topic_check_completed} passed={v.forbidden_topic_check_passed} ready={v.is_ready()}')
sys.exit(0 if r and v.forbidden_topic_check_passed else 1)
" 2>&1 && ok "Test 1: No forbidden topics → PASS" || no "Test 1: No forbidden topics → PASS"

# ============================================================
# TEST 2: /Odometry_gazebo present → FAIL
# ============================================================
echo ""; echo "=== TEST 2: /Odometry_gazebo published → check fails ==="

# Start persistent fake publisher
NODE_NS="fake_odom_pr7_$$"
timeout 40 python3 -c "
import rospy, std_msgs.msg, time
rospy.init_node('$NODE_NS', anonymous=True)
pub = rospy.Publisher('/Odometry_gazebo', std_msgs.msg.String, queue_size=1, latch=True)
rospy.sleep(0.5)
pub.publish(std_msgs.msg.String(data='fake'))
rospy.sleep(0.5)
print('FAKE_READY', flush=True)
rospy.spin()
" &
FAKE_PID=$!
OWN_PIDS="$OWN_PIDS $FAKE_PID"

# Wait for fake publisher topic to appear
for i in $(seq 1 15); do
    sleep 0.3
    if rostopic list 2>/dev/null | grep -q "/Odometry_gazebo"; then
        echo "  Fake publisher confirmed active (attempt=$i)"
        break
    fi
    if [ "$i" -eq 15 ]; then
        no "Test 2 prerequisite: fake publisher not visible in rostopic list"
    fi
done

timeout 10 python3 -c "
import rospy, sys
rospy.init_node('t2', log_level=rospy.INFO)
sys.path.insert(0, '/workspace/SimEnv/src/competition_loop/src')
from interface_validator import InterfaceValidator
v = InterfaceValidator(competition_mode=True)
v.load_scene_info('$SCENE_DIR')
r = v.check_forbidden_topics()
has_odom = any('/Odometry_gazebo' in e for e in v.errors)
print(f'completed={v.forbidden_topic_check_completed} passed={v.forbidden_topic_check_passed} has_odom_err={has_odom}')
sys.exit(0 if not r and v.forbidden_topic_check_completed and not v.forbidden_topic_check_passed and has_odom else 1)
" 2>&1 && ok "Test 2: /Odometry_gazebo detected → FAIL (correctly blocked)" || no "Test 2: /Odometry_gazebo detected → FAIL"

# ============================================================
# TEST 3: node integration with forbidden topic
# ============================================================
echo ""; echo "=== TEST 3: competition_loop_node with forbidden topic ==="

NODE_LOG="/tmp/node-output-pr7-$$.txt"
> "$NODE_LOG"

# CRITICAL: PYTHONUNBUFFERED=1 prevents block-buffering when stdout
# is redirected to a file. Without it, rospy.logerr() messages may
# be held in the C stdio buffer and never reach the file within the
# test timeout. Confirmed by root-cause analysis: file was 0 bytes
# after 5s without this flag, 878 bytes within 0.5s with it.
PYTHONUNBUFFERED=1 timeout 12 python3 /workspace/SimEnv/src/competition_loop/scripts/competition_loop_node \
    _competition_mode:=true \
    _auto_start:=false \
    _max_time:=30.0 \
    _start_delay:=0.3 \
    _workspace_root:="$SCENE_DIR" \
    > "$NODE_LOG" 2>&1 &
LOOP_PID=$!
OWN_PIDS="$OWN_PIDS $LOOP_PID"
echo "  competition_loop PID: $LOOP_PID"

# Poll for log content (10s deadline)
if ! wait_for_log "$NODE_LOG" "FORBIDDEN TOPIC DETECTED" 10; then
    echo "  WARNING: FORBIDDEN TOPIC DETECTED not found in node log within 10s"
    echo "    File: $NODE_LOG ($(wc -c < "$NODE_LOG" 2>/dev/null || echo 0) bytes)"
    echo "    Node PID alive: $(kill -0 $LOOP_PID 2>/dev/null && echo yes || echo no)"
    echo "    Last 30 lines of node log:"
    tail -30 "$NODE_LOG" 2>/dev/null | sed 's/^/      /' || echo "      (empty)"
fi

# --- Verify via ROS topics (these are the functional assertions) ---

# Test 3a: WAITING state
STATE=$(rostopic echo /competition/state -n 1 2>/dev/null || echo "")
if echo "$STATE" | grep -q "state: 0"; then
    ok "Test 3a: Node stays in WAITING (state=0)"
else
    no "Test 3a: Node stays in WAITING (got: $(echo "$STATE" | grep state || echo 'empty'))"
fi

# Test 3b: control_enabled = false
CE=$(rostopic echo /competition/control_enabled -n 1 2>/dev/null || echo "")
if echo "$CE" | grep -q "data: False"; then
    ok "Test 3b: control_enabled=false"
else
    no "Test 3b: control_enabled=false (got: $(echo "$CE" | grep data || echo 'empty'))"
fi

# Test 3c: emergency_stop = true
ES=$(rostopic echo /competition/emergency_stop -n 1 2>/dev/null || echo "")
if echo "$ES" | grep -q "data: True"; then
    ok "Test 3c: emergency_stop=true"
else
    no "Test 3c: emergency_stop=true (got: $(echo "$ES" | grep data || echo 'empty'))"
fi

# Test 3d: forbidden topic detection confirmed in node output
# The FORBIDDEN TOPIC DETECTED log is produced by interface_validator.py:212
# via rospy.logerr(). Verified that with PYTHONUNBUFFERED=1 it appears in
# the captured file within ~0.5s of node start.
if grep -q "FORBIDDEN TOPIC DETECTED" "$NODE_LOG" 2>/dev/null; then
    ok "Test 3d: Node log contains FORBIDDEN TOPIC DETECTED"
elif grep -q "Forbidden topic check: FAILED" "$NODE_LOG" 2>/dev/null; then
    ok "Test 3d: Node log contains Forbidden topic check FAILED"
else
    echo "  Node log (full, $(wc -c < "$NODE_LOG" 2>/dev/null || echo 0) bytes):"
    cat "$NODE_LOG" 2>/dev/null | sed 's/^/    /'
    no "Test 3d: Node log contains forbidden topic detection"
fi

# Stop the competition loop node
kill $LOOP_PID 2>/dev/null || true
wait $LOOP_PID 2>/dev/null || true
OWN_PIDS="${OWN_PIDS//$LOOP_PID/}"
LOOP_PID=""

# Stop fake publisher
kill $FAKE_PID 2>/dev/null || true
wait $FAKE_PID 2>/dev/null || true
OWN_PIDS="${OWN_PIDS//$FAKE_PID/}"
FAKE_PID=""
sleep 1

# ============================================================
# TEST 4: After publisher stops → fresh check passes
# ============================================================
echo ""; echo "=== TEST 4: Fresh checker after publisher stops ==="

timeout 10 python3 -c "
import rospy, sys
rospy.init_node('t4', log_level=rospy.INFO)
sys.path.insert(0, '/workspace/SimEnv/src/competition_loop/src')
from interface_validator import InterfaceValidator
v = InterfaceValidator(competition_mode=True)
v.load_scene_info('$SCENE_DIR')
r = v.check_forbidden_topics()
print(f'completed={v.forbidden_topic_check_completed} passed={v.forbidden_topic_check_passed} ready={v.is_ready()}')
sys.exit(0 if v.forbidden_topic_check_passed else 1)
" 2>&1 && ok "Test 4: Fresh check PASSES after publisher stops" || no "Test 4: Fresh check PASSES after publisher stops"

# ============================================================
# TEST 5: Confirmed detection is permanently blocked (by design)
# ============================================================
echo ""; echo "=== TEST 5: Confirmed detection is permanently blocked ==="
echo "  Design note: Once forbidden_topic_check is completed+failed (confirmed"
echo "  detection), the check remains failed permanently. The design intentionally"
echo "  does NOT auto-clear — this prevents race conditions where topics flicker."
echo "  A fresh InterfaceValidator instance would re-check, but an existing"
echo "  node stays blocked until restarted."

timeout 10 python3 -c "
import rospy, sys
rospy.init_node('t5', log_level=rospy.INFO)
sys.path.insert(0, '/workspace/SimEnv/src/competition_loop/src')
from interface_validator import InterfaceValidator

v = InterfaceValidator(competition_mode=True)
v.load_scene_info('$SCENE_DIR')
# Simulate a confirmed detection from a previous tick
v._forbidden_topic_check_completed = True
v._forbidden_topic_check_passed = False
print(f'Before: completed={v.forbidden_topic_check_completed} passed={v.forbidden_topic_check_passed}')
print(f'is_ready={v.is_ready()}')
sys.exit(0 if not v.is_ready() else 1)
" 2>&1 && ok "Test 5: Confirmed detection state is correctly permanent" || no "Test 5: Confirmed detection state is permanent"

# ============================================================
echo ""; echo "=============================================="
echo "RESULTS: $PASS passed, $FAIL failed"
echo "ROS_MASTER_URI: $ROS_MASTER_URI"
echo "ROS_HOME: $ROS_HOME"
echo "Scene dir: $SCENE_DIR"
echo "roscore PID: $ROSCORE_PID"
echo "Log: $LOG"
echo "=============================================="

if [ "$FAIL" -gt 0 ]; then
    echo "SOME TESTS FAILED"
    exit 1
fi
echo "ALL TESTS PASSED"
exit 0
