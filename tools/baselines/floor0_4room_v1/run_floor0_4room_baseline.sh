#!/usr/bin/env bash
# =============================================================================
# Floor0 four-room baseline reproduction entry point
# -----------------------------------------------------------------------------
# VALIDATION / BASELINE REPRODUCTION ONLY — this script does not change any
# production algorithm.  It only selects parameters that already exist in the
# committed launch files.
#
# Frozen result being reproduced:
#     ROOM_SCAN_ZONES_TRIGGERED = 4/4
#     CURRENT_FLOOR_COMPLETE    = PASS
#
# Known limitation (NOT solved by this baseline):
#     RETURN_REACHABILITY = FAIL
#     "main entrance connectivity cut in inflated map during RETURNING
#      replanning"
#
# Usage (inside a ROS environment, from anywhere):
#     bash tools/baselines/floor0_4room_v1/run_floor0_4room_baseline.sh
#
# Requirements:
#   * catkin workspace already built (./devel/setup.bash must exist)
#   * Gazebo / unitree / ROS available (the team container image
#     simenv:noetic-cu128 satisfies this)
# =============================================================================
set -Eeuo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BASELINE_DIR="$ROOT/tools/baselines/floor0_4room_v1"
MANIFEST="$BASELINE_DIR/BASELINE_MANIFEST.md"
LAUNCH="$BASELINE_DIR/floor0_4room_replay.launch"

# Guard rails for the readiness waits (seconds).
SPAWN_TIMEOUT="${SPAWN_TIMEOUT:-600}"
CLOCK_TIMEOUT="${CLOCK_TIMEOUT:-300}"
HANDOFF_TIMEOUT="${HANDOFF_TIMEOUT:-600}"

fail() {
    echo "[baseline] ERROR: $*" >&2
    exit 1
}

info() {
    echo "[baseline] $*"
}

# -----------------------------------------------------------------------------
# 1. Expected commit manifest read
# -----------------------------------------------------------------------------
[ -f "$MANIFEST" ] || fail "missing $MANIFEST"
[ -f "$LAUNCH" ] || fail "missing $LAUNCH"

manifest_value() {
    # Reads "KEY = value" lines from BASELINE_MANIFEST.md.
    sed -n "s/^$1[[:space:]]*=[[:space:]]*//p" "$MANIFEST" | head -1 | tr -d '\r'
}

EXPECTED_BASE="$(manifest_value SUPERPROJECT_BASE)"
EXPECTED_FASTLIO="$(manifest_value FAST_LIO_COMMIT)"
EXPECTED_LIVOX="$(manifest_value LIVOX_COMMIT)"
# SUPERPROJECT_BASELINE_COMMIT is informational: a git commit cannot contain
# its own hash, so the check below anchors on SUPERPROJECT_BASE instead.
RECORDED_TIP="$(manifest_value SUPERPROJECT_BASELINE_COMMIT)"

[ -n "$EXPECTED_BASE" ] || fail "cannot parse SUPERPROJECT_BASE from manifest"
[ -n "$EXPECTED_FASTLIO" ] || fail "cannot parse FAST_LIO_COMMIT from manifest"
[ -n "$EXPECTED_LIVOX" ] || fail "cannot parse LIVOX_COMMIT from manifest"

info "expected superproject base = $EXPECTED_BASE"
info "expected fast_lio  commit  = $EXPECTED_FASTLIO"
info "expected livox     commit  = $EXPECTED_LIVOX"

# -----------------------------------------------------------------------------
# 2. Check the current checkout really is the frozen baseline
# -----------------------------------------------------------------------------
command -v git >/dev/null 2>&1 || fail "git not found"

ACTUAL_HEAD="$(git -C "$ROOT" rev-parse HEAD)"
ACTUAL_FASTLIO="$(git -C "$ROOT/src/fast_lio" rev-parse HEAD 2>/dev/null || echo MISSING)"
ACTUAL_LIVOX="$(git -C "$ROOT/src/livox_ros_driver" rev-parse HEAD 2>/dev/null || echo MISSING)"

# Superproject: must descend from the frozen base, and must not have diverged
# from it in tracked source (only gitlinks / .gitmodules / baseline files are
# allowed to differ).
git -C "$ROOT" cat-file -e "$EXPECTED_BASE^{commit}" 2>/dev/null \
    || fail "frozen base commit $EXPECTED_BASE is not present in this clone"
git -C "$ROOT" merge-base --is-ancestor "$EXPECTED_BASE" "$ACTUAL_HEAD" \
    || fail "HEAD $ACTUAL_HEAD does not descend from frozen base $EXPECTED_BASE"

UNEXPECTED_SRC_DIFF="$(git -C "$ROOT" diff --name-only "$EXPECTED_BASE" "$ACTUAL_HEAD" -- src/ \
    | grep -v -e '^src/fast_lio$' -e '^src/livox_ros_driver$' || true)"
[ -z "$UNEXPECTED_SRC_DIFF" ] \
    || fail "HEAD diverged from the frozen base in source files: $UNEXPECTED_SRC_DIFF"

# Submodules: frozen commits must match exactly.
[ "$ACTUAL_FASTLIO" = "$EXPECTED_FASTLIO" ] \
    || fail "src/fast_lio commit mismatch: have $ACTUAL_FASTLIO want $EXPECTED_FASTLIO"
[ "$ACTUAL_LIVOX" = "$EXPECTED_LIVOX" ] \
    || fail "src/livox_ros_driver commit mismatch: have $ACTUAL_LIVOX want $EXPECTED_LIVOX"

DIRTY_SUBMODULES="$(git -C "$ROOT" status --porcelain -- src/fast_lio src/livox_ros_driver)"
[ -z "$DIRTY_SUBMODULES" ] \
    || fail "submodules are dirty; a clean recursive checkout is required: $DIRTY_SUBMODULES"

info "checkout verified: HEAD=$ACTUAL_HEAD descends from base, submodules match"
[ "$RECORDED_TIP" = "$ACTUAL_HEAD" ] \
    && info "tip matches recorded SUPERPROJECT_BASELINE_COMMIT" \
    || info "note: recorded tip is $RECORDED_TIP while HEAD is $ACTUAL_HEAD"

# -----------------------------------------------------------------------------
# 3. Environment check
# -----------------------------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi
# shellcheck disable=SC1091
[ -f "$ROOT/devel/setup.bash" ] || fail "workspace not built: missing $ROOT/devel/setup.bash (build first)"
source "$ROOT/devel/setup.bash"

[ -n "${ROS_DISTRO:-}" ] || fail "ROS not sourced"
command -v roslaunch >/dev/null 2>&1 || fail "roslaunch not available"
info "ROS $ROS_DISTRO ready; workspace devel overlay sourced"

# -----------------------------------------------------------------------------
# 4. Explicit, frozen environment (never inherited from the caller's shell)
# -----------------------------------------------------------------------------
export SEED=77
export FLOOR_COUNT=1
export ROOMS_PER_FLOOR=4
export ENABLE_GROUND_TRUTH=0
export ENABLE_REFEREE_ODOM=0
export WRITE_GENERATED_TRUTH_COPY=0
export UNITREE_AUTO_CMD_VEL=1
export CONTROLLER_FOREGROUND=0
export GUI="${GUI:-false}"

info "env frozen: SEED=$SEED FLOOR_COUNT=$FLOOR_COUNT ROOMS_PER_FLOOR=$ROOMS_PER_FLOOR"
info "env frozen: ENABLE_GROUND_TRUTH=$ENABLE_GROUND_TRUTH ENABLE_REFEREE_ODOM=$ENABLE_REFEREE_ODOM"
info "env frozen: WRITE_GENERATED_TRUTH_COPY=$WRITE_GENERATED_TRUTH_COPY"
info "env frozen: UNITREE_AUTO_CMD_VEL=$UNITREE_AUTO_CMD_VEL CONTROLLER_FOREGROUND=$CONTROLLER_FOREGROUND GUI=$GUI"

cd "$ROOT"

# -----------------------------------------------------------------------------
# 5. Remove stale logs so readiness is NEVER judged from an old run
# -----------------------------------------------------------------------------
RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="$ROOT/artifacts/baseline-floor0-4room-v1-$RUN_STAMP"
mkdir -p "$RUN_DIR"
rm -f "$ROOT/logs/junior_ctrl.log" \
      "$ROOT/logs/competition_gazebo.log" \
      "$ROOT/logs/building_control.log"
info "stale logs removed; this run writes to $RUN_DIR"

# -----------------------------------------------------------------------------
# 6. Gazebo + robot spawn (scene generation included), backgrounded
# -----------------------------------------------------------------------------
if [ ! -x "$ROOT/auto.sh" ]; then
    fail "missing or non-executable $ROOT/auto.sh"
fi

info "starting Gazebo / spawn / controller via auto.sh"
setsid bash ./auto.sh > "$RUN_DIR/auto_stdout.log" 2>&1 &
AUTO_PID=$!
echo "$AUTO_PID" > "$RUN_DIR/auto.pid"

cleanup() {
    if kill -0 "$AUTO_PID" 2>/dev/null; then
        info "stopping auto.sh (pid $AUTO_PID)"
        kill "$AUTO_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

# -----------------------------------------------------------------------------
# 7. Readiness gate: /use_sim_time, advancing /clock, RL_HANDOFF_READY
#    Written fresh each run -> no stale-log judgement is possible.
# -----------------------------------------------------------------------------
PROBE="$RUN_DIR/wait_ready.py"
cat > "$PROBE" <<'PY'
import sys, time
import rostopic
import rospy

clock_to, handoff_to, spawn_to = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
log_path, run_epoch = sys.argv[4], float(sys.argv[5])

rospy.init_node("floor0_baseline_ready_probe", anonymous=True, disable_signals=True)

# --- /use_sim_time --------------------------------------------------------
deadline = time.time() + clock_to
while time.time() < deadline:
    try:
        if bool(rospy.get_param("/use_sim_time", False)):
            break
    except Exception:
        pass
    time.sleep(1.0)
else:
    print("[probe] FAIL: /use_sim_time was never true", flush=True)
    sys.exit(2)
print("[probe] /use_sim_time=true", flush=True)

# --- robot spawn ----------------------------------------------------------
import rosservice
deadline = time.time() + spawn_to
spawned = False
while time.time() < deadline:
    try:
        rosservice.get_service_class
        from gazebo_msgs.srv import GetModelState
        rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
        get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        r = get(model_name="a1_gazebo", relative_entity_name="world")
        if r.success:
            spawned = True
            print("[probe] robot a1_gazebo spawned", flush=True)
            break
    except Exception:
        time.sleep(2.0)
if not spawned:
    print("[probe] FAIL: robot spawn not confirmed", flush=True)
    sys.exit(3)

# --- /clock advancing -----------------------------------------------------
try:
    from rosgraph_msgs.msg import Clock
except Exception:
    Clock = None

def wait_clock(timeout):
    if Clock is None:
        return (None, None)
    samples = []
    end = time.time() + timeout
    def cb(msg):
        samples.append(msg.clock.to_sec())
    sub = rospy.Subscriber("/clock", Clock, cb)
    while time.time() < end and len(samples) < 2:
        time.sleep(0.5)
    # need strictly increasing samples
    last = len(samples)
    deadline = time.time() + timeout
    while time.time() < deadline:
        time.sleep(1.0)
        if len(samples) >= 2 and samples[-1] > samples[0]:
            sub.unregister()
            return (samples[0], samples[-1])
        if len(samples) == 0:
            continue
    sub.unregister()
    return (samples[0] if samples else None, samples[-1] if samples else None)

first, last = wait_clock(clock_to)
if first is None or last is None or not (last > first):
    print("[probe] FAIL: /clock not advancing (first=%s last=%s)" % (first, last), flush=True)
    sys.exit(4)
print("[probe] /clock advancing: %s -> %s" % (first, last), flush=True)

# --- RL_HANDOFF_READY from THIS run's log only ----------------------------
import os
import re
pattern = re.compile(r"RL_HANDOFF_READY=True")
offset = 0
deadline = time.time() + handoff_to
while time.time() < deadline:
    try:
        if os.path.exists(log_path):
            st = os.stat(log_path)
            if st.st_mtime >= run_epoch - 1:
                with open(log_path, "r", errors="ignore") as fh:
                    if "RL_HANDOFF_READY=True" in fh.read():
                        print("[probe] RL_HANDOFF_READY=True observed in fresh log", flush=True)
                        print("[probe] READY", flush=True)
                        sys.exit(0)
    except Exception:
        pass
    time.sleep(2.0)
print("[probe] FAIL: RL_HANDOFF_READY=True not seen within timeout", flush=True)
sys.exit(5)
PY

RUN_EPOCH="$(date +%s)"
info "waiting for sim readiness (use_sim_time + advancing /clock + RL_HANDOFF_READY)"
set +e
python3 "$PROBE" "$CLOCK_TIMEOUT" "$HANDOFF_TIMEOUT" "$SPAWN_TIMEOUT" \
    "$ROOT/logs/junior_ctrl.log" "$RUN_EPOCH" 2>&1 | tee "$RUN_DIR/readiness.log"
PROBE_RC="${PIPESTATUS[0]}"
set -e
[ "$PROBE_RC" -eq 0 ] || fail "sim never became ready (probe rc=$PROBE_RC); see $RUN_DIR/readiness.log"
info "sim ready"

# -----------------------------------------------------------------------------
# 8. Start the frozen Floor0 competition stack
# -----------------------------------------------------------------------------
info "launching Floor0 four-room baseline stack"
info "  max_time request = 1800.0 s (validation watchdog; formal budget is 600.0 s)"
info "  verify with: rosparam get /competition_loop/max_time"

set +e
roslaunch "$LAUNCH" 2>&1 | tee "$RUN_DIR/competition_stack.log"
LAUNCH_RC="${PIPESTATUS[0]}"
set -e

echo
info "effective /competition_loop/max_time check (may be 600.0 — see README caveat):"
rosparam get /competition_loop/max_time 2>/dev/null || true

[ "$LAUNCH_RC" -eq 0 ] || fail "competition stack exited rc=$LAUNCH_RC"
info "baseline run finished; evidence in $RUN_DIR"
