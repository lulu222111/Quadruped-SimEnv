#!/usr/bin/env bash
# =============================================================================
# floor0-doorfix-closure-v1 — Floor0 full-closure baseline reproduction runner
# -----------------------------------------------------------------------------
# VALIDATION / BASELINE REPRODUCTION ONLY.
#   * Changes NO production algorithm and NO production file.
#   * Selects only parameters that already exist in the committed launch files.
#   * Creates no file outside <repo>/tools/baselines/floor0-doorfix-closure-v1
#     and its own run directory under <repo>/artifacts/.
#
# Frozen identity, resolved parameters and provenance: see BASELINE_MANIFEST.md.
# Every frozen value used below is PARSED FROM THE MANIFEST so there is exactly
# one source of truth.
#
# Usage:
#   bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh
#   bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh --dry-run
#
# Environment overrides (all optional):
#   DRY_RUN=1              run the pre-flight gates only, start nothing
#   ALLOW_PRE_CLEAN=1      kill stale ROS/Gazebo processes instead of aborting
#   GUI=0                  headless deviation (reported as DEVIATION, not PASS)
#   RECORD_BAG=0           do not rosbag-record the mission topics
#   RUN_WALL_TIMEOUT=10800 wall-clock watchdog for the mission (seconds)
#   RUN_DIR=<path>         write evidence somewhere else
#
# Exit status: 0 only when EVERY gate passes.  Every gate starts as NOT_RUN and
# the run aborts at the first FAIL, so a green report means each gate was
# actually evaluated.
# =============================================================================
# NOTE: deliberately NOT `set -u` — the ROS/catkin profile scripts are not
# unbound-variable safe (e.g. 10.roslaunch.sh references ROS_MASTER_URI).
set -Eeo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/../../.." && pwd)"
BASELINE_DIR="$ROOT/tools/baselines/floor0-doorfix-closure-v1"
MANIFEST="$BASELINE_DIR/BASELINE_MANIFEST.md"
LAUNCH="$BASELINE_DIR/floor0_doorfix_closure.launch"

DRY_RUN="${DRY_RUN:-0}"
ALLOW_PRE_CLEAN="${ALLOW_PRE_CLEAN:-0}"
RECORD_BAG="${RECORD_BAG:-1}"
RUN_WALL_TIMEOUT="${RUN_WALL_TIMEOUT:-10800}"

usage() {
    sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
    exit 0
}

for _arg in "$@"; do
    case "$_arg" in
        --dry-run)          DRY_RUN=1 ;;
        --allow-pre-clean)  ALLOW_PRE_CLEAN=1 ;;
        -h|--help)          usage ;;
        *) echo "[closure] FATAL: unknown argument '$_arg'" >&2; exit 2 ;;
    esac
done

# -----------------------------------------------------------------------------
# Gate bookkeeping
# -----------------------------------------------------------------------------
GATE_ORDER=(HEAD_SOURCE_GATE SUBMODULE_GATE BUILD_GATE PROCESS_GATE SCENE_GATE \
            DOOR_GATE SIM_READY_GATE RGBD_GATE GUI_GATE MAX_TIME_GATE \
            GT_COMPLIANCE_GATE CLOSURE_GATE)
declare -A GATE_STATE GATE_NOTE
for _g in "${GATE_ORDER[@]}"; do
    GATE_STATE["$_g"]="NOT_RUN"
    GATE_NOTE["$_g"]=""
done

pass_gate() {
    GATE_STATE["$1"]="PASS"
    GATE_NOTE["$1"]="$2"
    echo "[closure] GATE PASS : $1 — $2"
}

deviation_gate() {
    GATE_STATE["$1"]="DEVIATION"
    GATE_NOTE["$1"]="$2"
    echo "[closure] GATE DEV  : $1 — $2"
}

fail_gate() {
    GATE_STATE["$1"]="FAIL"
    GATE_NOTE["$1"]="$2"
    echo "[closure] GATE FAIL : $1 — $2" >&2
    report_and_exit 1
}

report_and_exit() {
    local rc="$1" g
    echo
    echo "================ FLOOR0 DOORFIX CLOSURE — GATE REPORT ================"
    printf "%-22s %-10s %s\n" GATE STATE DETAIL
    for g in "${GATE_ORDER[@]}"; do
        printf "%-22s %-10s %s\n" "$g" "${GATE_STATE[$g]}" "${GATE_NOTE[$g]}"
    done
    echo "======================================================================"
    if [ "$rc" -eq 0 ]; then
        echo "[closure] RESULT: ALL GATES PASSED — evidence in ${RUN_DIR:-<dry-run>}"
    else
        echo "[closure] RESULT: FAILED (exit $rc) — evidence in ${RUN_DIR:-<dry-run>}"
    fi
    exit "$rc"
}

info() { echo "[closure] $*"; }

# -----------------------------------------------------------------------------
# 0. Files + frozen values parsed from the manifest
# -----------------------------------------------------------------------------
[ -f "$MANIFEST" ] || { echo "[closure] FATAL: missing $MANIFEST" >&2; exit 1; }
[ -f "$LAUNCH"   ] || { echo "[closure] FATAL: missing $LAUNCH" >&2; exit 1; }

manifest_value() {
    sed -n "s/^$1[[:space:]]*=[[:space:]]*//p" "$MANIFEST" | head -1 | tr -d '\r'
}

FROZEN_BASE="$(manifest_value PRODUCTION_BASE_HEAD)"
FROZEN_SCOPE="$(manifest_value TOOLING_SCOPE)"
FROZEN_FASTLIO="$(manifest_value FAST_LIO)"
FROZEN_LIVOX="$(manifest_value LIVOX)"
FROZEN_SEED="$(manifest_value SEED)"
FROZEN_FLOORS="$(manifest_value FLOOR_COUNT)"
FROZEN_ROOMS="$(manifest_value ROOMS_PER_FLOOR)"
FROZEN_GT="$(manifest_value ENABLE_GROUND_TRUTH)"
FROZEN_REF_ODOM="$(manifest_value ENABLE_REFEREE_ODOM)"
FROZEN_TRUTH_COPY="$(manifest_value WRITE_GENERATED_TRUTH_COPY)"
FROZEN_AUTO_CMD_VEL="$(manifest_value UNITREE_AUTO_CMD_VEL)"
FROZEN_CTRL_FG="$(manifest_value CONTROLLER_FOREGROUND)"
FROZEN_GUI="$(manifest_value GUI)"
FROZEN_REALSENSE="$(manifest_value ENABLE_REALSENSE)"
FROZEN_SENSORS="$(manifest_value ENABLE_SENSORS)"
FROZEN_MAX_TIME="$(manifest_value MAX_TIME)"
FROZEN_WIDTH="$(manifest_value BUILDING_WIDTH)"
FROZEN_LENGTH="$(manifest_value BUILDING_LENGTH)"
FROZEN_DANGER_RANGE="$(manifest_value DANGER_COUNT_RANGE)"
FROZEN_DISTRACTOR_RANGE="$(manifest_value DISTRACTOR_COUNT_RANGE)"
FROZEN_SCENE_DANGER="$(manifest_value SCENE_DANGER_COUNT)"
FROZEN_SCENE_DISTRACTOR="$(manifest_value SCENE_DISTRACTOR_COUNT)"
FROZEN_CLEARANCE="$(manifest_value MAIN_ENTRANCE_CLEARANCE)"
GUI="${GUI:-$FROZEN_GUI}"
FROZEN_ROS_MASTER="$(manifest_value ROS_MASTER_URI)"
FROZEN_GAZEBO_MASTER="$(manifest_value GAZEBO_MASTER_URI)"

for _pair in "PRODUCTION_BASE_HEAD:$FROZEN_BASE" "TOOLING_SCOPE:$FROZEN_SCOPE" \
             "FAST_LIO:$FROZEN_FASTLIO" \
             "LIVOX:$FROZEN_LIVOX" "SEED:$FROZEN_SEED" "FLOOR_COUNT:$FROZEN_FLOORS" \
             "ROOMS_PER_FLOOR:$FROZEN_ROOMS" "MAX_TIME:$FROZEN_MAX_TIME" \
             "MAIN_ENTRANCE_CLEARANCE:$FROZEN_CLEARANCE" "SCENE_DANGER_COUNT:$FROZEN_SCENE_DANGER" \
             "SCENE_DISTRACTOR_COUNT:$FROZEN_SCENE_DISTRACTOR" \
             "ENABLE_GROUND_TRUTH:$FROZEN_GT" "ENABLE_REFEREE_ODOM:$FROZEN_REF_ODOM" \
             "GUI:$FROZEN_GUI" "BUILDING_WIDTH:$FROZEN_WIDTH" "BUILDING_LENGTH:$FROZEN_LENGTH"; do
    [ -n "${_pair#*:}" ] || { echo "[closure] FATAL: cannot parse ${_pair%%:*} from manifest" >&2; exit 1; }
done

info "baseline        : $BASELINE_DIR"
info "frozen identity : PRODUCTION_BASE_HEAD=$FROZEN_BASE (ancestor of HEAD)"
info "tooling scope   : $FROZEN_SCOPE (only tracked changes allowed here)"
info "frozen scene    : SEED=$FROZEN_SEED FLOOR_COUNT=$FROZEN_FLOORS ROOMS_PER_FLOOR=$FROZEN_ROOMS"
info "frozen budget   : /competition_loop/max_time=$FROZEN_MAX_TIME (resolved value, verified live)"
info "frozen door     : main entrance clearance=${FROZEN_CLEARANCE} m"

# -----------------------------------------------------------------------------
# 1. HEAD_SOURCE_GATE — production base is an ancestor, and the only tracked
#    difference since it is the reproduction tooling.
# -----------------------------------------------------------------------------
command -v git >/dev/null 2>&1 || fail_gate HEAD_SOURCE_GATE "git not found"
GIT_ERR="$(git -C "$ROOT" rev-parse --git-dir 2>&1 >/dev/null)"
if [ -n "$GIT_ERR" ]; then
    fail_gate HEAD_SOURCE_GATE "git metadata unreadable in $ROOT ($GIT_ERR) — use a plain clone (or mount a worktree's parent repo)"
fi

ACTUAL_HEAD="$(git -C "$ROOT" rev-parse HEAD 2>/dev/null || echo MISSING)"
[ "$ACTUAL_HEAD" != "MISSING" ] || fail_gate HEAD_SOURCE_GATE "cannot resolve HEAD in $ROOT"

# (1) ancestry: the frozen production commit must be an ancestor of HEAD.
if ! git -C "$ROOT" cat-file -e "$FROZEN_BASE^{commit}" 2>/dev/null; then
    fail_gate HEAD_SOURCE_GATE "PRODUCTION_BASE_HEAD $FROZEN_BASE is not present in this clone (fetch it first)"
fi
if ! git -C "$ROOT" merge-base --is-ancestor "$FROZEN_BASE" "$ACTUAL_HEAD"; then
    fail_gate HEAD_SOURCE_GATE "HEAD=$ACTUAL_HEAD does not descend from PRODUCTION_BASE_HEAD=$FROZEN_BASE"
fi

# Tracked __pycache__ byte-code caches are excluded: CPython regenerates them
# on import after a fresh checkout (source mtime changes), so they differ after
# any run regardless of the algorithm.  They are reported, not ignored silently.
ALL_DIFF="$(git -C "$ROOT" diff --name-only "$FROZEN_BASE" "$ACTUAL_HEAD" 2>/dev/null || true)"
PYC_DIFF="$(printf '%s\n' "$ALL_DIFF" | grep '__pycache__' || true)"
REAL_DIFF="$(printf '%s\n' "$ALL_DIFF" | grep -v '__pycache__' || true)"
if [ -n "$PYC_DIFF" ]; then
    info "note: tracked CPython byte-code caches differ from the base (regenerated on import): $(printf '%s' "$PYC_DIFF" | tr '\n' ' ')"
fi

# (2) every other tracked change must live inside the tooling scope.
UNEXPECTED_DIFF="$(printf '%s\n' "$REAL_DIFF" | grep -v "^${FROZEN_SCOPE}/" || true)"
if [ -n "$UNEXPECTED_DIFF" ]; then
    echo "[closure] tracked changes outside $FROZEN_SCOPE since $FROZEN_BASE:" >&2
    printf '%s\n' "$UNEXPECTED_DIFF" | sed 's/^/[closure]   /' >&2
    fail_gate HEAD_SOURCE_GATE "HEAD=$ACTUAL_HEAD changes tracked files outside the tooling scope"
fi

# (3) production / runtime paths must be byte-identical to the base.
PRODUCTION_PATHS=(src auto.sh setup_multi_floor_simulation.py docker compose.yaml scripts .gitmodules)
PROD_DIFF=""
for _p in "${PRODUCTION_PATHS[@]}"; do
    if [ "$_p" = "src" ]; then
        _d="$(printf '%s\n' "$ALL_DIFF" | grep '^src/' | grep -v '__pycache__' || true)"
    else
        _d="$(printf '%s\n' "$ALL_DIFF" | grep -E "^${_p}(/|$)" || true)"
    fi
    if [ -n "$_d" ]; then PROD_DIFF+="$_d"$'\n'; fi
done
if [ -n "$PROD_DIFF" ]; then
    echo "[closure] production/runtime files differ from PRODUCTION_BASE_HEAD:" >&2
    printf '%s' "$PROD_DIFF" | sed 's/^/[closure]   /' >&2
    fail_gate HEAD_SOURCE_GATE "production source is not identical to $FROZEN_BASE"
fi

DIRTY_TRACKED="$(git -C "$ROOT" status --porcelain --untracked-files=no | grep -v '__pycache__' | wc -l | tr -d ' ' || true)"
[ "$DIRTY_TRACKED" = "0" ] || info "note: $DIRTY_TRACKED tracked file(s) are dirty in the working tree (uncommitted; not part of the base-vs-HEAD audit)"

TOOLING_DIFF_COUNT="$(printf '%s\n' "$REAL_DIFF" | grep -c . || true)"
pass_gate HEAD_SOURCE_GATE "HEAD=$ACTUAL_HEAD descends from $FROZEN_BASE; ${TOOLING_DIFF_COUNT} tracked change(s), all inside ${FROZEN_SCOPE}; production paths zero-diff"

# -----------------------------------------------------------------------------
# 2. SUBMODULE_GATE — exact submodule commits, clean worktrees
# -----------------------------------------------------------------------------
check_submodule() {
    local path="$1" want="$2" got
    got="$(git -C "$ROOT/$path" rev-parse HEAD 2>/dev/null || echo MISSING)"
    if [ "$got" != "$want" ]; then
        echo "MISMATCH:$path:have=$got:want=$want"
        return 1
    fi
    local dirty
    dirty="$(git -C "$ROOT" status --porcelain -- "$path")"
    if [ -n "$dirty" ]; then
        echo "DIRTY:$path:$dirty"
        return 1
    fi
    echo "OK:$path:$got"
    return 0
}

R1="$(check_submodule src/fast_lio "$FROZEN_FASTLIO")" || fail_gate SUBMODULE_GATE "$R1"
R2="$(check_submodule src/livox_ros_driver "$FROZEN_LIVOX")" || fail_gate SUBMODULE_GATE "$R2"
pass_gate SUBMODULE_GATE "fast_lio=${R1##*:} livox=${R2##*:} (exact, clean)"

# -----------------------------------------------------------------------------
# 3. BUILD_GATE — required workspace build exists
# -----------------------------------------------------------------------------
if [ -z "${ROS_DISTRO:-}" ] && [ -f /opt/ros/noetic/setup.bash ]; then
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
fi
if [ ! -f /opt/ros/noetic/setup.bash ]; then
    fail_gate BUILD_GATE "ROS noetic not found at /opt/ros/noetic — run this inside the simenv container"
fi
if [ ! -f "$ROOT/devel/setup.bash" ]; then
    fail_gate BUILD_GATE "workspace not built: missing $ROOT/devel/setup.bash (run catkin_make first)"
fi
# shellcheck disable=SC1091
source "$ROOT/devel/setup.bash"
[ -n "${ROS_DISTRO:-}" ] || fail_gate BUILD_GATE "ROS environment did not load from $ROOT/devel/setup.bash"

MISSING_PKGS=""
for _pkg in competition_bringup competition_loop competition_localization \
            competition_planning competition_perception competition_mapping \
            competition_control fast_lio livox_ros_driver unitree_guide; do
    if ! rospack find "$_pkg" >/dev/null 2>&1; then
        MISSING_PKGS="$MISSING_PKGS $_pkg"
    fi
done
[ -z "$MISSING_PKGS" ] || fail_gate BUILD_GATE "catkin packages not built:$MISSING_PKGS"

REQUIRED_ARTIFACTS=(
    "devel/lib/fast_lio/fastlio_mapping"
    "devel/lib/competition_localization/localization_node"
    "src/competition_bringup/launch/competition_system.launch"
    "src/competition_localization/launch/localization_corrected.launch"
    "src/unitree_guide/unitree_guide/unitree_guide/launch/multi_floor_gazeboSim.launch"
    "auto.sh"
)
MISSING_ART=""
for _art in "${REQUIRED_ARTIFACTS[@]}"; do
    [ -e "$ROOT/$_art" ] || MISSING_ART="$MISSING_ART $_art"
done
[ -z "$MISSING_ART" ] || fail_gate BUILD_GATE "required build artifacts missing:$MISSING_ART"
pass_gate BUILD_GATE "devel overlay + 10 packages + launch/nodes present (ROS $ROS_DISTRO)"

# -----------------------------------------------------------------------------
# 4. PROCESS_GATE — no stale ROS/Gazebo process contamination
# -----------------------------------------------------------------------------
STALE_PATTERNS=(gzserver gzclient gazebo rosmaster roscore junior_ctrl \
                competition_loop laserMapping building_generator_classic_control \
                "rosbag record")

# One `ps` snapshot; zombies (state Z) are NOT contamination and are skipped,
# and this script's own processes are excluded so the scan can never match
# itself.
stale_scan() {
    local pid stat args pat out=""
    # `read` (default IFS) strips the leading padding ps prints for pid/stat
    # and puts the whole command line into `args`.
    while read -r pid stat args; do
        [ -n "$pid" ] || continue
        case "$stat" in *Z*) continue ;; esac
        [ "$pid" = "$$" ] && continue
        case "$args" in
            *run_floor0_doorfix_closure*|*stale_scan*|*"ps -eo pid="*) continue ;;
        esac
        for pat in "${STALE_PATTERNS[@]}"; do
            case "$args" in *"$pat"*) out+="${pid} ${args}"$'\n' ;; esac
        done
    done < <(ps -eo pid=,stat=,args= 2>/dev/null)
    printf '%s' "$out" | awk 'NF && !seen[$1]++'
}

# Exact process-name liveness check that ignores zombies.
proc_alive() {
    ps -eo stat=,comm= 2>/dev/null | awk -v n="$1" \
        '$1 !~ /Z/ && $2 == n { found = 1 } END { exit(found ? 0 : 1) }'
}

STALE="$(stale_scan)"
if [ -n "$STALE" ]; then
    if [ "$ALLOW_PRE_CLEAN" = "1" ]; then
        info "ALLOW_PRE_CLEAN=1 — terminating stale processes before starting:"
        echo "$STALE" | sed 's/^/[closure]   /'
        echo "$STALE" | awk '{print $1}' | xargs -r kill -TERM 2>/dev/null || true
        sleep 3
        echo "$STALE" | awk '{print $1}' | xargs -r kill -KILL 2>/dev/null || true
        sleep 2
        STALE2="$(stale_scan)"
        [ -z "$STALE2" ] || fail_gate PROCESS_GATE "processes still alive after pre-clean: $(echo "$STALE2" | awk '{print $1}' | tr '\n' ' ')"
    else
        echo "[closure] stale processes detected:" >&2
        echo "$STALE" | sed 's/^/[closure]   /' >&2
        fail_gate PROCESS_GATE "stale ROS/Gazebo processes present — stop them or re-run with ALLOW_PRE_CLEAN=1"
    fi
fi
pass_gate PROCESS_GATE "no gzserver/gzclient/rosmaster/junior_ctrl/competition_loop/rosbag running"

# -----------------------------------------------------------------------------
# 4b. GUI pre-check (GUI_GATE is finalised once Gazebo is up)
# -----------------------------------------------------------------------------
if [ "$GUI" = "1" ] || [ "$GUI" = "true" ]; then
    if [ -z "${DISPLAY:-}" ]; then
        fail_gate GUI_GATE "GUI=1 but DISPLAY is empty — the frozen run renders gzclient on a real X display (or set GUI=0 to record an explicit deviation)"
    fi
    info "GUI=1 DISPLAY=$DISPLAY (gzclient must be alive once Gazebo is up)"
else
    deviation_gate GUI_GATE "GUI=0 requested — DEVIATES from the frozen evidence (gzclient/disabled); run is still executed"
fi

# -----------------------------------------------------------------------------
# Dry run: stop here, nothing has been started.
# -----------------------------------------------------------------------------
if [ "$DRY_RUN" = "1" ]; then
    info "DRY_RUN=1 — pre-flight gates only; no sim, no stack, no scene regeneration"
    report_and_exit 0
fi

# =============================================================================
# Frozen environment (never inherited from the caller's shell)
# =============================================================================
export ROS_MASTER_URI="$FROZEN_ROS_MASTER"
export GAZEBO_MASTER_URI="$FROZEN_GAZEBO_MASTER"
export ROS_HOME="${ROS_HOME:-$ROOT/artifacts/ros}"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/tmp/.Xauthority}"

export SEED="$FROZEN_SEED"
export FLOOR_COUNT="$FROZEN_FLOORS"
export ROOMS_PER_FLOOR="$FROZEN_ROOMS"
export ENABLE_GROUND_TRUTH="$FROZEN_GT"
export ENABLE_REFEREE_ODOM="$FROZEN_REF_ODOM"
export WRITE_GENERATED_TRUTH_COPY="$FROZEN_TRUTH_COPY"
export UNITREE_AUTO_CMD_VEL="$FROZEN_AUTO_CMD_VEL"
export CONTROLLER_FOREGROUND="$FROZEN_CTRL_FG"
export ENABLE_REALSENSE="$FROZEN_REALSENSE"
export ENABLE_SENSORS="$FROZEN_SENSORS"
export BUILDING_WIDTH="$FROZEN_WIDTH"
export BUILDING_LENGTH="$FROZEN_LENGTH"
export DANGER_COUNT="$FROZEN_DANGER_RANGE"
export DISTRACTOR_COUNT="$FROZEN_DISTRACTOR_RANGE"
# Robot start pose, frozen explicitly (identical to the authoritative run).
export ROBOT_X=0.0
export ROBOT_Y=-3.2
export ROBOT_Z=0.6
export ROBOT_YAW=1.5708
# Gazebo GUI: the frozen run is observable on a real X display.
if [ "$GUI" = "1" ] || [ "$GUI" = "true" ]; then export GUI=true; else export GUI=false; fi

RUN_STAMP="$(date +%Y%m%d-%H%M%S)"
RUN_DIR="${RUN_DIR:-$ROOT/artifacts/baseline-floor0-doorfix-closure-v1-$RUN_STAMP}"
mkdir -p "$RUN_DIR"
cp -f "$MANIFEST" "$RUN_DIR/BASELINE_MANIFEST.md"
cp -f "$LAUNCH" "$RUN_DIR/floor0_doorfix_closure.launch"
RUN_EPOCH="$(date +%s)"
echo "RUN_START_WALL=$(date -Iseconds)" > "$RUN_DIR/run_timeline.txt"
echo "RUN_EPOCH=$RUN_EPOCH"            >> "$RUN_DIR/run_timeline.txt"
info "frozen env: SEED=$SEED FLOOR_COUNT=$FLOOR_COUNT ROOMS_PER_FLOOR=$ROOMS_PER_FLOOR GUI=$GUI"
info "frozen env: ENABLE_GROUND_TRUTH=$ENABLE_GROUND_TRUTH ENABLE_REFEREE_ODOM=$ENABLE_REFEREE_ODOM WRITE_GENERATED_TRUTH_COPY=$WRITE_GENERATED_TRUTH_COPY"
info "run dir   : $RUN_DIR"

cd "$ROOT"

# Stale logs are removed BEFORE auto.sh starts: auto.sh truncates
# junior_ctrl.log only once junior_ctrl actually runs, so a leftover log would
# satisfy the readiness grep immediately.
info "removing stale logs so readiness cannot be judged from an old run"
rm -f "$ROOT/logs/junior_ctrl.log" \
      "$ROOT/logs/competition_gazebo.log" \
      "$ROOT/logs/building_control.log"

# -----------------------------------------------------------------------------
# 5. Start Gazebo + scene generation + robot + controller (production auto.sh)
# -----------------------------------------------------------------------------
info "starting sim via ./auto.sh (scene generation included)"
setsid bash ./auto.sh > "$RUN_DIR/auto_stdout.log" 2>&1 &
AUTO_PID=$!
echo "$AUTO_PID" > "$RUN_DIR/auto.pid"

cleanup() {
    local pid
    for pid in "${STACK_PID:-}" "${BAG_PID:-}" "${SAMPLER_PID:-}" "$AUTO_PID"; do
        [ -n "$pid" ] || continue
        kill -TERM "$pid" 2>/dev/null || true
    done
    sleep 2
    for pid in "${STACK_PID:-}" "${BAG_PID:-}" "${SAMPLER_PID:-}" "$AUTO_PID"; do
        [ -n "$pid" ] || continue
        kill -KILL "$pid" 2>/dev/null || true
    done
}
trap cleanup EXIT

# -----------------------------------------------------------------------------
# 6. SCENE_GATE — generated scene really is SEED=77 / Floor0 / 4 rooms
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/check_scene.py" <<'PY'
import json, sys, time
root, epoch = sys.argv[1], float(sys.argv[2])
want_seed, want_floors, want_rooms = int(sys.argv[3]), int(sys.argv[4]), int(sys.argv[5])
want_danger, want_distractor = int(sys.argv[6]), int(sys.argv[7])
timeout = float(sys.argv[8]) if len(sys.argv) > 8 else 300.0

manifest_p = root + "/generated_building/scene_manifest.json"
config_p = root + "/generated_building/building_config.json"
deadline = time.time() + timeout
while time.time() < deadline:
    try:
        m_ok = __import__("os").path.getmtime(manifest_p) >= epoch
        c_ok = __import__("os").path.getmtime(config_p) >= epoch
        if m_ok and c_ok:
            break
    except OSError:
        pass
    time.sleep(2.0)
else:
    print("[scene] RESULT=FAIL freshly generated scene_manifest.json/building_config.json not seen")
    sys.exit(1)

man = json.load(open(manifest_p))
cfg = json.load(open(config_p))
checks = [
    ("manifest.seed", man.get("seed"), want_seed),
    ("building.seed", cfg.get("seed"), want_seed),
    ("building.num_floors", cfg.get("num_floors"), want_floors),
    ("building.room_count", cfg.get("room_count"), want_rooms),
    ("manifest.danger_count", man.get("danger_count"), want_danger),
    ("manifest.distractor_count", man.get("distractor_count"), want_distractor),
]
bad = []
for name, got, want in checks:
    ok = str(got) == str(want)
    print("[scene] %-26s = %-8s want=%s %s" % (name, got, want, "OK" if ok else "MISMATCH"))
    if not ok:
        bad.append(name)
print("[scene] world=%s" % man.get("world_file"))
print("[scene] robot_start=%s" % man.get("robot_start"))
if bad:
    print("[scene] RESULT=FAIL mismatched: %s" % ",".join(bad))
    sys.exit(1)
print("[scene] RESULT=PASS SEED=%s Floor0 rooms=%s dangers=%s distractors=%s"
      % (want_seed, want_rooms, want_danger, want_distractor))
PY

set +e
python3 "$RUN_DIR/check_scene.py" "$ROOT" "$RUN_EPOCH" "$FROZEN_SEED" \
    "$FROZEN_FLOORS" "$FROZEN_ROOMS" "$FROZEN_SCENE_DANGER" \
    "$FROZEN_SCENE_DISTRACTOR" 300 2>&1 | tee "$RUN_DIR/scene_check.txt"
SCENE_RC="${PIPESTATUS[0]}"
set -e
[ "$SCENE_RC" -eq 0 ] || fail_gate SCENE_GATE "generated scene is not SEED=$FROZEN_SEED / Floor0 / $FROZEN_ROOMS rooms (see $RUN_DIR/scene_check.txt)"
pass_gate SCENE_GATE "SEED=$FROZEN_SEED num_floors=$FROZEN_FLOORS rooms=$FROZEN_ROOMS dangers=$FROZEN_SCENE_DANGER distractors=$FROZEN_SCENE_DISTRACTOR"

# -----------------------------------------------------------------------------
# 7. DOOR_GATE — main entrance clearance must be 2.0 m in the GENERATED scene
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/check_door.py" <<'PY'
"""Recompute the main-entrance open clearance from this run's generated
door_config.yaml, using production's own geometry:
    panel_width   = width / 2
    inner_edge    = |panel_open_y| - panel_width / 2
    clearance     = min(width, 2 * inner_edge)
and production's own doorway criterion (exporter._validate_door_config):
    |open_y| >= width/2 + width/4
"""
import os, sys
import yaml

root = sys.argv[1]
expected = float(sys.argv[2])
path = os.path.join(root, "generated_building", "door_config.yaml")
cfg = yaml.safe_load(open(path))
doors = cfg.get("doors", []) if isinstance(cfg, dict) else []
door = next((d for d in doors if d.get("id") == "main_entrance"), None)
if door is None:
    print("[door] RESULT=FAIL main_entrance not present in %s" % path)
    sys.exit(1)

width = float(door["width"])
panel_width = width / 2.0
pp = door.get("panel_poses", {}) or {}
lo, ro = pp.get("left_open"), pp.get("right_open")
if lo is None or ro is None:
    print("[door] RESULT=FAIL main_entrance has no open panel poses")
    sys.exit(1)
inner = min(abs(float(lo[1])) - panel_width / 2.0,
            abs(float(ro[1])) - panel_width / 2.0)
clearance = max(0.0, min(width, 2.0 * inner))
required_offset = width / 2.0 + width / 4.0
clears = (abs(float(lo[1])) >= required_offset - 1e-9
          and abs(float(ro[1])) >= required_offset - 1e-9)

sdf = os.path.join(root, "generated_building", "model.sdf")
apron = False
try:
    apron = "entrance_apron" in open(sdf, errors="ignore").read()
except OSError:
    pass

print("[door] id                  = %s" % door.get("id"))
print("[door] width (jamb)        = %.3f m" % width)
print("[door] panel leaf length   = %.3f m" % panel_width)
print("[door] left_open  y        = %.4f" % float(lo[1]))
print("[door] right_open y        = %.4f" % float(ro[1]))
print("[door] inner edge          = %.3f m  (must be >= %.3f to fully clear)" % (inner, width / 2.0))
print("[door] DOOR_OPEN_CLEARANCE = %.3f m  (expected %.3f)" % (clearance, expected))
print("[door] production criterion |open_y| >= %.3f : %s" % (required_offset, clears))
print("[door] initial_open=%s dynamic=%s entrance_apron_in_sdf=%s"
      % (door.get("initial_open"), door.get("dynamic"), apron))

ok = clears and abs(clearance - expected) <= 1e-6
print("[door] RESULT=%s" % ("PASS" if ok else "FAIL"))
sys.exit(0 if ok else 1)
PY

set +e
python3 "$RUN_DIR/check_door.py" "$ROOT" "$FROZEN_CLEARANCE" 2>&1 | tee "$RUN_DIR/door_check.txt"
DOOR_RC="${PIPESTATUS[0]}"
set -e
[ "$DOOR_RC" -eq 0 ] || fail_gate DOOR_GATE "main entrance clearance != ${FROZEN_CLEARANCE} m in the generated scene (see $RUN_DIR/door_check.txt)"
pass_gate DOOR_GATE "main_entrance open clearance = ${FROZEN_CLEARANCE} m in this run's generated scene"

# -----------------------------------------------------------------------------
# 8. SIM_READY_GATE — /use_sim_time, advancing /clock, spawn, RL_HANDOFF_READY
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/probe_ready.py" <<'PY'
import os, sys, time
import rospy

clock_to, handoff_to, spawn_to = float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3])
log_path, run_epoch = sys.argv[4], float(sys.argv[5])

rospy.init_node("closure_ready_probe", anonymous=True, disable_signals=True)

# --- /use_sim_time --------------------------------------------------------
deadline = time.time() + clock_to
ok = False
while time.time() < deadline:
    try:
        if bool(rospy.get_param("/use_sim_time", False)):
            ok = True
            break
    except Exception:
        pass
    time.sleep(1.0)
print("[ready] /use_sim_time=%s" % ok, flush=True)
if not ok:
    print("[ready] RESULT=FAIL use_sim_time", flush=True)
    sys.exit(2)

# --- robot spawn ----------------------------------------------------------
from gazebo_msgs.srv import GetModelState
spawned = False
deadline = time.time() + spawn_to
while time.time() < deadline:
    try:
        rospy.wait_for_service("/gazebo/get_model_state", timeout=5.0)
        get = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        r = get(model_name="a1_gazebo", relative_entity_name="world")
        if r.success:
            spawned = True
            print("[ready] robot a1_gazebo spawned", flush=True)
            break
    except Exception:
        time.sleep(2.0)
if not spawned:
    print("[ready] RESULT=FAIL robot_not_spawned", flush=True)
    sys.exit(3)

# --- /clock advancing -----------------------------------------------------
from rosgraph_msgs.msg import Clock
samples = []
sub = rospy.Subscriber("/clock", Clock, lambda m: samples.append(m.clock.to_sec()))
deadline = time.time() + clock_to
advanced = False
while time.time() < deadline:
    time.sleep(1.0)
    if len(samples) >= 2 and samples[-1] > samples[0]:
        advanced = True
        break
sub.unregister()
if not advanced:
    print("[ready] RESULT=FAIL clock_not_advancing (n=%d)" % len(samples), flush=True)
    sys.exit(4)
print("[ready] /clock advancing: %.3f -> %.3f" % (samples[0], samples[-1]), flush=True)

# --- RL_HANDOFF_READY from THIS run's log only ----------------------------
deadline = time.time() + handoff_to
while time.time() < deadline:
    try:
        if os.path.exists(log_path) and os.stat(log_path).st_mtime >= run_epoch - 1:
            with open(log_path, "r", errors="ignore") as fh:
                if "RL_HANDOFF_READY=True" in fh.read():
                    print("[ready] RL_HANDOFF_READY=True observed in a FRESH log", flush=True)
                    print("[ready] RESULT=PASS", flush=True)
                    sys.exit(0)
    except OSError:
        pass
    time.sleep(2.0)
print("[ready] RESULT=FAIL RL_HANDOFF_READY never appeared in a fresh log", flush=True)
sys.exit(5)
PY

info "waiting for sim readiness (use_sim_time + spawn + advancing /clock + RL_HANDOFF_READY)"
set +e
python3 "$RUN_DIR/probe_ready.py" 300 600 600 \
    "$ROOT/logs/junior_ctrl.log" "$RUN_EPOCH" 2>&1 | tee "$RUN_DIR/readiness.log"
READY_RC="${PIPESTATUS[0]}"
set -e
[ "$READY_RC" -eq 0 ] || fail_gate SIM_READY_GATE "sim never became ready (rc=$READY_RC); see $RUN_DIR/readiness.log"
pass_gate SIM_READY_GATE "use_sim_time + spawn + advancing /clock + RL_HANDOFF_READY=True (fresh log)"

# -----------------------------------------------------------------------------
# 9. RGBD_GATE — RGB + depth publishers exist and messages advance
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/probe_cameras.py" <<'PY'
import sys
import rospy
from sensor_msgs.msg import Image, CameraInfo

seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 12.0
rospy.init_node("closure_camera_gate", anonymous=True, disable_signals=True)
got = {}


def mk(key):
    def cb(m):
        got.setdefault(key, []).append(m.header.stamp.to_sec())
    return cb


rospy.Subscriber("/real_sense/rgb/image_raw", Image, mk("rgb"))
rospy.Subscriber("/real_sense/depth/image_raw", Image, mk("depth"))
rospy.Subscriber("/real_sense/rgb/camera_info", CameraInfo, mk("rgb_ci"))
rospy.Subscriber("/real_sense/depth/camera_info", CameraInfo, mk("depth_ci"))
rospy.sleep(seconds)

advancing = {}
for key in ("rgb", "depth", "rgb_ci", "depth_ci"):
    s = got.get(key, [])
    advancing[key] = len(s) >= 3 and s[-1] > s[0]
    print("  %-9s msgs=%-6d advancing=%s" % (key, len(s), advancing[key]), flush=True)

ok = advancing["rgb"] and advancing["depth"]
print("CAMERA_GATE=%s" % ("PASS" if ok else "FAIL"), flush=True)
sys.exit(0 if ok else 3)
PY

info "RGB-D gate (publishers must exist and stamps must advance)"
set +e
python3 "$RUN_DIR/probe_cameras.py" 12 2>&1 | tee "$RUN_DIR/camera_check.txt"
CAM_RC="${PIPESTATUS[0]}"
set -e
[ "$CAM_RC" -eq 0 ] || fail_gate RGBD_GATE "RGB-D gate failed (rc=$CAM_RC); see $RUN_DIR/camera_check.txt"
pass_gate RGBD_GATE "/real_sense rgb+depth publishers live and advancing"

# -----------------------------------------------------------------------------
# 10. GUI_GATE — gzclient actually rendering (frozen run is GUI=true)
# -----------------------------------------------------------------------------
if [ "$GUI" = "true" ]; then
    if ! proc_alive gzclient; then
        fail_gate GUI_GATE "GUI=true but gzclient is not running (DISPLAY=$DISPLAY) — the frozen run is rendered and observable"
    fi
    if ! proc_alive gzserver; then
        fail_gate GUI_GATE "gzserver is not running while GUI=true"
    fi
    pass_gate GUI_GATE "gzserver + gzclient alive on DISPLAY=$DISPLAY"
fi

# -----------------------------------------------------------------------------
# 11. Instrumentation: pose sampler (diagnostics, production odom only) + bag
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/pose_sampler.py" <<'PY'
"""Validation-only trajectory sampler.

Subscribes to /competition/localization/odom (the PRODUCTION localization
output — never a ground-truth channel) so the post-run evaluator can derive
room-zone coverage and entrance crossings.
"""
import sys, time
import rospy
from nav_msgs.msg import Odometry

out_path = sys.argv[1]
interval = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
rospy.init_node("closure_pose_sampler", anonymous=True, disable_signals=True)
last = {"v": None}


def cb(msg):
    last["v"] = (msg.header.stamp.to_sec(),
                 msg.pose.pose.position.x,
                 msg.pose.pose.position.y)


rospy.Subscriber("/competition/localization/odom", Odometry, cb)
with open(out_path, "w") as fh:
    fh.write("wall\tsim\tx\ty\n")
    while not rospy.is_shutdown():
        v = last["v"]
        if v is not None:
            fh.write("%.3f\t%.3f\t%.4f\t%.4f\n" % (time.time(), v[0], v[1], v[2]))
            fh.flush()
        rospy.sleep(interval)
PY

python3 -u "$RUN_DIR/pose_sampler.py" "$RUN_DIR/watcher.tsv" 1.0 \
    > "$RUN_DIR/sampler.out" 2>&1 &
SAMPLER_PID=$!

if [ "$RECORD_BAG" = "1" ]; then
    info "starting rosbag record (mission topics)"
    rosbag record -O "$RUN_DIR/run.bag" \
        /competition/state \
        /competition/localization/odom \
        /competition/localization/status \
        /competition/map \
        /competition/path \
        /competition/planning/status \
        /competition/control/path_follower_status \
        /competition/control/status \
        /competition/raw_cmd_vel \
        /cmd_vel \
        /competition/perception/status \
        /competition/hazard_candidates \
        > "$RUN_DIR/bag.out" 2>&1 &
    BAG_PID=$!
    echo "$BAG_PID" > "$RUN_DIR/bag.pid"
    sleep 3
fi

# -----------------------------------------------------------------------------
# 12. Launch the frozen competition stack
# -----------------------------------------------------------------------------
SNAPSHOT_BEFORE="$RUN_DIR/roslog_before.txt"
mkdir -p "$ROS_HOME/log"
ls -1d "$ROS_HOME"/log/*/ > "$SNAPSHOT_BEFORE" 2>/dev/null || true

info "launching competition stack (max_time request = $FROZEN_MAX_TIME s)"
roslaunch "$LAUNCH" max_time:="$FROZEN_MAX_TIME" \
    > "$RUN_DIR/competition_stack.log" 2>&1 &
STACK_PID=$!
echo "$STACK_PID" > "$RUN_DIR/stack.pid"

# -----------------------------------------------------------------------------
# 13. MAX_TIME_GATE + GT_COMPLIANCE_GATE — read back from the LIVE system
# -----------------------------------------------------------------------------
cat > "$RUN_DIR/probe_live.py" <<'PY'
"""Live contract audit: the effective max_time and the GT non-feedback rule.

Read-only.  Never publishes.  Uses `rostopic info` for the subscriber/publisher
audit, exactly like the authoritative run's gt_compliance.txt.
"""
import subprocess, sys, time
import rospy

expected_max_time = float(sys.argv[1])
timeout = float(sys.argv[2]) if len(sys.argv) > 2 else 180.0

rospy.init_node("closure_live_audit", anonymous=True, disable_signals=True)

# --- 1. effective /competition_loop/max_time -----------------------------
value = None
deadline = time.time() + timeout
while time.time() < deadline:
    try:
        value = float(rospy.get_param("/competition_loop/max_time"))
        break
    except Exception:
        time.sleep(1.0)
if value is None:
    print("[live] MAX_TIME_GATE=FAIL (parameter never appeared)", flush=True)
    sys.exit(4)
print("[live] resolved /competition_loop/max_time = %s" % value, flush=True)
max_time_ok = abs(value - expected_max_time) <= 1e-6
print("[live] MAX_TIME_GATE=%s" % ("PASS" if max_time_ok else "FAIL"), flush=True)

# --- 2. GT / referee channel audit ---------------------------------------
FORBIDDEN_PUBLISHED = ["/Odometry_gazebo", "/ground_truth/odom", "/ground_truth/state"]
FORBIDDEN_SUBSCRIBED = FORBIDDEN_PUBLISHED + ["/gazebo/model_states", "/gazebo/link_states"]
PRODUCTION_NODES = {
    "/competition_loop", "/planning", "/mapping", "/perception", "/localization",
    "/laserMapping", "/filter_livox", "/leg_odometry", "/path_follower",
    "/cmd_vel_guard", "/pointcloud2livox",
}
# Gazebo-side plugins / validation-only probes are not production algorithms.
NON_PRODUCTION_SUBSCRIBERS = {"/unitree_gazebo_servo", "/rosout"}


def topic_parties(topic):
    try:
        out = subprocess.run(["rostopic", "info", topic], capture_output=True,
                             text=True, timeout=25).stdout
    except Exception:
        return [], []
    pubs, subs, mode = [], [], None
    for line in out.splitlines():
        s = line.strip()
        if s.startswith("Publishers:"):
            mode = "p"
        elif s.startswith("Subscribers:"):
            mode = "s"
        elif s.startswith("*"):
            name = s.lstrip("* ").split(" ")[0].strip()
            if name and name != "None":
                (pubs if mode == "p" else subs).append(name)
    return pubs, subs


print("[live] --- forbidden help topics: publishers ---", flush=True)
published = []
for t in FORBIDDEN_PUBLISHED:
    pubs, _ = topic_parties(t)
    print("[live] %-24s publishers=%s" % (t, pubs or "none"), flush=True)
    if pubs:
        published.append((t, pubs))

print("[live] --- production nodes subscribing to a truth channel ---", flush=True)
bad_subs = []
for t in FORBIDDEN_SUBSCRIBED:
    _, subs = topic_parties(t)
    prod = sorted(set(subs) & PRODUCTION_NODES)
    other = sorted(set(subs) - PRODUCTION_NODES - NON_PRODUCTION_SUBSCRIBERS)
    print("[live] %-24s subscribers=%s production=%s other=%s"
          % (t, subs or "none", prod or "none", other or "none"), flush=True)
    if prod:
        bad_subs.append((t, prod))

gt_ok = not published and not bad_subs
print("[live] GT_COMPLIANCE_GATE=%s" % ("PASS" if gt_ok else "FAIL"), flush=True)
sys.exit(0 if (max_time_ok and gt_ok) else 5)
PY

info "auditing live contracts (max_time resolution + GT non-feedback)"
set +e
python3 "$RUN_DIR/probe_live.py" "$FROZEN_MAX_TIME" 180 2>&1 | tee "$RUN_DIR/live_audit.txt"
LIVE_RC="${PIPESTATUS[0]}"
set -e
LIVE_TXT="$(cat "$RUN_DIR/live_audit.txt")"
case "$LIVE_TXT" in
    *"MAX_TIME_GATE=PASS"*) pass_gate MAX_TIME_GATE "/competition_loop/max_time resolved to $FROZEN_MAX_TIME (read back live, not merely requested)" ;;
    *) fail_gate MAX_TIME_GATE "/competition_loop/max_time did not resolve to $FROZEN_MAX_TIME (see $RUN_DIR/live_audit.txt)" ;;
esac
case "$LIVE_TXT" in
    *"GT_COMPLIANCE_GATE=PASS"*) pass_gate GT_COMPLIANCE_GATE "no forbidden truth publishers; no production node subscribes to a truth channel" ;;
    *) fail_gate GT_COMPLIANCE_GATE "GT/referee channel violation (see $RUN_DIR/live_audit.txt)" ;;
esac

# roslaunch per-node logs for THIS run only (mission events land there).
NEWEST_LOG="$(ls -1dt "$ROS_HOME"/log/*/ 2>/dev/null | head -1 || true)"
echo "ROS_LOG_DIR=$NEWEST_LOG" > "$RUN_DIR/ros_log_dir.txt"
info "per-node roslog dir: ${NEWEST_LOG:-<none>}"

# -----------------------------------------------------------------------------
# 14. Wait for the mission to finish autonomously
# -----------------------------------------------------------------------------
info "audits captured; mission runs autonomously (wall watchdog ${RUN_WALL_TIMEOUT}s)"
DEADLINE=$(( $(date +%s) + RUN_WALL_TIMEOUT ))
while kill -0 "$STACK_PID" 2>/dev/null; do
    if [ "$(date +%s)" -ge "$DEADLINE" ]; then
        info "wall watchdog fired after ${RUN_WALL_TIMEOUT}s — aborting"
        kill -TERM "$STACK_PID" 2>/dev/null || true
        sleep 5
        kill -KILL "$STACK_PID" 2>/dev/null || true
        if [ "$RECORD_BAG" = "1" ] && [ -n "${BAG_PID:-}" ]; then kill -INT "$BAG_PID" 2>/dev/null || true; fi
        if [ -n "${SAMPLER_PID:-}" ]; then kill -TERM "$SAMPLER_PID" 2>/dev/null || true; fi
        sleep 3
        fail_gate CLOSURE_GATE "mission did not finish within ${RUN_WALL_TIMEOUT}s wall clock"
    fi
    sleep 10
done
set +e
wait "$STACK_PID"
STACK_RC=$?
set -e
info "competition stack exited (rc=$STACK_RC)"

# -----------------------------------------------------------------------------
# 15. Post-run: snapshot evidence, evaluate closure
# -----------------------------------------------------------------------------
if [ "$RECORD_BAG" = "1" ] && [ -n "${BAG_PID:-}" ]; then
    kill -INT "$BAG_PID" 2>/dev/null || true
    sleep 5
fi
if [ -n "${SAMPLER_PID:-}" ]; then
    kill -TERM "$SAMPLER_PID" 2>/dev/null || true
    sleep 2
fi

mkdir -p "$RUN_DIR/scene"
for f in generated_building/scene_manifest.json generated_building/building_config.json \
         generated_building/door_config.yaml generated_building/generation_checks.json \
         results/danger_truth.json results/detected_danger.json; do
    [ -f "$ROOT/$f" ] && cp -f "$ROOT/$f" "$RUN_DIR/scene/" 2>/dev/null || true
done

cat > "$RUN_DIR/evaluate_closure.py" <<'PY'
"""Post-run closure evaluation.

GROUND TRUTH IS READ HERE ONLY (results/danger_truth.json), after the mission,
for scoring.  No production node ever sees it.
"""
import json, math, os, re, sys
from pathlib import Path

run_dir = Path(sys.argv[1])
repo = Path(sys.argv[2])
ros_log_dir = Path(sys.argv[3]) if len(sys.argv) > 3 and sys.argv[3] else None

STACK_LOG = run_dir / "competition_stack.log"
WATCHER = run_dir / "watcher.tsv"
ZONES = [(1.1, 9.5, 7.9, 21.9), (-9.5, -1.1, 7.9, 21.9),
         (1.1, 9.5, 21.9, 35.9), (-9.5, -1.1, 21.9, 35.9)]
MATCH_RADIUS = 1.0
CROSSING_HALF_WIDTH = 1.15  # main entrance opening half width (jamb 1.0 + margin)


def read(p):
    try:
        return Path(p).read_text(errors="ignore")
    except (OSError, TypeError):
        return ""


node_logs = []
if ros_log_dir and ros_log_dir.is_dir():
    node_logs = sorted(ros_log_dir.glob("competition_loop-*.log")) + \
                sorted(ros_log_dir.glob("planning-*.log"))
nodelog = "\n".join(read(p) for p in node_logs)
console = read(STACK_LOG)
log = nodelog if nodelog.strip() else console
out = {}

out["RETURNING_ENTERED"] = bool(re.search(r"Mission event: RETURNING to start", log))
m = re.search(r"Mission event: ARRIVED_START \(distance=([\d.]+) m, samples=(\d+)\)", log)
out["ARRIVED_START"] = bool(m)
out["ARRIVED_START_DISTANCE_M"] = float(m.group(1)) if m else None
m = re.search(r"Competition FINISHED\. elapsed=([\d.]+) s", log)
out["FINISHED"] = bool(m)
out["TOTAL_SIM_TIME"] = float(m.group(1)) if m else None
out["CURRENT_FLOOR_COMPLETE"] = "CURRENT_FLOOR_COMPLETE" in log or \
    "CURRENT_FLOOR_COMPLETE" in read(WATCHER)
out["EXPLORATION_COMPLETE_LOGGED"] = bool(
    re.search(r"exploration complete after \d+ consecutive", log))
out["MAX_TIME_REACHED"] = "max_time reached; transitioning to FINISHED" in log
out["RETURNING_GOAL_NOT_REACHABLE_COUNT"] = len(
    re.findall(r"RETURNING goal is not reachable", log))
out["RETURN_PATH_COMPUTATIONS"] = len(
    re.findall(r"RETURNING path with \d+ poses to start", log))
out["VISUAL_SCAN_COMPLETE_COUNT"] = len(
    re.findall(r"visual scan complete \(\d+ headings\)", log))

# --- hazards: score the written file against referee truth -----------------
truth = json.loads((repo / "results" / "danger_truth.json").read_text())
sources = [(s["model_name"], tuple(s["position"])) for s in truth["danger_sources"]]
detected = []
try:
    dd = json.loads((repo / "results" / "detected_danger.json").read_text())
    detected = [tuple(s["position"]) for s in dd.get("detected_danger_sources", [])]
except Exception as exc:
    out["DETECTED_ERROR"] = str(exc)
matched, false_alarms = set(), 0
for p in detected:
    best, bd = None, 1e9
    for name, sp in sources:
        d = math.dist(tuple(p), tuple(sp))
        if d < bd:
            bd, best = d, name
    if bd <= MATCH_RADIUS:
        matched.add(best)
    else:
        false_alarms += 1
out["TRUE_DANGER_COUNT"] = len(sources)
out["DETECTED_DANGER_JSON_COUNT"] = len(detected)
out["HAZARD_UNIQUE_DETECTED_COUNT"] = len(matched)
out["FALSE_ALARMS_WRITTEN"] = false_alarms

# --- entrance crossings + zone coverage from the production odom samples ---
traj = []
for line in read(WATCHER).splitlines()[1:]:
    f = line.split("\t")
    if len(f) < 4:
        continue
    try:
        x, y = float(f[2]), float(f[3])
    except ValueError:
        continue
    traj.append((x, y))
cross_in = cross_out = False
for (x0, y0), (x1, y1) in zip(traj, traj[1:]):
    if abs(x0) <= CROSSING_HALF_WIDTH and y0 < 0.0 <= y1:
        cross_in = True
    if abs(x0) <= CROSSING_HALF_WIDTH and y0 > 0.0 >= y1:
        cross_out = True
out["ENTRANCE_CROSSING_OUT_TO_IN"] = cross_in
out["ENTRANCE_CROSSING_IN_TO_OUT"] = cross_out
out["TRAJECTORY_SAMPLES"] = len(traj)
zones_hit = set()
for (x, y) in traj:
    for i, (zx0, zx1, zy0, zy1) in enumerate(ZONES):
        if zx0 <= x <= zx1 and zy0 <= y <= zy1:
            zones_hit.add(i)
out["ROOM_SCAN_ZONES_TRIGGERED"] = "%d/4" % len(zones_hit)

REQUIRED = {
    "RETURNING_ENTERED": True,
    "ARRIVED_START": True,
    "FINISHED": True,
    "CURRENT_FLOOR_COMPLETE": True,
    "EXPLORATION_COMPLETE_LOGGED": True,
    "MAX_TIME_REACHED": False,
    "ENTRANCE_CROSSING_OUT_TO_IN": True,
    "ENTRANCE_CROSSING_IN_TO_OUT": True,
}
REQUIRED_COUNTS = {
    "HAZARD_UNIQUE_DETECTED_COUNT": ("==", "TRUE_DANGER_COUNT"),
    "FALSE_ALARMS_WRITTEN": ("==", 0),
    "RETURNING_GOAL_NOT_REACHABLE_COUNT": ("==", 0),
}
failed = []
for k, want in REQUIRED.items():
    if bool(out.get(k)) is not bool(want):
        failed.append("%s=%s(want %s)" % (k, out.get(k), want))
for k, (op, other) in REQUIRED_COUNTS.items():
    want = out[other] if isinstance(other, str) else other
    if out.get(k) != want:
        failed.append("%s=%s(want %s)" % (k, out.get(k), want))

print("=" * 72)
print("FLOOR0 DOORFIX CLOSURE — POST-RUN EVALUATION")
print("=" * 72)
for k in sorted(out, key=lambda s: (s not in REQUIRED and s not in REQUIRED_COUNTS, s)):
    tag = "REQUIRED" if (k in REQUIRED or k in REQUIRED_COUNTS) else "diagnostic"
    print("%-36s = %-12s [%s]" % (k, out[k], tag))
print()
if failed:
    print("CLOSURE=FAIL  failing: %s" % "; ".join(failed))
    sys.exit(1)
print("CLOSURE=PASS  ENTRY -> exploration -> 6/6 hazards -> CURRENT_FLOOR_COMPLETE"
      " -> RETURNING -> entrance crossing -> ARRIVED_START -> FINISHED")
sys.exit(0)
PY

set +e
python3 "$RUN_DIR/evaluate_closure.py" "$RUN_DIR" "$ROOT" "$NEWEST_LOG" 2>&1 \
    | tee "$RUN_DIR/closure_evaluation.txt"
CLOSURE_RC="${PIPESTATUS[0]}"
set -e
[ "$CLOSURE_RC" -eq 0 ] || fail_gate CLOSURE_GATE "closure sequence not achieved (see $RUN_DIR/closure_evaluation.txt)"
pass_gate CLOSURE_GATE "ENTRY -> 6/6 -> CURRENT_FLOOR_COMPLETE -> RETURNING -> crossing -> ARRIVED_START -> FINISHED"

report_and_exit 0
