#!/usr/bin/env bash
# =============================================================================
# VALIDATION HARNESS — one clean, fully observable Floor0 run at SEED=77,
# GUI=true, on commit 3503f44e617385c56f264875575d8ca8b68c3b36.
#
# Executed INSIDE the container.  Changes NO production algorithm.
#
# Order:
#   clean slate -> sim up (GUI=true) -> RL handoff ready -> CAMERA GATE (RGB-D)
#   -> instrumentation (rosbag + watcher + GUI screenshots) -> competition stack
#   -> live contract audits (max_time / GT non-feedback) -> wait for FSM end
# =============================================================================
# NOT `set -u`: ROS/catkin profile scripts are not unbound-variable safe.
set -o pipefail

RUN_DIR="/workspace/SimEnv/artifacts/floor0-video-return/20260914-153840"

source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
source ./devel/setup.bash

export ROS_MASTER_URI="http://127.0.0.1:11511"
export GAZEBO_MASTER_URI="http://127.0.0.1:11545"
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/tmp/.Xauthority}"

log() { echo "[runner $(date +%H:%M:%S)] $*"; }

# --- 1. clean slate ----------------------------------------------------------
log "stopping any previous stack/sim"
pkill -f "floor0_full_replay.launch" 2>/dev/null || true
pkill -f "multi_floor_gazeboSim.launch" 2>/dev/null || true
pkill -f "building_generator_classic_control" 2>/dev/null || true
pkill -f "junior_ctrl" 2>/dev/null || true
pkill -f "rosbag record" 2>/dev/null || true
pkill -f "watcher.py" 2>/dev/null || true
pkill -f "gzserver|gzclient|gazebo" 2>/dev/null || true
pkill -f "roscore|rosmaster" 2>/dev/null || true
sleep 5

# --- 2. sim up ---------------------------------------------------------------
# Stale logs are removed FIRST: auto.sh truncates junior_ctrl.log only once
# junior_ctrl actually starts, so a leftover log would satisfy the readiness
# grep immediately.
log "removing stale logs so readiness cannot be judged from an old run"
rm -f /workspace/SimEnv/logs/junior_ctrl.log \
      /workspace/SimEnv/logs/competition_gazebo.log \
      /workspace/SimEnv/logs/building_control.log

RUN_EPOCH="$(date +%s)"
echo "RUN_START_WALL=$(date -Iseconds)" > "$RUN_DIR/run_timeline.txt"
echo "RUN_EPOCH=$RUN_EPOCH" >> "$RUN_DIR/run_timeline.txt"
log "starting sim (auto.sh, GUI=true)  RUN_EPOCH=$RUN_EPOCH"
bash "$RUN_DIR/start_sim.sh" > "$RUN_DIR/auto_stdout.log" 2>&1 &
echo $! > "$RUN_DIR/auto.pid"

log "waiting for RL_HANDOFF_READY (fresh log only)"
READY=0
for i in $(seq 1 300); do
  JL=/workspace/SimEnv/logs/junior_ctrl.log
  if [ -f "$JL" ] && [ "$(stat -c %Y "$JL" 2>/dev/null || echo 0)" -ge "$RUN_EPOCH" ] \
     && grep -aq "RL_HANDOFF_READY=True" "$JL"; then
    log "RL_HANDOFF_READY observed after ${i}s"; READY=1; break
  fi
  sleep 1
done
[ "$READY" -eq 1 ] || { log "FATAL: RL_HANDOFF_READY never appeared in a FRESH log"; exit 1; }

# --- 3. CAMERA GATE (RGB-D must be live) -------------------------------------
log "camera gate (RGB + depth + CameraInfo)"
python3 - <<'PY'
import sys, rospy
from sensor_msgs.msg import Image, CameraInfo
rospy.init_node("cam_gate", anonymous=True, disable_signals=True)
got = {}
def mk(t):
    def cb(m): got.setdefault(t, []).append(m.header.stamp.to_sec())
    return cb
rospy.Subscriber("/real_sense/rgb/image_raw", Image, mk("rgb"))
rospy.Subscriber("/real_sense/depth/image_raw", Image, mk("depth"))
rospy.Subscriber("/real_sense/rgb/camera_info", CameraInfo, mk("rgb_ci"))
rospy.Subscriber("/real_sense/depth/camera_info", CameraInfo, mk("depth_ci"))
rospy.sleep(12.0)
ok = True
for k in ("rgb", "depth", "rgb_ci", "depth_ci"):
    s = got.get(k, [])
    adv = len(s) >= 3 and s[-1] > s[0]
    print("  %-9s msgs=%-5d advancing=%s" % (k, len(s), adv), flush=True)
    ok = ok and adv
print("CAMERA_GATE=%s" % ("PASS" if ok else "FAIL"), flush=True)
sys.exit(0 if ok else 3)
PY
CAM_RC=$?
[ "$CAM_RC" -eq 0 ] || { log "FATAL: camera gate failed (rc=$CAM_RC)"; exit 1; }
log "camera gate PASS"

# --- 4. instrumentation ------------------------------------------------------
log "starting rosbag + watcher + GUI screenshot loop"
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
echo $! > "$RUN_DIR/bag.pid"

python3 -u "$RUN_DIR/watcher.py" "$RUN_DIR" 10 > "$RUN_DIR/watcher.out" 2>&1 &
echo $! > "$RUN_DIR/watcher.pid"

# GUI evidence: real full-screen captures of display :1 (gzclient window).
sleep 3

# --- 5. competition stack (mission auto-starts) ------------------------------
log "launching competition stack (mission auto_start=true, GUI=true)"
roslaunch "$RUN_DIR/floor0_full_replay.launch" max_time:=1800.0 \
  > "$RUN_DIR/competition_stack.log" 2>&1 &
STACK_PID=$!
echo $STACK_PID > "$RUN_DIR/stack.pid"

# --- 6. live contract audits -------------------------------------------------
log "waiting for /competition_loop/max_time to be set by the stack"
for i in $(seq 1 120); do
  if rosparam get /competition_loop/max_time >/dev/null 2>&1; then break; fi
  sleep 1
done

{
  echo "### EFFECTIVE ROS PARAMS (captured live, $(date -Iseconds))"
  echo "/competition_loop/max_time = $(rosparam get /competition_loop/max_time 2>&1)"
  echo "/use_sim_time = $(rosparam get /use_sim_time 2>&1)"
  echo "/competition_loop/return_arrival_tolerance = $(rosparam get /competition_loop/return_arrival_tolerance 2>&1)"
  echo "/planning/inflation_radius = $(rosparam get /planning/inflation_radius 2>&1)"
  echo "/planning/room_mission_enabled = $(rosparam get /planning/room_mission_enabled 2>&1)"
  echo "/planning/visual_scan_route_spacing = $(rosparam get /planning/visual_scan_route_spacing 2>&1)"
  echo
  echo "### room_scan_zones"
  echo "$(rosparam get /planning/room_scan_zones 2>&1)"
} > "$RUN_DIR/params.txt" 2>&1

{
  echo "### GT / referee-channel audit (captured live, $(date -Iseconds))"
  echo "### 1. forbidden help topics present?"
  for t in /Odometry_gazebo /ground_truth/odom /ground_truth/state /gazebo/model_states /gazebo/link_states; do
    n=$(rostopic info "$t" 2>/dev/null | sed -n 's/^Publishers: *//p' | wc -c)
    printf "%-28s publishers_present=%s\n" "$t" "$(rostopic info "$t" >/dev/null 2>&1 && echo yes || echo no)"
  done
  echo
  echo "### 2. does any PRODUCTION node subscribe to a truth channel?"
  for t in /gazebo/model_states /gazebo/link_states /ground_truth/odom /ground_truth/state; do
    if rostopic info "$t" >/dev/null 2>&1; then
      echo "--- $t"
      rostopic info "$t" 2>/dev/null | sed -n '/^Subscribers:/,$p'
    fi
  done
  echo
  echo "### 3. any topic whose name contains ground_truth / Odometry_gazebo?"
  rostopic list 2>/dev/null | grep -E "ground_truth|Odometry_gazebo" || echo "(none)"
  echo
  echo "### 4. live node list"
  rosnode list 2>/dev/null | sort
} > "$RUN_DIR/gt_compliance.txt" 2>&1

# roslaunch per-node logs for THIS run only (see log_source_note in the
# doorfix-causal run: console capture misses output="log" nodes).
log "newest roslog dir under \$ROS_HOME/log:"
ls -1dt /workspace/SimEnv/artifacts/ros/log/*/ 2>/dev/null | head -1 | tee "$RUN_DIR/ros_log_dir.txt"
sed -i 's|^|ROS_LOG_DIR=|' "$RUN_DIR/ros_log_dir.txt"

log "audits captured; waiting for competition stack to exit (mission runs autonomously)"
# Validation wall watchdog: bound the wait so a permanent deadlock is reported
# instead of consuming the whole validation budget.
WALL_LIMIT="${WALL_LIMIT:-9000}"
WAITED=0
while kill -0 "$STACK_PID" 2>/dev/null; do
  if [ "$WAITED" -ge "$WALL_LIMIT" ]; then
    log "FATAL: wall watchdog fired after ${WALL_LIMIT}s; stopping the stack"
    pkill -f "floor0_full_replay.launch" 2>/dev/null || true
    kill -TERM "$STACK_PID" 2>/dev/null || true
    sleep 5
    kill -KILL "$STACK_PID" 2>/dev/null || true
    break
  fi
  sleep 10
  WAITED=$((WAITED + 10))
done
wait "$STACK_PID"
log "competition stack exited"
