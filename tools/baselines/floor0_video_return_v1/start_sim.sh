#!/usr/bin/env bash
# =============================================================================
# VALIDATION HARNESS — starts Gazebo (GUI=true) + robot + junior_ctrl.
# Runs INSIDE the container.  Changes NO production algorithm.
#
# Frozen conditions:
#   SEED=77  FLOOR_COUNT=1  ROOMS_PER_FLOOR=4
#   ENABLE_GROUND_TRUTH=0  ENABLE_REFEREE_ODOM=0  WRITE_GENERATED_TRUTH_COPY=0
#   UNITREE_AUTO_CMD_VEL=1  CONTROLLER_FOREGROUND=0
#   GUI=true                <-- requested by the task: Gazebo client window is
#                               actually created on the real X display :1, so
#                               the run is observable (xwd screenshots).
#   ENABLE_REALSENSE=1      <-- RGB-D must be live
# =============================================================================
# NOTE: deliberately NOT `set -u` — the ROS/catkin profile scripts are not
# unbound-variable safe (e.g. 10.roslaunch.sh references ROS_MASTER_URI).
set -o pipefail

source /opt/ros/noetic/setup.bash
cd /workspace/SimEnv
source ./devel/setup.bash

export ROS_MASTER_URI="http://127.0.0.1:11511"
export GAZEBO_MASTER_URI="http://127.0.0.1:11545"

# --- real X display :1 (host Xorg, socket bind-mounted) ----------------------
export DISPLAY="${DISPLAY:-:1}"
export XAUTHORITY="${XAUTHORITY:-/tmp/.Xauthority}"

# --- frozen scene conditions -------------------------------------------------
export SEED=77
export FLOOR_COUNT=1
export ROOMS_PER_FLOOR=4
export ENABLE_GROUND_TRUTH=0
export ENABLE_REFEREE_ODOM=0
export WRITE_GENERATED_TRUTH_COPY=0
export UNITREE_AUTO_CMD_VEL=1
export CONTROLLER_FOREGROUND=0
export GUI=true
export ENABLE_REALSENSE=1
export ENABLE_SENSORS=1

echo "[harness] DISPLAY=$DISPLAY XAUTHORITY=$XAUTHORITY"
echo "[harness] ROS_MASTER_URI=$ROS_MASTER_URI GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
echo "[harness] SEED=$SEED FLOOR_COUNT=$FLOOR_COUNT ROOMS_PER_FLOOR=$ROOMS_PER_FLOOR"
echo "[harness] ENABLE_GROUND_TRUTH=$ENABLE_GROUND_TRUTH ENABLE_REFEREE_ODOM=$ENABLE_REFEREE_ODOM WRITE_GENERATED_TRUTH_COPY=$WRITE_GENERATED_TRUTH_COPY"
echo "[harness] UNITREE_AUTO_CMD_VEL=$UNITREE_AUTO_CMD_VEL CONTROLLER_FOREGROUND=$CONTROLLER_FOREGROUND"
echo "[harness] GUI=$GUI ENABLE_REALSENSE=$ENABLE_REALSENSE ENABLE_SENSORS=$ENABLE_SENSORS"

exec bash ./auto.sh
