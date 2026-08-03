# competition_mapping

This package builds a fixed-size 2D `nav_msgs/OccupancyGrid` from legitimate
localization odometry and LiDAR `PointCloud2` data.

## Interface contract

- Input: `/competition/localization/status` (`competition_interfaces/ModuleStatus`)
- Input: `/competition/localization/odom` (`nav_msgs/Odometry`)
- Input: `/livox/Pointcloud2` (`sensor_msgs/PointCloud2`)
- Output: `/competition/map` (`nav_msgs/OccupancyGrid`)
- Output: `/competition/mapping/status` (`competition_interfaces/ModuleStatus`)

The node never reads Gazebo ground truth, world/layout/truth files, or publishes
`/cmd_vel`. Point clouds are fused only while localization status and odometry
are valid and fresh.

## Files

- `src/competition_mapping/occupancy_grid_core.py`: ROS-independent grid logic.
- `scripts/mapping_node`: subscriptions, TF conversion, health checks, map/status publication.
- `config/mapping.yaml`: all map, filter, timeout, and pose-jump parameters.
- `launch/mapping.launch`: production launch; it starts no mock nodes.
- `test/test_occupancy_grid_core.py`: deterministic pure-Python unit tests.
- `test/mapping_integration.test`: ROS integration and localization-failure test.

## Replace the existing package

From the SimEnv repository on the host, first back up or commit your current
work. Then replace the contents of `src/competition_mapping/` with the contents
of this directory. Preserve the real maintainer name/email in `package.xml` if
your repository already has them.

Make scripts executable:

```bash
chmod +x \
  src/competition_mapping/scripts/mapping_node \
  src/competition_mapping/test/test_mapping_integration.py \
  src/competition_mapping/test/test_occupancy_grid_core.py
```

## Test 1: no-ROS unit test

Run on the host or in the container:

```bash
cd "$HOME/SimEnv"
PYTHONPATH=src/competition_mapping/src \
python3 -m unittest -v \
  src/competition_mapping/test/test_occupancy_grid_core.py
```

Expected result: `Ran 12 tests` and `OK`.

## Test 2: build and compile checks

Run inside the `simenv-my` container (or use your actual personal service name):

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash

python3 -m py_compile \
  src/competition_mapping/src/competition_mapping/*.py \
  src/competition_mapping/scripts/mapping_node \
  src/competition_mapping/test/*.py

catkin_make --pkg competition_mapping
source devel/setup.bash
rospack find competition_mapping
rosrun competition_mapping mapping_node
```

The final `rosrun` command starts the node, so a ROS master must already be
running. Stop it with `Ctrl+C` after confirming the executable is found and the
node reports that it is waiting for inputs.

## Test 3: automated ROS integration test

With no other mapping node or mock publisher running:

```bash
cd /workspace/SimEnv
source /opt/ros/noetic/setup.bash
source devel/setup.bash

rostest competition_mapping mapping_integration.test
```

The test publishes synthetic legal localization, odometry, and base-frame point
clouds. It verifies map metadata, `-1/0/100` encoding, `status.valid=true`, then
injects `localization.valid=false` and verifies that the map no longer changes.

## Test 4: manual synthetic inspection

Terminal 1:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash
roslaunch competition_mapping mapping.launch \
  pointcloud_topic:=/test/points
```

The automated `rostest` is the preferred synthetic test. For a manual test,
run the publishers from `test/test_mapping_integration.py` via rostest, or use
your team's `competition_testing` mock only in `test_mode=true`.

Inspect interfaces in another terminal:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash
rostopic info /competition/map
rostopic hz /competition/map
rostopic echo -n 1 /competition/map/info
rostopic echo -n 1 /competition/mapping/status
```

Expected map values are `unknown=-1`, `free=0`, and `occupied=100`; the default
metadata is 0.1 m resolution, 400 x 400 cells, origin (-20, -20), frame `odom`.

## Test 5: real simulation

Reserve the shared simulator first. Start SimEnv only once, with truth disabled:

```bash
cd /workspace/SimEnv
ENABLE_GROUND_TRUTH=0 ENABLE_REFEREE_ODOM=0 \
UNITREE_AUTO_CMD_VEL=1 CONTROLLER_FOREGROUND=0 \
SEED=77 GUI=true ./auto.sh
```

In another container terminal:

```bash
source /opt/ros/noetic/setup.bash
source /workspace/SimEnv/devel/setup.bash

rostopic type /livox/Pointcloud2
rostopic hz /livox/Pointcloud2
rostopic echo -n 1 /livox/Pointcloud2/header
rostopic echo -n 1 /competition/localization/odom/header
rostopic echo -n 1 /competition/localization/odom/child_frame_id
rosrun tf tf_echo base_link <POINTCLOUD_FRAME>

roslaunch competition_mapping mapping.launch
```

Replace `base_link` with the actual odometry `child_frame_id`, and
`<POINTCLOUD_FRAME>` with the point-cloud header frame. If `tf_echo` cannot find
that transform, the mapper will correctly report `BLOCKED`; the sensor-to-base
static TF must be fixed in bringup rather than bypassed in mapping.

In RViz:

1. Set Fixed Frame to `odom`.
2. Add `Map`, topic `/competition/map`.
3. Add `PointCloud2`, topic `/livox/Pointcloud2`.
4. Confirm walls are continuous, robot surroundings are free, unknown space
   remains unknown, and the whole map does not rotate or drift with the robot.

## Failure injection

Do this only in an isolated test session where a mock localization status
publisher is allowed. Record one map, publish invalid status, keep sending the
cloud, and compare the map again. The provided rostest automates this check.

For pose-jump protection, publish an odometry jump larger than `max_pose_jump`
or `max_yaw_jump`. Mapping status becomes `BLOCKED`. After
`pose_recovery_samples` mutually consistent poses, the default behavior resets
the old map and resumes, preventing two coordinate frames from being fused.

## Compliance check

```bash
rg -n \
  '/Odometry_gazebo|/ground_truth/|get_model_state|danger_truth.json|layout_metadata.json|competition_scene.world|scene_manifest.json|/cmd_vel' \
  src/competition_mapping
```

Only explanatory README text should match; executable Python and launch/config
files must contain no forbidden interface or truth-file access.
