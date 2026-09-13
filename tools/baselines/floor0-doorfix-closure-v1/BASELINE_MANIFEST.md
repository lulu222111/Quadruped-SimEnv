# BASELINE MANIFEST — floor0-doorfix-closure-v1

Validation / baseline-reproduction directory. **Not production defaults.**
**No production algorithm was modified to create or run this baseline.**

This manifest is the *only* source of truth for the runner
(`run_floor0_doorfix_closure.sh`): every frozen value is parsed from the
`KEY = value` lines below. Do not duplicate those keys elsewhere in this file.

---

## 1. Frozen identity

```text
PRODUCTION_BASE_HEAD = 3503f44e617385c56f264875575d8ca8b68c3b36
FAST_LIO = bf5f05130f693fd9e32a1913d9abbb65f84a07f2
LIVOX = 24cf049379cd32f96b673d62766df4cd5bcd0cf8
TOOLING_SCOPE = tools/baselines/floor0-doorfix-closure-v1

SEED = 77
FLOOR_COUNT = 1
ROOMS_PER_FLOOR = 4
ENABLE_GROUND_TRUTH = 0
ENABLE_REFEREE_ODOM = 0
```

* `PRODUCTION_BASE_HEAD` subject: `fix(building-generator): fully clear main entrance when open`
* Branch that carries it: `origin/fix/huangym/main-entrance-open-geometry-v1`
* Submodule paths: `src/fast_lio`, `src/livox_ros_driver`
* Container image used by the authoritative run: `simenv:noetic-cu128`

### 1.1 Three distinct identities — do not conflate them

```text
PRODUCTION_BASE_HEAD        = 3503f44e617385c56f264875575d8ca8b68c3b36
                              the frozen PRODUCTION commit under test
REPRODUCTION_TOOLING_COMMIT = the descendant commit that adds this directory
                              (printed by the runner as `git rev-parse HEAD`;
                               never used as a gate — a commit cannot contain
                               its own hash)
PRODUCTION_SOURCE_CHANGED   = false
```

`HEAD_SOURCE_GATE` therefore does **not** require
`CURRENT_HEAD == 3503f44` — committing this directory makes that impossible.
It requires (see §4):

1. `3503f44` is an **ancestor** of `HEAD`;
2. every tracked file changed between `3503f44..HEAD` lives under
   `TOOLING_SCOPE`;
3. `src/**`, `auto.sh`, `setup_multi_floor_simulation.py`, `docker/**`,
   `compose.yaml`, `scripts/**` and `.gitmodules` are **zero-diff** versus
   `3503f44`.

Tracked CPython byte-code caches (`src/**/__pycache__/*.pyc`) are excluded from
the zero-diff requirement: CPython silently regenerates them on import because a
fresh checkout gives the sources a new mtime, so they differ after any run
regardless of the algorithm. They are reported as a note; the corresponding
`.py` sources are still checked byte-for-byte.

## 2. Authoritative run this baseline was recovered from

```text
AUTHORITATIVE_RUN = artifacts/floor0-full-repro/floor0-full-repro-20260913-155635
```

Everything in sections 3–5 was read back from that run's own artifacts
(`params.txt`, `run_summary.md`, `runner.log`, `auto_stdout.log`,
`gt_compliance.txt`, `scene_manifest.json`, `door_clearance.txt`,
`evaluation.txt`, `competition_stack.log`). **Nothing here was copied from the
old `tools/baselines/floor0_4room_v1` freeze.**

Result being reproduced (full, untruncated, autonomous):

```text
ENTRY -> exploration -> 6/6 hazards -> CURRENT_FLOOR_COMPLETE
      -> RETURNING -> entrance crossing -> ARRIVED_START -> FINISHED

TOTAL_SIM_TIME              = 736.8 s
MAX_TIME_REACHED            = False
HAZARD_UNIQUE_DETECTED      = 6 / 6   (FALSE_ALARMS = 0)
RETURNING_GOAL_NOT_REACHABLE= 0
MANUAL_INTERVENTION_COUNT   = 0
GROUND_TRUTH_FEEDBACK_COUNT = 0
```

## 3. Resolved runtime configuration (recovered, not guessed)

### 3.1 Launch wiring

The runner launches one validation-only wrapper,
`tools/baselines/floor0-doorfix-closure-v1/floor0_doorfix_closure.launch`,
which includes two **unmodified** production launches:

| Include | Args actually used |
|---|---|
| `competition_localization/launch/localization_corrected.launch` | `rviz=false`, `team_scene_info_path=generated_building/team_scene_info.json`, `correction_enabled=true`, `correction_mode=planar_relative` |
| `competition_bringup/launch/competition_system.launch` | `competition_mode=true`, `auto_start=true`, `localization_enabled=false`, `floor_lifecycle_enabled=true`, `planning_room_mission_enabled=true`, `planning_room_scan_zones=<4 boxes>`, `planning_visual_scan_route_spacing=4.0` |

`localization_enabled=false` + the explicit `localization_corrected.launch`
include is what makes that chain the **only** `/competition/localization/odom`
publisher in the run (verified live node list in `gt_compliance.txt`).

### 3.2 Localization chain (recovered from `gt_compliance.txt` node list)

```text
/livox/Pointcloud2 -> filter_livox -> /livox/Pointcloud2_filtered -> laserMapping (FAST-LIO) -> /Odometry
/a1_gazebo/joint_states + /trunk_imu -> leg_odometry -> /leg_odometry
localization_node (correction planar_relative) -> /competition/localization/odom
```

Live nodes observed: `filter_livox`, `laserMapping`, `leg_odometry`,
`localization`, `pointcloud2livox`, `mapping`, `planning`, `perception`,
`path_follower`, `competition_loop`, `cmd_vel_guard`.

### 3.3 Room mission config / zones / scan spacing (from this run's `params.txt`)

```text
PLANNING_ROOM_MISSION_ENABLED = true
PLANNING_ROOM_SCAN_ZONES = [[1.1,9.5,7.9,21.9],[-9.5,-1.1,7.9,21.9],[1.1,9.5,21.9,35.9],[-9.5,-1.1,21.9,35.9]]
PLANNING_VISUAL_SCAN_ROUTE_SPACING = 4.0
PLANNING_INFLATION_RADIUS = 0.25
RETURN_ARRIVAL_TOLERANCE = 0.25
```

These are the values `rosparam get` returned **live** during the authoritative
run, e.g. `/planning/room_mission_enabled = true`,
`/planning/visual_scan_route_spacing = 4.0`.

### 3.4 Scene generation settings (from `auto_stdout.log` + `scene_manifest.json`)

```text
BUILDING_WIDTH = 20.0
BUILDING_LENGTH = 36.0
DANGER_COUNT_RANGE = 3:6
DISTRACTOR_COUNT_RANGE = 4:8
SCENE_DANGER_COUNT = 6
SCENE_DISTRACTOR_COUNT = 7
ROBOT_START = 0.0,-3.2,0.6,1.5708
GAZEBO_PHYSICS = max_step=0.002  update_rate=500  ode_iters=40  contact_max_correcting_vel=5.0
WRITE_GENERATED_TRUTH_COPY: 0
UNITREE_AUTO_CMD_VEL: 1
CONTROLLER_FOREGROUND: 0
ENABLE_SENSORS: 1
ENABLE_REALSENSE: 1
PAUSED: false   AUTO_UNPAUSE: true after 6s
```

Resolved scene identity (`generated_building/building_config.json`):
`seed=77`, `num_floors=1`, `room_count=4`, `building_width=20.0`,
`building_depth=36.0`, `floor_heights=[0.0]`, `entrance_pose=[0,0,0,0,0,0]`.

### 3.5 Entrance apron setting (production constant, read-only)

The apron is emitted by production `exporter._append_entrance_apron`:

```text
size = (4.5, 2.4, 0.08)   pose = (0.0, -1.2, -0.04, 0, 0, 0)   color = 0.62 0.62 0.64 1
```

i.e. an 8 cm-thick slab whose **top face is coplanar with the Gazebo ground**
(z = -0.04 + 0.04 = 0.0), spanning x ∈ [-2.25, 2.25], y ∈ [-2.4, 0.0]. This
baseline does **not** change it; the gate only records that the generated world
carries it (`entrance_apron` present in `generated_building/model.sdf`).

### 3.6 Main entrance clearance (the thing under test)

```text
MAIN_ENTRANCE_CLEARANCE = 2.0
```

Recomputed by the runner from the **freshly generated**
`generated_building/door_config.yaml`, using production's own geometry:
`panel_width = width/2 = 1.0`, `left_open_y = -1.65`, `inner_edge =
|left_open_y| - panel_width/2 = 1.15 >= width/2 = 1.0`, therefore the opening is
fully clear and the clearance equals the jamb width `2.000 m`
(pre-fix: `1.120 m`). The runner additionally requires
`|left/right_open_y| >= 0.75 * width`, the criterion production itself uses in
`exporter._validate_door_config`.

### 3.7 Effective `max_time`

```text
MAX_TIME = 7200.0
```

`src/competition_bringup/launch/competition_system.launch` hard-codes
`<param name="max_time" value="600.0"/>` inside its `competition_loop` node and
exposes **no `<arg>`** for it, so a production launch arg cannot set it.

**How this baseline reaches 7200 without touching production:** the validation
wrapper sets the parameter **after** the `<include>`:

```xml
<include file=".../competition_system.launch"> ... </include>
<param name="/competition_loop/max_time" value="$(arg max_time)" />
```

A `<param>` placed *before* the include is overwritten by the included node's
private param (this is exactly the limitation recorded in the old
`floor0_4room_v1` README); placing it *after* the include makes the override
survive roslaunch ordering. Verified live in the authoritative run:
`rosparam get /competition_loop/max_time` → `7200.0` (`params.txt`).

Because "requested 7200" and "resolved 7200" are different things, the runner
does not trust the launch file: `MAX_TIME_GATE` reads the **live parameter**
back from the parameter server and fails closed unless it is `7200.0`.

### 3.8 GUI / rendering requirements

```text
GUI = true
DISPLAY = :1 (real host Xorg; container bind-mounts /tmp/.X11-unix and XAUTHORITY)
```

The authoritative run rendered on the host NVIDIA GPU (`glxinfo`: direct
rendering) with **both** `gzserver` and `gzclient` alive, and captured real `xwd`
root-window screenshots. The runner requires (a) `DISPLAY` set, (b) `gzclient`
running, and fails `GUI_GATE` otherwise. If you must run headless, set `GUI=0`
explicitly — the runner will then mark `GUI_GATE` as `DEVIATION` (reported,
still visible in the final report) instead of silently diverging from the
frozen evidence.

### 3.9 RGB-D topic readiness contract

```text
/real_sense/rgb/image_raw
/real_sense/rgb/camera_info
/real_sense/depth/image_raw
/real_sense/depth/camera_info
/real_sense/depth/points
```

`RGBD_GATE` requires publishers to exist **and** message stamps to advance for
`/real_sense/rgb/image_raw` and `/real_sense/depth/image_raw` (CameraInfo is
checked too and reported). Authoritative run: `rgb 604 / depth 604 /
rgb_ci 115 / depth_ci 115` messages, all advancing.

### 3.10 Ground-truth / referee-channel contract

```text
ENABLE_GROUND_TRUTH: 0
ENABLE_REFEREE_ODOM: 0
WRITE_GENERATED_TRUTH_COPY: 0
```

`GT_COMPLIANCE_GATE` fails closed if:

* `/Odometry_gazebo`, `/ground_truth/odom`, `/ground_truth/state` have any
  publisher; or
* any **production** node subscribes to `/gazebo/model_states`,
  `/gazebo/link_states`, `/ground_truth/odom`, `/ground_truth/state`,
  `/Odometry_gazebo`.

Production node set checked: `competition_loop`, `planning`, `mapping`,
`perception`, `localization`, `laserMapping`, `filter_livox`, `leg_odometry`,
`path_follower`, `cmd_vel_guard`, `pointcloud2livox`.
`/unitree_gazebo_servo` (a Gazebo joint-servo plugin) subscribing to
`/ground_truth/*` is **not** a violation: those topics have no publishers.

ROS endpoints used by the run:

```text
ROS_MASTER_URI = http://127.0.0.1:11311
GAZEBO_MASTER_URI = http://127.0.0.1:11345
```

## 4. Gate contract

| # | Gate | Fail-closed condition |
|---|---|---|
| 1 | `HEAD_SOURCE_GATE` | `PRODUCTION_BASE_HEAD` is not an ancestor of `HEAD`, **or** `3503f44..HEAD` changes any tracked file outside `TOOLING_SCOPE`, **or** any production path (`src/**`, `auto.sh`, `setup_multi_floor_simulation.py`, `docker/**`, `compose.yaml`, `scripts/**`, `.gitmodules`) differs from the base (tracked `__pycache__/*.pyc` excepted, reported as a note) |
| 2 | `SUBMODULE_GATE` | `src/fast_lio` / `src/livox_ros_driver` HEAD != frozen, or submodule worktree dirty |
| 3 | `BUILD_GATE` | `devel/setup.bash` missing, ROS not sourceable, or a required package/node/launch is missing |
| 4 | `PROCESS_GATE` | any stale `gzserver` / `gzclient` / `gazebo` / `rosmaster` / `junior_ctrl` / `competition_loop` / `laserMapping` / `building_generator_classic_control` / `rosbag record` process is alive |
| 5 | `SCENE_GATE` | generated scene is not `seed=77` / `num_floors=1` / `room_count=4` (and not 6 dangers / 7 distractors) |
| 6 | `DOOR_GATE` | main entrance open clearance != 2.0 m |
| 7 | `RGBD_GATE` | RGB or depth publisher missing, or stamps not advancing |
| 8 | `MAX_TIME_GATE` | live `/competition_loop/max_time` != 7200.0 |
| 9 | `GT_COMPLIANCE_GATE` | a forbidden truth topic has a publisher, or a production node subscribes to one |

Additional gates (not in the required nine, still fail-closed):

* `SIM_READY_GATE` — `/use_sim_time`, advancing `/clock`, robot spawned,
  `RL_HANDOFF_READY=True` in **this run's** `logs/junior_ctrl.log`.
* `GUI_GATE` — `DISPLAY` set and `gzclient` alive (or explicit `GUI=0`
  deviation).
* `CLOSURE_GATE` — post-run: `RETURNING` entered, `ARRIVED_START`,
  `FINISHED`, `CURRENT_FLOOR_COMPLETE`, `HAZARD_UNIQUE = 6/6`,
  `FALSE_ALARMS_WRITTEN = 0`, `MAX_TIME_REACHED = False`,
  `RETURNING_GOAL_NOT_REACHABLE = 0`, both entrance crossings observed.

Every gate starts as `NOT_RUN`; the run aborts at the first `FAIL`, so a green
final report means every gate was actually evaluated.

## 5. Diagnostics — explicitly NOT failure conditions

```text
ROOM_SCAN_ZONES_TRIGGERED = 3/4   (diagnostic)
```

The authoritative run physically entered 3 of the 4 room scan zones (the NE
room, zone index 2, `x 1.1..9.5, y 21.9..35.9`, was never entered). The
mission's own coverage contract was satisfied
(`CURRENT_FLOOR_COMPLETE floor=0 remaining=[]`,
`EXPLORATION_COMPLETE_LOGGED=True`) and the requested success sequence contains
no 4/4 requirement. The runner therefore **reports** the zone count and never
fails on it.

Also reported, never fatal: `DETECTED_DANGER_JSON_COUNT` may exceed 6 because
one truth source (`danger_red_sphere_01`) can be represented by two clusters
> `hazard_cluster_radius` apart; both are true positives, `FALSE_ALARMS = 0`.

## 6. Non-goals / explicit exclusions

* **No production source change.** `PRODUCTION_SOURCE_CHANGED = false`. The
  only new files are the four under `tools/baselines/floor0-doorfix-closure-v1/`.
* Do **not** copy `floor0_4room_v1`'s `4/4` zones or its
  `RETURN_REACHABILITY = FAIL` conclusion into this baseline — they belong to a
  different (pre-door-fix) freeze.
* Do **not** reset or clean tracked `*.pyc`; the authoritative run itself ran
  with modified tracked `__pycache__` files and the runner treats that as a
  reported note, not a failure.
* Do **not** delete existing evidence under `artifacts/`.
