# floor0-doorfix-closure-v1 — reproducible Floor0 full-closure baseline

> Validation / baseline-reproduction directory. **Not production defaults.**
> **No production algorithm or production file was changed.**

## Status — read first

```text
Production baseline 3503f44:
  previously validated full Floor0 closure = PASS
  (SEED=77: ENTRY -> exploration -> 6/6 hazards -> CURRENT_FLOOR_COMPLETE
   -> RETURNING -> entrance crossing -> ARRIVED_START -> FINISHED)

This reproduction wrapper:
  static / preflight validation = PASS
  (HEAD_SOURCE / SUBMODULE / BUILD / PROCESS / SCENE / DOOR / SIM_READY /
   RGBD / GUI / MAX_TIME / GT_COMPLIANCE all verified live in a fresh clone)

Independent teammate live reproduction:
  PENDING
```

The runner itself has **not** yet been run to full closure on a fresh machine.
The first real cross-environment verification is performed by a **teammate**;
their run also doubles as the visual/video capture session, so the owner
workstation does not repeat another 45-minute run.

## What this baseline freezes

The Floor0 configuration that produced the full, untruncated, autonomous
closure on commit `3503f44e617385c56f264875575d8ca8b68c3b36`
("fix(building-generator): fully clear main entrance when open"), with
`max_time` actually resolving to **7200 s** and **zero** ground-truth feedback.

Provenance for every frozen value: [`BASELINE_MANIFEST.md`](BASELINE_MANIFEST.md),
recovered **only** from the verified run
`artifacts/floor0-full-repro/floor0-full-repro-20260913-155635`
(`params.txt`, `run_summary.md`, `runner.log`, `auto_stdout.log`,
`gt_compliance.txt`, `scene_manifest.json`, `door_clearance.txt`,
`evaluation.txt`) — **not** from the older `floor0_4room_v1` freeze.

## Teammate quick start (copy-paste)

Requires: a GPU workstation with a real X display, Docker, and the
`simenv:noetic-cu128` image (the team container image).

### 1. Clone + submodules

```bash
git clone --recurse-submodules https://github.com/Carl-Huang12138/DG-202602-SimEnv.git
cd DG-202602-SimEnv
git switch --track origin/baseline/floor0-doorfix-closure-v1
git submodule sync --recursive
git submodule update --init --recursive
```

> If `git submodule update` fails on the nested
> `src/fast_lio/include/ikd-Tree` with a transient
> `curl 16 Error in the HTTP2 framing layer`, re-run it (optionally with
> `git -c http.version=HTTP/1.1 submodule update --init --recursive`).
> A partially initialised clone does not build.

### 2. Start the container (complete command)

Run this from the repository root (`$PWD` must be the clone):

```bash
docker run -d --name simenv-doorfix-closure \
  --hostname simenv-doorfix-closure \
  --add-host simenv-doorfix-closure:127.0.0.1 \
  --network host \
  --shm-size 2gb \
  --init \
  --gpus all \
  -e DISPLAY="${DISPLAY:-:1}" \
  -e XAUTHORITY=/tmp/.Xauthority \
  -e QT_X11_NO_MITSHM=1 \
  -e NVIDIA_VISIBLE_DEVICES=all \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e ROS_HOME=/workspace/SimEnv/artifacts/ros \
  -v "$PWD":/workspace/SimEnv \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}":/tmp/.Xauthority:ro \
  -w /workspace/SimEnv \
  simenv:noetic-cu128 \
  sleep infinity
```

Every flag above is required:

* `--add-host <hostname>:127.0.0.1` — without it the container's own hostname
  does not resolve and `roslaunch` dies during startup with
  `RLException: Unable to contact my own server at [http://<hostname>:<port>/]`
  (an environment fault, never a mission result).
  `getent hosts "$(hostname)"` must print `127.0.0.1` before the run.
* `-e DISPLAY` + the two X11 bind mounts — GUI=true is the frozen
  configuration; `gzclient` must render on a real display.
* `--network host` + `--shm-size 2gb` + `--gpus all` — the sim and the
  GPU-rendered sensors need them.
* A **clean** container: `PROCESS_GATE` fails closed if a `gzserver`/
  `rosmaster` from an earlier attempt is still alive in the same container.

> Equivalent alternative: `docker compose up -d --no-build` from the repo
> root — `compose.yaml` carries the same settings (`extra_hosts`, X11 mounts,
> GPU, host network). The `docker run` command above is the explicit form.

### 3. Build (first time only)

```bash
docker exec simenv-doorfix-closure bash -lc \
  'source /opt/ros/noetic/setup.bash && cd /workspace/SimEnv && catkin_make -j'
```

### 4. Run the reproduction

```bash
docker exec simenv-doorfix-closure bash -lc \
  'cd /workspace/SimEnv && bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh'
```

That one command regenerates the SEED=77 scene, starts Gazebo with
`GUI=true`, gates everything (see below), launches the frozen mission, waits
for it to finish autonomously, and evaluates the closure. Expected wall time
is **~45 minutes** at RTF ~0.3. Do not touch the keyboard or the simulation.

Pre-flight only (starts nothing, regenerates nothing):

```bash
docker exec simenv-doorfix-closure bash -lc \
  'cd /workspace/SimEnv && bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh --dry-run'
```

## GUI + recording checklist

The run is `GUI=true`: a Gazebo window must be visible on your display.

1. Start screen recording **when you launch the runner** — the runner spends
   its first minutes on startup gates, so nothing is missed. Any recorder is
   fine, e.g. on the workstation:

   ```bash
   ffmpeg -video_size 1920x1080 -f x11grab -i "$DISPLAY" -pix_fmt yuv420p ~/floor0_repro.mp4
   ```

2. Before treating the run as the formal attempt, confirm (the runner also
   gates all of these — if a gate fails, it aborts and says why):

   ```text
   gzserver alive      gzclient alive
   RGB advancing       Depth advancing      (RGBD_GATE)
   ```

3. **Never drive the robot manually** for the recording — the run must stay
   fully autonomous.

Footage worth keeping (full original video + trimmed segments):

```text
1. Gazebo scene and the A1's initial state
2. Entering the main entrance
3. Normal autonomous Floor0 navigation
4. Room scanning behaviour
5. Hazard identification moments
6. CURRENT_FLOOR_COMPLETE -> switch to RETURNING
7. Driving back toward the main entrance
8. Crossing the entrance outward
9. ARRIVED_START
10. FINISHED
```

## Teammate success criteria

```text
SEED=77  FLOOR_COUNT=1  ROOMS_PER_FLOOR=4

RGBD_GATE=PASS
MAX_TIME_GATE=PASS            (live /competition_loop/max_time == 7200.0)
GT_COMPLIANCE_GATE=PASS

HAZARDS=6/6                   FALSE_ALARMS=0

CURRENT_FLOOR_COMPLETE=PASS
RETURNING=PASS
RETURNING_GOAL_NOT_REACHABLE_COUNT=0

HOME_REACHED=true             FINISHED=true
MAX_TIME_REACHED=false

GROUND_TRUTH_FEEDBACK_COUNT=0
MANUAL_INTERVENTION_COUNT=0
```

`ROOM_SCAN_ZONES_TRIGGERED = 3/4` is **diagnostic only** — see below.

## If the run fails

First principles:

```text
DO NOT PATCH
DO NOT TUNE
```

Save, before changing anything:

```text
run logs (artifacts/baseline-floor0-doorfix-closure-v1-*/)
resolved params (live_audit.txt), git HEAD, submodule HEADs
RGB-D health (camera_check.txt)
Gazebo screenshot/video
first divergence
```

Then report:

```text
FIRST_DIVERGENCE_SIM_TIME =
EXPECTED =
OBSERVED =
```

Only after that decide whether to modify anything.

## Frozen knobs

```text
SEED=77  FLOOR_COUNT=1  ROOMS_PER_FLOOR=4
ENABLE_GROUND_TRUTH=0  ENABLE_REFEREE_ODOM=0  WRITE_GENERATED_TRUTH_COPY=0

localization_corrected.launch  rviz=false  correction_enabled=true  correction_mode=planar_relative
competition_system.launch      localization_enabled=false  floor_lifecycle_enabled=true
                               planning_room_mission_enabled=true
                               planning_room_scan_zones = 4 boxes
                               planning_visual_scan_route_spacing = 4.0

GUI=true (gzclient on a real X display)   ENABLE_REALSENSE=1
/competition_loop/max_time = 7200.0 (resolved, verified live)
main entrance open clearance = 2.000 m
```

## Why `max_time` really is 7200 here

Production `src/competition_bringup/launch/competition_system.launch`
hard-codes `<param name="max_time" value="600.0"/>` inside its
`competition_loop` node and exposes no `<arg>`. Production is **not**
modified. The validation wrapper
(`floor0_doorfix_closure.launch`) sets `/competition_loop/max_time=7200`
**after** the `<include>` — a `<param>` written *before* the include is
overwritten by the node's private parameter. Because "requested" is not
"resolved", `MAX_TIME_GATE` reads the parameter back from the **live**
parameter server and fails closed unless it is exactly `7200.0`.

## Three identities — read before comparing commits

```text
PRODUCTION_BASE_HEAD        = 3503f44e617385c56f264875575d8ca8b68c3b36
                              the production commit under test (ancestor)
REPRODUCTION_TOOLING_COMMIT = the descendant commit that adds this directory
                              (the runner prints it; never used as a gate —
                               a commit cannot contain its own hash)
PRODUCTION_SOURCE_CHANGED   = false
```

The runner does **not** require `HEAD == 3503f44` (committing this directory
makes that unsatisfiable). It requires `3503f44` to be an ancestor of `HEAD`
and every tracked difference since it to live inside this directory; the
production paths (`src/**`, `auto.sh`, `setup_multi_floor_simulation.py`,
`docker/**`, `compose.yaml`, `scripts/**`, `.gitmodules`) must be zero-diff.

> Environment note: the runner proves provenance with `git`, so it must run
> where the repository's git metadata is reachable. Use a **plain clone**; if
> you bind-mount a `git worktree` into a container, mount the parent
> repository as well, otherwise `HEAD_SOURCE_GATE` fails closed with
> "git metadata unreadable".

## Files

| File | Purpose |
|---|---|
| `BASELINE_MANIFEST.md` | frozen identity + every resolved parameter + gate contract (parsed by the runner) |
| `README.md` | this file |
| `floor0_doorfix_closure.launch` | validation-only wiring: localization chain + frozen mission args + `max_time` override |
| `run_floor0_doorfix_closure.sh` | one-command reproduction with the fail-closed gates and post-run closure evaluation |

## Gates (all fail closed)

| Gate | Check |
|---|---|
| `HEAD_SOURCE_GATE` | `3503f44` is an ancestor of `HEAD`, every tracked change since it is inside `tools/baselines/floor0-doorfix-closure-v1`, and all production paths are zero-diff |
| `SUBMODULE_GATE` | `src/fast_lio` / `src/livox_ros_driver` HEADs match exactly and are clean |
| `BUILD_GATE` | `devel/setup.bash`, 10 catkin packages, required nodes/launches exist |
| `PROCESS_GATE` | no stale `gzserver` / `gzclient` / `rosmaster` / `junior_ctrl` / `competition_loop` / `laserMapping` / `rosbag record` |
| `SCENE_GATE` | freshly generated scene is `seed=77`, 1 floor, 4 rooms, 6 dangers, 7 distractors |
| `DOOR_GATE` | main entrance open clearance is `2.000 m` in **this run's** generated `door_config.yaml` |
| `SIM_READY_GATE` | `/use_sim_time`, robot spawned, `/clock` advancing, `RL_HANDOFF_READY=True` in a fresh log |
| `RGBD_GATE` | `/real_sense/rgb\|depth/image_raw` publishers live **and** stamps advancing |
| `GUI_GATE` | `gzserver` + `gzclient` alive on a real display (explicit `GUI=0` is recorded as `DEVIATION`) |
| `MAX_TIME_GATE` | **live** value of `/competition_loop/max_time` is `7200.0` |
| `GT_COMPLIANCE_GATE` | no forbidden truth publisher; no production node subscribes to a truth channel |
| `CLOSURE_GATE` | the full sequence above, 6/6 hazards, 0 false alarms, both entrance crossings, and **no** `generated_building/danger_truth.json` on disk (the frozen `WRITE_GENERATED_TRUTH_COPY=0`) |

Every gate starts as `NOT_RUN`; the run aborts at the first `FAIL`, so a green
report means every gate was really evaluated. Exit code `0` only if all pass.

## Reading a run

Evidence is written to
`artifacts/baseline-floor0-doorfix-closure-v1-<timestamp>/`:

| File | Content |
|---|---|
| `scene_check.txt`, `door_check.txt` | scene identity and entrance clearance |
| `readiness.log`, `camera_check.txt` | sim readiness and RGB-D contract |
| `live_audit.txt` | effective `max_time` + GT/referee audit |
| `closure_evaluation.txt` | post-run closure metrics (required vs diagnostic) |
| `competition_stack.log`, `run.bag`, `watcher.tsv` | roslaunch console, topic recording, production-odom trajectory |
| `scene/` | copy of the generated scene, truth and detection files |

Expected values for a reproducing run:

```text
TOTAL_SIM_TIME                     ~736.8 s  (wall ~45 min at RTF ~0.3)
MAX_TIME_REACHED                   False
HAZARD_UNIQUE_DETECTED_COUNT       6  (FALSE_ALARMS_WRITTEN = 0)
RETURNING_GOAL_NOT_REACHABLE_COUNT 0
ARRIVED_START_DISTANCE_M           ~0.206
```

## Diagnostics that are NOT failures

* `ROOM_SCAN_ZONES_TRIGGERED = 3/4`. The NE room (zone index 2) was not
  physically entered in the authoritative run either. The mission's own
  coverage contract was satisfied (`CURRENT_FLOOR_COMPLETE floor=0
  remaining=[]`), and the success sequence contains no 4/4 requirement.
  Reported, never fatal.
* `DETECTED_DANGER_JSON_COUNT` can be 7 for 6 truth sources: one source may
  be written as two clusters further apart than `hazard_cluster_radius`.
  Both are true positives; `FALSE_ALARMS_WRITTEN` stays 0.

## Do not

* Do not copy `floor0_4room_v1`'s historical `4/4` zones or its
  `RETURN_REACHABILITY = FAIL` conclusion into this baseline — they belong to
  a different (pre-door-fix) freeze.
* Do not edit production source to make the runner pass. If a gate cannot be
  satisfied, report it instead.
* Do not reset/clean tracked `*.pyc` or delete existing evidence under
  `artifacts/`; the runner treats modified tracked `__pycache__` as a note
  only.
