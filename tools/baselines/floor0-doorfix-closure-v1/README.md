# floor0-doorfix-closure-v1 — reproducible Floor0 full-closure baseline

> Validation / baseline-reproduction directory. **Not production defaults.**
> **No production algorithm or production file was changed.**

This baseline freezes the Floor0 configuration that produced the full,
untruncated, autonomous closure

```text
ENTRY -> exploration -> 6/6 hazards -> CURRENT_FLOOR_COMPLETE
      -> RETURNING -> entrance crossing -> ARRIVED_START -> FINISHED
```

on commit `3503f44e617385c56f264875575d8ca8b68c3b36`
("fix(building-generator): fully clear main entrance when open"), with
`max_time` actually resolving to **7200 s** and **zero** ground-truth feedback.

Provenance for every frozen value: [`BASELINE_MANIFEST.md`](BASELINE_MANIFEST.md).
It was recovered **only** from the verified run

```text
artifacts/floor0-full-repro/floor0-full-repro-20260913-155635
```

(`params.txt`, `run_summary.md`, `runner.log`, `auto_stdout.log`,
`gt_compliance.txt`, `scene_manifest.json`, `door_clearance.txt`,
`evaluation.txt`) — **not** from the older `floor0_4room_v1` freeze.

## Teammate quick start (fresh clone, one command)

Inside the team container (`simenv:noetic-cu128`, workspace mounted at
`/workspace/SimEnv`, real X display for `gzclient`):

Check out the **reproduction tooling commit** (a descendant of the production
base) — not the base itself, which does not contain this directory:

```bash
git clone --recurse-submodules https://github.com/Carl-Huang12138/DG-202602-SimEnv.git \
  && cd DG-202602-SimEnv \
  && git checkout origin/fix/huangym/main-entrance-open-geometry-v1 \
  && git submodule sync --recursive \
  && git submodule update --init --recursive \
  && catkin_make -j \
  && bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh
```

Already cloned:

```bash
cd ~/DG-202602-SimEnv
git fetch origin
git checkout origin/fix/huangym/main-entrance-open-geometry-v1
git submodule sync --recursive && git submodule update --init --recursive
bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh
```

The runner prints `HEAD` (= `REPRODUCTION_TOOLING_COMMIT`) and proves that
`PRODUCTION_BASE_HEAD=3503f44…` is its ancestor with zero production diff.

Pre-flight only (nothing is started, nothing is regenerated):

```bash
bash tools/baselines/floor0-doorfix-closure-v1/run_floor0_doorfix_closure.sh --dry-run
```

The runner depends on **nothing** outside this repository: no private absolute
paths, no `/tmp` wrappers, no uncommitted patches.

### Container prerequisites (verified live)

* **ROS networking.** Start the container through `compose.yaml`
  (`docker compose up -d --no-build`), which sets
  `extra_hosts: "<hostname>:127.0.0.1"`. When creating it by hand, pass the same
  mapping explicitly:

  ```bash
  docker run … --network host --hostname simenv-closure \
      --add-host simenv-closure:127.0.0.1 …
  ```

  Without it the container's own hostname does not resolve, `auto.sh` starts and
  generates the scene, and then `roslaunch` dies during startup with

  ```text
  RLException: Unable to contact my own server at [http://<hostname>:<port>/]
  ```

  `getent hosts "$(hostname)"` must print `127.0.0.1` before the run. This is an
  environment fault, never a mission result.
* **Display.** GUI=true is the frozen configuration, so `DISPLAY` must be set and
  `/tmp/.X11-unix` + the `XAUTHORITY` file must be bind-mounted (again, what
  `compose.yaml` does).
* **Clean container.** `PROCESS_GATE` fails closed if a `gzserver`/`rosmaster`
  from an earlier attempt is still alive in the same container.
* **Submodules.** `git submodule update --init --recursive` occasionally fails on
  the nested `src/fast_lio/include/ikd-Tree` with a transient
  `curl 16 Error in the HTTP2 framing layer`. Re-run it (optionally with
  `-c http.version=HTTP/1.1`); a partially initialised clone does not build.

### Three identities — read before comparing commits

```text
PRODUCTION_BASE_HEAD        = 3503f44e617385c56f264875575d8ca8b68c3b36
                              the production commit under test (ancestor)
REPRODUCTION_TOOLING_COMMIT = the descendant commit that adds this directory
                              (the runner prints it; never used as a gate —
                               a commit cannot contain its own hash)
PRODUCTION_SOURCE_CHANGED   = false
```

The runner therefore does **not** require `HEAD == 3503f44` (committing this
directory makes that unsatisfiable). It requires `3503f44` to be an ancestor,
with the only tracked difference being this directory.

> Environment note: the runner proves provenance with `git`, so it must run
> where the repository's git metadata is reachable. Use a **plain clone**; if
> you bind-mount a `git worktree` into a container, mount the parent repository
> as well, otherwise `HEAD_SOURCE_GATE` fails closed with "git metadata
> unreadable".

## Files

| File | Purpose |
|---|---|
| `BASELINE_MANIFEST.md` | frozen identity + every resolved parameter + gate contract (parsed by the runner) |
| `README.md` | this file |
| `floor0_doorfix_closure.launch` | validation-only wiring: localization chain + frozen mission args + `max_time` override |
| `run_floor0_doorfix_closure.sh` | one-command reproduction with the fail-closed gates and post-run closure evaluation |

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

## Gates (all fail closed)

| Gate | Check |
|---|---|
| `HEAD_SOURCE_GATE` | `3503f44` is an ancestor of `HEAD`, every tracked change since it is inside `tools/baselines/floor0-doorfix-closure-v1`, and all production paths (`src/**`, `auto.sh`, `setup_multi_floor_simulation.py`, `docker/**`, `compose.yaml`, `scripts/**`, `.gitmodules`) are zero-diff |
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

## Why `max_time` really is 7200 here

`competition_system.launch` hard-codes `<param name="max_time" value="600.0"/>`
inside its `competition_loop` node and exposes no `<arg>`. The wrapper therefore
sets the parameter **after** the `<include>` (a `<param>` written *before* the
include is overwritten by the node's private parameter — exactly the limitation
recorded in `floor0_4room_v1`). Because "requested" is not "resolved",
`MAX_TIME_GATE` reads the parameter back from the live parameter server.

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
  physically entered in the authoritative run either. The mission's own coverage
  contract was satisfied (`CURRENT_FLOOR_COMPLETE floor=0 remaining=[]`), and
  the success sequence contains no 4/4 requirement. Reported, never fatal.
* `DETECTED_DANGER_JSON_COUNT` can be 7 for 6 truth sources: one source may be
  written as two clusters further apart than `hazard_cluster_radius`. Both are
  true positives; `FALSE_ALARMS_WRITTEN` stays 0.

## Do not

* Do not copy `floor0_4room_v1`'s historical `4/4` zones or its
  `RETURN_REACHABILITY = FAIL` conclusion into this baseline — they belong to a
  different (pre-door-fix) freeze.
* Do not edit production source to make the runner pass. If a gate cannot be
  satisfied, report it instead.
* Do not reset/clean tracked `*.pyc` or delete existing evidence under
  `artifacts/`; the runner treats modified tracked `__pycache__` as a note only.
