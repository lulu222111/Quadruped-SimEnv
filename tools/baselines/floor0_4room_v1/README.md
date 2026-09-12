# floor0-4room-v1 — reproducible Floor0 four-room baseline

> Validation / baseline-reproduction directory. **Not production defaults.**

This baseline freezes the one Floor0 configuration that produced

```text
ROOM_SCAN_ZONES_TRIGGERED = 4/4
CURRENT_FLOOR_COMPLETE    = PASS
```

It is a **historically validated exploration baseline**, **not** a
competition-faithful final scene baseline. Read
[`BASELINE_MANIFEST.md`](BASELINE_MANIFEST.md) for the full provenance,
including `KNOWN_SCENE_DEVIATIONS`.

## Teammate quick start

Fresh clone:

```bash
git clone --recurse-submodules \
    https://github.com/Carl-Huang12138/DG-202602-SimEnv.git
cd DG-202602-SimEnv

git switch -c feat/<TEAMMATE>/entry-threshold-v1 \
    origin/baseline/floor0-4room-v1
git submodule sync --recursive
git submodule update --init --recursive
```

Already cloned:

```bash
cd ~/DG-202602-SimEnv
git fetch origin
git switch -c feat/<TEAMMATE>/entry-threshold-v1 \
    origin/baseline/floor0-4room-v1
git submodule sync --recursive
git submodule update --init --recursive
```

Build:

```bash
catkin_make -j
source ./devel/setup.bash
```

Run the frozen Floor0 configuration:

```bash
bash tools/baselines/floor0_4room_v1/run_floor0_4room_baseline.sh
```

This entry point depends on **nothing** outside this repository: no private
absolute paths, no `/tmp` wrappers, no uncommitted patches, no manual
submodule patches.

## Files

| File | Purpose |
|---|---|
| `BASELINE_MANIFEST.md` | frozen identity, commits, patch digests, validated result, known limitations, scene deviations |
| `README.md` | this file |
| `floor0_4room_replay.launch` | validation-only wrapper selecting the frozen parameters |
| `run_floor0_4room_baseline.sh` | one-command reproduction with checkout/env/readiness gates |

## Frozen knobs

```text
SEED=77  FLOOR_COUNT=1  ROOMS_PER_FLOOR=4

ENABLE_GROUND_TRUTH=0  ENABLE_REFEREE_ODOM=0  WRITE_GENERATED_TRUTH_COPY=0
UNITREE_AUTO_CMD_VEL=1  CONTROLLER_FOREGROUND=0

localization_corrected.launch  correction_enabled=true  correction_mode=planar_relative
competition_system.launch      localization_enabled=false
                               floor_lifecycle_enabled=true
                               planning_room_mission_enabled=true
                               planning_room_scan_zones = 4 boxes
                               planning_visual_scan_route_spacing=4.0
```

## Known wrapper caveat — read before comparing run lengths

The validated runs used a `/competition_loop/max_time` of **1800 s**. That is a
**validation watchdog only**; the formal mission budget is **600 s**
(`src/competition_bringup/config/default.yaml`, `competition_system.launch`,
`time_keeper.py`, and the scoring threshold in `docs/evaluation.md`).

`competition_system.launch` hard-codes `<param name="max_time" value="600.0"/>`
inside its `competition_loop` node and exposes **no `<arg>`** for it. Because
this baseline must not edit that production launch, the wrapper can only set
the parameter before its `<include>`, and roslaunch lets the included file's
value win. Always check what actually took effect:

```bash
rosparam get /competition_loop/max_time
```

If that reports `600.0`, the run is bounded by the formal budget — this is a
known, reported limitation of keeping production untouched, **not** a silent
change. Raising it cleanly would require adding a pass-through
`<arg name="max_time" default="600.0"/>` to `competition_system.launch`, which
is deliberately out of scope for this freeze.

## Known limitations

```text
RETURN_PATH_GENERATION  = PASS
RETURN_PATH_EXECUTION   = PARTIAL_PASS
RETURN_REACHABILITY     = FAIL

FIRST_FULL_NATURAL_FLOOR0_CLOSURE = NOT_YET_VALIDATED

KNOWN_BLOCKER =
main entrance connectivity cut in inflated map during RETURNING replanning
```

Do not treat a green exploration run here as Floor0 return closure.
