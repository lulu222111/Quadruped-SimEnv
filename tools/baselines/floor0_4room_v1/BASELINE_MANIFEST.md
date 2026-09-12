# BASELINE_MANIFEST — floor0-4room-v1

> **This is a historically validated EXPLORATION baseline, NOT a
> competition-faithful final scene baseline.**
>
> The `PASS` recorded below means **Floor0 exploration reached 4/4 room scan
> zones and `CURRENT_FLOOR_COMPLETE`**. It does **not** mean Floor0 return
> closure passed.

```text
floor0-4room-v1 =
historical validated exploration baseline

NOT =
competition-faithful final scene baseline
```

---

## 1. Identity

```text
BASELINE_NAME = floor0-4room-v1

SUPERPROJECT_BASE           = 08704862c05b4c97a8a80fbe71ca14b262528612
SUPERPROJECT_BASELINE_COMMIT = <superproject tip of baseline/floor0-4room-v1>
SUPERPROJECT_REMOTE_BRANCH  = origin/baseline/floor0-4room-v1
SUPERPROJECT_REMOTE         = https://github.com/Carl-Huang12138/DG-202602-SimEnv.git

SOURCE_WORKTREE             = /home/ubuntu/SimEnv-floor0-completion-speed-v1
SOURCE_BRANCH               = fix/huangym/floor0-completion-speed-v1
```

`SUPERPROJECT_BASELINE_COMMIT` cannot literally contain its own hash (a git
commit cannot embed the hash of the tree it belongs to). The run script
therefore anchors on `SUPERPROJECT_BASE` (must be an ancestor of HEAD, and HEAD
must not diverge from it in tracked source) plus the two frozen submodule
commits, which ARE self-contained and verified exactly.

## 2. Frozen submodules

Both upstream remotes were read-only for this account (`push = false`, HTTP 403
on dry-run push), so each frozen change was published to a project-owned fork
and the gitlink repointed there. This keeps the baseline a genuine
`CLEAN_RECURSIVE_CHECKOUT_BASELINE`: no teammate ever has to apply a local
dirty patch by hand.

```text
FAST_LIO_BASE_COMMIT    = e6aeba10869c2f9f51edb2db2124472bbc569f3c
FAST_LIO_UPSTREAM       = https://github.com/153Ah/jwh_FAST_LIO.git      (read-only)
FAST_LIO_COMMIT         = bf5f05130f693fd9e32a1913d9abbb65f84a07f2
FAST_LIO_FORK           = https://github.com/Carl-Huang12138/jwh_FAST_LIO.git
FAST_LIO_BASELINE_BRANCH = baseline/floor0-4room-v1
FAST_LIO_PATCH_ROLE     = RUNTIME_REQUIRED
FAST_LIO_PATCH_SHA256   = d2033978ee6f5065c4c6920840c1e80a273dc708936c1e2b11d2f0614212317f
FAST_LIO_PATCH_EQUIVALENT = true

LIVOX_BASE_COMMIT       = 3d240d5666129e1a3052e78ee8487a04b08fdda3
LIVOX_UPSTREAM          = https://github.com/Livox-SDK/livox_ros_driver.git (read-only)
LIVOX_COMMIT            = 24cf049379cd32f96b673d62766df4cd5bcd0cf8
LIVOX_FORK              = https://github.com/Carl-Huang12138/livox_ros_driver.git
LIVOX_BASELINE_BRANCH   = baseline/floor0-4room-v1
LIVOX_PATCH_ROLE        = BUILD_ONLY
LIVOX_PATCH_SHA256      = 9735cf823c373bd0d3bfbb55ec66f8a99e4a1704df824730ed7d6c667d159c46
LIVOX_PATCH_EQUIVALENT  = true
```

### Patch equivalence proof

Round-trip: applying the frozen patch to the clean base commit and re-running
`git diff --binary` reproduced a **byte-identical** patch (verified with `cmp`,
not just equal SHA256). Per-file content hashes of all 6 patched files also
match the originally validated working tree exactly.

### Patch contents (audited, no diagnostic code)

| Repo | File | Role |
|---|---|---|
| fast_lio | `CMakeLists.txt` | `add_dependencies(...)` fixes catkin message-header build ordering — BUILD_ONLY |
| fast_lio | `src/IMU_Processing.hpp` | initial gravity alignment (`init_state.rot` from measured gravity); drops the MARSIM `pcl_beg_time` branch — RUNTIME |
| fast_lio | `src/laserMapping.cpp` | `p_pre->det_range = DET_RANGE` — RUNTIME |
| fast_lio | `src/preprocess.cpp` | `std::hypot` true range, NaN/inf rejection, `det_range` upper clip — RUNTIME |
| fast_lio | `src/preprocess.h` | adds `det_range` member — RUNTIME |
| livox_ros_driver | `livox_ros_driver/CMakeLists.txt` | removes the network `git clone Livox-SDK` build fallback; guards the node target behind `LIVOX_SDK_FOUND` — BUILD_ONLY |

Audit result: **no debug print, no validation-only instrumentation, no absolute
path, no machine-specific hack** was introduced by either patch.

## 3. Scene / seed

```text
SEED           = 77
FLOOR_COUNT    = 1
ROOMS_PER_FLOOR = 4
GUI            = false

UNITREE_AUTO_CMD_VEL       = 1
CONTROLLER_FOREGROUND      = 0
ENABLE_GROUND_TRUTH        = 0
ENABLE_REFEREE_ODOM        = 0
WRITE_GENERATED_TRUTH_COPY = 0
```

Every value above was recovered from committed evidence, never guessed:

- `reports/floor0_completion_attempt_matrix.md` — run convention / harness
- `reports/floor0_closure/round1_manifest.md` — configuration under test
- `reports/floor0_closure/round2_manifest.md`

## 4. Localization

```text
LOCALIZATION = FAST-LIO-assisted proprioceptive planar-relative hybrid

chain: localization_corrected.launch
  correction_enabled = true
  correction_mode    = planar_relative

  PLANAR XY   = leg odometry (vel_scale) + one-time alignment
  YAW         = leg odometry yaw + latched offset
  Z           = FAST-LIO
  ROLL/PITCH  = FAST-LIO

competition_system.launch:
  localization_enabled = false   (the chain above owns /competition/localization/odom)
```

## 5. Planning / room mission

```text
ROOM_MISSION       = enabled   (planning_room_mission_enabled = true)
FLOOR_LIFECYCLE    = enabled   (floor_lifecycle_enabled = true)

ROOM_SCAN_ZONES    = 4
  [[1.1, 9.5, 7.9, 21.9],
   [-9.5, -1.1, 7.9, 21.9],
   [1.1, 9.5, 21.9, 35.9],
   [-9.5, -1.1, 21.9, 35.9]]

VISUAL_SCAN_SPACING = 4.0      (planning_visual_scan_route_spacing; production default is 2.0)

VALIDATION_WATCHDOG_MAX_TIME = 1800.0 s   <- validation only
FORMAL_MISSION_BUDGET        =  600.0 s   <- competition_bringup/config/default.yaml,
                                             competition_system.launch, time_keeper.py
                                             and the docs/evaluation.md scoring threshold

VERIFIED_EFFECTIVE_MAX_TIME  =  600.0 s   <- measured with `roslaunch --dump-params`
```

**Verified caveat.** The validated runs used a 1800 s watchdog. This baseline
must not edit `competition_system.launch`, which hard-codes `max_time=600.0`
inside its `competition_loop` node behind no `<arg>`. The wrapper therefore
sets the parameter *before* its `<include>`, and roslaunch lets the included
file win. `roslaunch --dump-params` confirms the effective value is **600.0**.
Raising it would require adding a pass-through `<arg name="max_time"
default="600.0"/>` to that production launch — deliberately out of scope for
this freeze. The exploration result being frozen (4/4 zones +
`CURRENT_FLOOR_COMPLETE`) is carried by the run evidence, not by this watchdog.

## 6. Validated result (frozen evidence)

```text
FLOOR0_EXPLORATION_CURRENT_HEAD = PASS
ROOM_SCAN_ZONES_TRIGGERED       = 4/4
CURRENT_FLOOR_COMPLETE          = PASS

MANUAL_INTERVENTION_COUNT       = 0
GROUND_TRUTH_FEEDBACK_COUNT     = 0
```

Layout/danger truth is only consulted post-hoc by the referee-side evaluator;
no production node reads a truth channel.

## 7. Return status (NOT solved)

```text
RETURN_PATH_GENERATION  = PASS
RETURN_PATH_EXECUTION   = PARTIAL_PASS
RETURN_REACHABILITY     = FAIL

FIRST_FULL_NATURAL_FLOOR0_CLOSURE = NOT_YET_VALIDATED

KNOWN_BLOCKER =
main entrance connectivity cut in inflated map during RETURNING replanning
```

Do **not** describe this baseline as `FULL FLOOR0 PASS`.

## 8. KNOWN_SCENE_DEVIATIONS

Read-only audit of the baseline source tree at `SUPERPROJECT_BASE` plus the
generated scene. **Nothing was modified** by this audit.

### 8.1 Main entrance `entrance_apron`

Conclusion: **the `entrance_apron` is present and enabled in this baseline —
it has NOT been deleted and NOT been disabled.**

```text
file    = src/building_generator_core/building_generator_core/exporter.py
caller  = _build_static_shell_model() -> _append_entrance_apron(model, layout)   (line 142)
status  = invoked unconditionally; no flag, no scene-profile gate

size    = (4.5, 2.4, 0.08) m
pose    = (0.0, -1.2, -0.04, 0, 0, 0)     <- NOTE: z = -0.04, NOT +0.04
```

With centre z = -0.04 and height 0.08 the box spans z ∈ [-0.08, 0.00], i.e. its
**top face is coplanar with the Gazebo ground**, so there is no vertical step.
This was confirmed in the generated world file (read-only):
`generated_building/competition_scene.world` contains
`<pose>0.0000 -1.2000 -0.0400 0.0000 0.0000 0.0000</pose>` for link
`entrance_apron`.

Provenance of the pose change:

| Commit | Date | Author | Subject | Pose z |
|---|---|---|---|---|
| `6d2aa9ce60b78b78b046555b63d8c697a84080f2` | 2026-05-09 | guoyulun | 优化随机生成建筑 | `+0.04` |
| `25266f652bf3c3d10bfacc473efcf45b375637a2` | 2026-08-26 | Carl-Huang12138 | chore(baseline): freeze validated R26-R29 minimal loop | `-0.04` |

`25266f6` is an **ancestor** of `SUPERPROJECT_BASE` (verified with
`git merge-base --is-ancestor`), therefore this baseline carries the
`-0.04` (flush) variant. The originating commit's own comment records why:

> Keep the public entrance transition coplanar with the Gazebo ground. A box
> centred at +0.04 m creates an 8 cm vertical step that the validated
> flat-terrain A1 policy cannot traverse.

Corollary for downstream work: there is **no `main_entrance_threshold` object**
in this repository, but the main entrance does carry this `entrance_apron`. Any
future entry-threshold experiment must therefore reason about the 4.5 × 2.4 m,
8 cm thick apron whose top is flush with the ground, not about a missing
threshold object.

### 8.2 Door open geometry bug (audited, still present)

```text
KNOWN_DOOR_OPEN_GEOMETRY_BUG = true

intended opening = 2.0 m
authored open pose leaves = 1.12 m  (0.62 m after the planner's 0.25 m inflation)
```

Evidence:

- `reports/floor0_defect4_root_cause.md` §2 — `door_config.yaml` open poses put
  the panels at world x ∈ [0.56, 1.56] and [-1.56, -0.56], leaving a 1.12 m gap;
  the open pose also historically was never applied at runtime.
- `reports/floor0_door_runtime_repair.md` §6 — the gate's own `free width`
  metric still reports 0.1 m because the authored open pose deliberately leaves
  1.12 m of the 2.0 m opening for the panels.
- `reports/floor0_entry_timebox_result.md` — related note that 0.88 m of the
  2.0 m opening stayed blocked in the elevator-door case.

### 8.3 Net statement

```text
SCENE_GEOMETRY_MODIFIED_BY_THIS_BASELINE = false
```

This baseline changes **no scene geometry**. It repoints two submodules to
frozen commits and adds validation-only reproduction files under
`tools/baselines/floor0_4room_v1/`. The 4-room `PASS` therefore was achieved
*with* the apron present and *with* the 1.12 m door bug present — which is
exactly why this must not be mistaken for a competition-faithful final scene.

## 9. Reproducing this baseline

```bash
bash tools/baselines/floor0_4room_v1/run_floor0_4room_baseline.sh
```

The script verifies the checkout (base ancestry + both frozen submodule
commits + clean submodules), freezes the environment explicitly, waits for
`/use_sim_time=true`, an advancing `/clock`, robot spawn and
`RL_HANDOFF_READY=True` **read from this run's log only** (stale logs are
deleted first), then launches `tools/baselines/floor0_4room_v1/floor0_4room_replay.launch`.

See `README.md` in the same directory for teammate checkout commands.
