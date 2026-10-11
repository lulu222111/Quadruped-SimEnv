# competition_recovery_testing

This package is an integration/smoke harness for one exceptional operation:
recovering a fallen Gazebo A1 by teleporting it back to the test spawn pose.
It is deliberately not included by `competition_system.launch` and must never
be used by the production DUT.

The lifecycle is fail-safe and ordered:

1. call the authoritative `/competition/finish` safety path (or verify an
   already-safe WAITING test system);
2. observe `control_enabled=false`, `emergency_stop=true`, then consecutive
   zero `/cmd_vel` samples;
3. stop the owned controller and the owned pose-derived DUT process group;
4. call `/gazebo/set_model_state`;
5. restart the controller with motion still gated and wait for fresh IMU/cloud;
6. restart the DUT with `auto_start:=false`;
7. require fresh raw FAST-LIO odom, Localization valid+odom, Mapping map, and
   Planning non-empty path from the new generation;
8. only then call `/competition/start` and observe control re-enabled.

On any timeout or stale-generation evidence, the coordinator leaves the robot
stopped.  It manages only process groups that it created itself; there is no
`pkill`, `killall`, `rosnode kill -a`, or container-wide cleanup.

Gazebo model state is never read by this package.  External audit collectors
may independently read ground truth, but no ground-truth value enters the DUT.

The executables are intentionally omitted from production launch files.  Run
`teleport_recovery_coordinator --help` inside an isolated smoke container for
its explicit process commands and readiness options.

## C2R7 straight-segment qualification

`straight_segment_path_source` is a second, independent test-only executable.
It publishes one three-waypoint world-frame path, aligned with the first fresh
valid `world/base` competition-localization sample, through the existing
planning output topics.  It never publishes a velocity command and never reads
Gazebo or ground-truth pose data.

The executable refuses to start unless all three safety conditions are
explicitly satisfied:

- `/competition_loop/competition_mode=false`;
- private `~test_only_acknowledged=true`;
- process environment `ENABLE_GROUND_TRUTH=false`.

The test launch defaults the acknowledgement to false.  Its explicit test run
form is:

```bash
ENABLE_GROUND_TRUTH=false roslaunch competition_recovery_testing \
  c2r7_straight_segment.launch test_only_acknowledged:=true
```

The launch disables only the production planner process, bounds the existing
follower's angular command and path timeout, and preserves the normal follower
to guard to controller chain.  Canonical `competition_system.launch` defaults
remain planner-enabled with the follower's existing parameter defaults.
