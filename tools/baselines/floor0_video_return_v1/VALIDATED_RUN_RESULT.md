# VIDEO RUN #4 — PASS (full natural closure, recorded take)

VIDEO_BASELINE = 3503f44e617385c56f264875575d8ca8b68c3b36
                 + src/competition_planning/scripts/planning_node = snapshot `4U1v`
                   sha256 76277c39266634ac1b9204ac209bc55f7b192f282f5774168c19e953e9ed2ee8
                 (byte-identical to the double-PASS source state of RUN #1 / #3)

```text
RUN_INDEX               = 4
START_TIME             = 2026-09-14 15:38:48 (+08:00)
SEED / FLOOR / ROOMS   = 77 / 1 / 4
MAX_TIME (live)        = 1800.0 s

RGBD_GATE              = PASS (rgb/depth 704, camera_info 115, advancing)
MAX_TIME_GATE          = PASS
GT_COMPLIANCE_GATE     = PASS
GUI_GATE               = PASS (gzserver=2 / gzclient=2 on DISPLAY=:1, no OGRE exception)

CURRENT_FLOOR_COMPLETE = 16:13:49
RETURNING              = 16:13:49   (return path 236 poses, arrival goal reported reachable)
entrance outward cross = ~sim 790
ARRIVED_START          = 16:23:24   (distance = 0.090 m, samples = 3)
FINISHED               = 16:23:24   (elapsed = 783.50 s)
MISSION_SIM_TIME       = 783.5 s

HAZARDS                = 6/6 unique truth sources matched (1.0 m tolerance)
FALSE_ALARMS           = 0
MAX_TIME_REACHED       = false

CONTACT_MARKS          = 2/12, all recovered  (142,204) (143,229) - room_1 wall sweep
SKIPS                  = 0     UNREACHABLE = 0     BUDGET_EXHAUSTED = no

GROUND_TRUTH_FEEDBACK_COUNT = 0
MANUAL_INTERVENTION_COUNT   = 0

RESULT                 = PASS
```

## Notes

* Fastest of the four takes (783.5 s), cleanest return (236-pose route with a
  reachable arrival goal, entrance crossing without any recovery event).
* GUI survived the whole run; nothing was stopped after FINISHED - the
  simulation, GUI and rosbag recorder were kept alive for owner recording.
* Evidence: watcher.tsv / watcher_events.log, competition_stack.log, per-node
  roslogs, params.txt, gt_compliance.txt, run.bag, results/detected_danger.json
  (6 entries) vs results/danger_truth.json (6 sources).
