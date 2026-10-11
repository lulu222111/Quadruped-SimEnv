#!/usr/bin/env python3
"""Low-frequency, event-driven run watcher (task spec §6, §12, §13).

Writes a periodic TSV status line and an event log for CHANGES only, so the
stdout stays quiet.  Read-only: subscribes only, never publishes.

Granularity is deliberately raised once the mission enters RETURNING (§13).
"""
import os
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock

try:
    from competition_interfaces.msg import (
        CompetitionState, ModuleStatus, HazardCandidateArray)
except Exception:
    CompetitionState = ModuleStatus = HazardCandidateArray = None

OUT_DIR = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
TICK_SEC = float(sys.argv[2]) if len(sys.argv) > 2 else 10.0

STATE_NAMES = {0: "WAITING", 1: "RUNNING", 2: "RETURNING", 3: "FINISHED", 4: "ERROR"}


class W:
    def __init__(self):
        self.sim = None
        self.state = None
        self.detail = ""
        self.odom = None
        self.plan = None
        self.follow = None
        self.map_stamp = None
        self.detected = 0
        self.cand = 0
        self.cand_uniq = set()
        self.prev = {}
        self.t0 = time.time()

    # --- callbacks --------------------------------------------------------
    def cb_clock(self, m):
        self.sim = m.clock.to_sec()

    def cb_state(self, m):
        self.state = m.state
        self.detail = getattr(m, "detail", "")
        self.detected = getattr(m, "detected_count", self.detected)

    def cb_odom(self, m):
        p = m.pose.pose.position
        self.odom = (p.x, p.y)

    def cb_plan(self, m):
        self.plan = (m.state, m.valid, getattr(m, "detail", ""))

    def cb_follow(self, m):
        self.follow = (m.state, m.valid, getattr(m, "detail", ""))

    def cb_map(self, m):
        self.map_stamp = m.header.stamp.to_sec()

    def cb_cand(self, m):
        n = len(getattr(m, "candidates", []))
        self.cand += n
        for c in getattr(m, "candidates", []):
            try:
                p = c.position
                self.cand_uniq.add((round(p.x, 1), round(p.y, 1)))
            except Exception:
                pass


def main():
    global TICK_SEC  # reassigned to a finer tick once RETURNING begins (§13)
    rospy.init_node("doorfix_watcher", anonymous=True, disable_signals=True)
    w = W()

    rospy.Subscriber("/clock", Clock, w.cb_clock)
    if CompetitionState:
        rospy.Subscriber("/competition/state", CompetitionState, w.cb_state)
    if ModuleStatus:
        rospy.Subscriber("/competition/planning/status", ModuleStatus, w.cb_plan)
        rospy.Subscriber("/competition/control/path_follower_status", ModuleStatus,
                         w.cb_follow)
    if HazardCandidateArray:
        rospy.Subscriber("/competition/hazard_candidates", HazardCandidateArray,
                         w.cb_cand)
    from nav_msgs.msg import OccupancyGrid
    rospy.Subscriber("/competition/map", OccupancyGrid, w.cb_map)
    rospy.Subscriber("/competition/localization/odom", Odometry, w.cb_odom)

    tsv_path = os.path.join(OUT_DIR, "watcher.tsv")
    ev_path = os.path.join(OUT_DIR, "watcher_events.log")
    tsv = open(tsv_path, "a", buffering=1)
    ev = open(ev_path, "a", buffering=1)
    tsv.write("sim\twall\tfsm\tdetail\tloc_x\tloc_y\tplan_state\tplan_valid\t"
              "plan_detail\tfollow_detail\tdetected\tcand\tuniq\n")

    def emit(tag, msg):
        ev.write("%s | sim=%s | %s | %s\n" % (
            time.strftime("%H:%M:%S"), ("%.1f" % w.sim) if w.sim else "?",
            tag, msg))

    emit("WATCHER_START", "tick=%.1fs" % TICK_SEC)
    print("[watcher] started -> %s" % tsv_path, flush=True)

    returning_seen = False
    while not rospy.is_shutdown():
        rospy.sleep(TICK_SEC)
        sn = STATE_NAMES.get(w.state, str(w.state))
        det = w.detail or ""
        pd = (w.plan[2] if w.plan else "")[:80]
        fd = (w.follow[2] if w.follow else "")[:60]
        lx, ly = (w.odom if w.odom else (float("nan"), float("nan")))
        tsv.write("%.1f\t%.0f\t%s\t%s\t%.3f\t%.3f\t%s\t%s\t%s\t%s\t%d\t%d\t%d\n" % (
            w.sim or 0.0, time.time() - w.t0, sn, det.replace("\t", " "),
            lx, ly, (w.plan[0] if w.plan else ""), (w.plan[1] if w.plan else ""),
            pd.replace("\t", " "), fd.replace("\t", " "),
            w.detected, w.cand, len(w.cand_uniq)))

        # --- event log: only on change (keeps stdout/disk quiet) ----------
        for key, val in (("fsm", sn), ("detail", det),
                         ("plan", (w.plan[0], w.plan[1], pd) if w.plan else None),
                         ("follow", fd),
                         ("detected", w.detected),
                         ("uniq_cand", len(w.cand_uniq)),
                         ("map_stamp", w.map_stamp)):
            if w.prev.get(key) != val:
                w.prev[key] = val
                emit(key.upper(), str(val))

        if sn == "RETURNING" and not returning_seen:
            returning_seen = True
            emit("RETURN_START",
                 "sim=%.1f pose=(%.2f,%.2f)" % (w.sim or 0.0, lx, ly))
            TICK_SEC = 5.0  # §13: finer granularity after RETURNING

        if sn in ("FINISHED", "ERROR"):
            emit("TERMINAL", "%s detail=%s sim=%.1f" % (sn, det, w.sim or 0.0))
            break

    emit("WATCHER_STOP", "sim=%s" % (("%.1f" % w.sim) if w.sim else "?"))
    tsv.close()
    ev.close()


if __name__ == "__main__":
    main()
