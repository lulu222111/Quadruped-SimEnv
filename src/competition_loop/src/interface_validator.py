#!/usr/bin/env python3
"""interface_validator.py — validate that the competition environment is compliant.

In competition mode this checks that forbidden topics are not being published
and that only the allowed scene file is read.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Optional

import rospy


class InterfaceValidator:
    """Start-up and runtime interface compliance checker."""

    # The single file the competition algorithm is allowed to read.
    ALLOWED_SCENE_FILE = "generated_building/team_scene_info.json"

    # Forbidden files the algorithm MUST NOT read.
    FORBIDDEN_FILES = [
        "results/danger_truth.json",
        "generated_building/danger_truth.json",
        "generated_building/layout_metadata.json",
        "generated_building/competition_scene.world",
        "generated_building/building_config.json",
        "generated_building/scene_manifest.json",
    ]

    def __init__(self, competition_mode: bool = True):
        self._competition_mode = competition_mode
        self._scene_info: Optional[dict] = None
        self._validation_errors: List[str] = []

    # ------------------------------------------------------------------
    # Scene info
    # ------------------------------------------------------------------

    def load_scene_info(self, workspace_root: Optional[str] = None) -> Optional[dict]:
        """Load the single allowed scene info file.

        Returns the parsed JSON dict, or None on failure.
        Stores the result for later checks.
        """
        root = Path(workspace_root) if workspace_root else Path.cwd()
        path = root / self.ALLOWED_SCENE_FILE
        if not path.is_file():
            msg = f"Allowed scene file not found: {path}"
            rospy.logerr(msg)
            self._validation_errors.append(msg)
            return None

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as exc:
            msg = f"Failed to parse {path}: {exc}"
            rospy.logerr(msg)
            self._validation_errors.append(msg)
            return None

        self._scene_info = data
        rospy.loginfo("Loaded team_scene_info.json (schema: %s)", data.get("schema", "unknown"))
        return data

    # ------------------------------------------------------------------
    # Forbidden topic check (competition mode only)
    # ------------------------------------------------------------------

    def check_forbidden_topics(self, timeout: float = 3.0) -> bool:
        """Verify that no forbidden topics are currently being published.

        Returns True if the environment is clean, False otherwise.
        Does NOT subscribe to the forbidden topics — only inspects the
        ROS master topic list.
        """
        if not self._competition_mode:
            rospy.loginfo("Test mode: skipping forbidden-topic check.")
            return True

        if self._scene_info is None:
            rospy.logwarn("Scene info not loaded; cannot determine forbidden topics.")
            return False

        forbidden = self._scene_info.get("referee_only", {}).get("forbidden_topics", [])
        if not forbidden:
            rospy.loginfo("No forbidden topics listed in team_scene_info.")
            return True

        rospy.loginfo("Checking for forbidden topics (timeout=%.1f s)...", timeout)
        try:
            published = set(rospy.get_published_topics(timeout))
        except rospy.ROSException as exc:
            msg = f"Failed to query published topics: {exc}"
            rospy.logerr(msg)
            self._validation_errors.append(msg)
            return False

        ok = True
        for topic in forbidden:
            if topic in published:
                msg = (
                    f"FORBIDDEN TOPIC DETECTED: {topic} is being published. "
                    f"In competition mode this MUST NOT be available. "
                    f"Set ENABLE_GROUND_TRUTH=0 and ENABLE_REFEREE_ODOM=0."
                )
                rospy.logerr(msg)
                self._validation_errors.append(msg)
                ok = False

        if ok:
            rospy.loginfo("Forbidden topic check PASSED.")
        return ok

    # ------------------------------------------------------------------
    # Forbidden file check
    # ------------------------------------------------------------------

    def check_forbidden_files(self, workspace_root: Optional[str] = None) -> bool:
        """Verify that forbidden files are not present on disk.

        Their mere presence is not a violation in test mode, but in
        competition mode we warn because the algorithm could read them.
        """
        root = Path(workspace_root) if workspace_root else Path.cwd()
        ok = True
        for rel in self.FORBIDDEN_FILES:
            if (root / rel).exists():
                msg = (
                    f"Forbidden file present: {rel}. "
                    f"Algorithm code MUST NOT read it."
                )
                if self._competition_mode:
                    rospy.logwarn(msg)
                else:
                    rospy.loginfo("Test mode: %s (allowed for debugging)", msg)
        return ok

    # ------------------------------------------------------------------
    # Aggregate
    # ------------------------------------------------------------------

    def is_ready(self) -> bool:
        """Return True if all validation checks have passed."""
        return len(self._validation_errors) == 0

    @property
    def errors(self) -> List[str]:
        return list(self._validation_errors)

    def report(self) -> str:
        if not self._validation_errors:
            return "Interface validation: ALL CHECKS PASSED"
        lines = ["Interface validation: FAILURES DETECTED"]
        for e in self._validation_errors:
            lines.append(f"  - {e}")
        return "\n".join(lines)
