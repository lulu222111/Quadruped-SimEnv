#!/usr/bin/env python3
"""interface_validator.py — validate that the competition environment is compliant.

In competition mode this checks that forbidden topics are not being published
and that only the allowed scene file is read.

Validation uses explicit state flags rather than relying on an empty error list.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
from typing import List, Optional, Set

import rospy


class InterfaceValidator:
    """Start-up and runtime interface compliance checker.

    Explicit state flags track which checks have been completed and whether
    they passed.  is_ready() requires ALL mandatory checks to be completed
    AND passed — an empty error list alone is never sufficient.
    """

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

        # --- Explicit validation state flags ---
        self._scene_info_loaded: bool = False
        self._forbidden_topic_check_completed: bool = False
        self._forbidden_topic_check_passed: bool = False
        self._forbidden_file_check_completed: bool = False

    # ------------------------------------------------------------------
    # Explicit state accessors
    # ------------------------------------------------------------------

    @property
    def scene_info_loaded(self) -> bool:
        return self._scene_info_loaded

    @property
    def forbidden_topic_check_completed(self) -> bool:
        return self._forbidden_topic_check_completed

    @property
    def forbidden_topic_check_passed(self) -> bool:
        return self._forbidden_topic_check_passed

    @property
    def forbidden_file_check_completed(self) -> bool:
        return self._forbidden_file_check_completed

    @property
    def competition_mode(self) -> bool:
        return self._competition_mode

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
            self._scene_info_loaded = False
            return None

        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError) as exc:
            msg = f"Failed to parse {path}: {exc}"
            rospy.logerr(msg)
            self._validation_errors.append(msg)
            self._scene_info_loaded = False
            return None

        self._scene_info = data
        self._scene_info_loaded = True
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

        In test mode the check is explicitly recorded as completed and
        passed without querying the master.
        """
        if not self._competition_mode:
            rospy.loginfo("Test mode: skipping forbidden-topic check (recorded as passed).")
            self._forbidden_topic_check_completed = True
            self._forbidden_topic_check_passed = True
            return True

        if self._scene_info is None:
            rospy.logwarn("Scene info not loaded; cannot determine forbidden topics.")
            self._forbidden_topic_check_completed = True
            self._forbidden_topic_check_passed = False
            return False

        forbidden: List[str] = self._scene_info.get("referee_only", {}).get("forbidden_topics", [])
        if not forbidden:
            rospy.loginfo("No forbidden topics listed in team_scene_info.")
            self._forbidden_topic_check_completed = True
            self._forbidden_topic_check_passed = True
            return True

        rospy.loginfo("Checking for forbidden topics (timeout=%.1f s)...", timeout)

        # rospy.get_published_topics() returns [(topic_name, topic_type), ...]
        # We must extract only the topic names for correct membership tests.
        try:
            published: List[tuple] = rospy.get_published_topics(timeout)
        except rospy.ROSException as exc:
            msg = f"Failed to query published topics from ROS Master: {exc}"
            rospy.logerr(msg)
            # Do not append duplicate errors
            if msg not in self._validation_errors:
                self._validation_errors.append(msg)
            self._forbidden_topic_check_completed = True
            self._forbidden_topic_check_passed = False
            return False

        published_names: Set[str] = {topic_name for topic_name, _topic_type in published}

        ok = True
        for topic in forbidden:
            if topic in published_names:
                msg = (
                    f"FORBIDDEN TOPIC DETECTED: {topic} is being published. "
                    f"In competition mode this MUST NOT be available. "
                    f"Set ENABLE_GROUND_TRUTH=0 and ENABLE_REFEREE_ODOM=0."
                )
                rospy.logerr(msg)
                # Do not append duplicate errors
                if msg not in self._validation_errors:
                    self._validation_errors.append(msg)
                ok = False

        self._forbidden_topic_check_completed = True
        self._forbidden_topic_check_passed = ok

        if ok:
            rospy.loginfo("Forbidden topic check PASSED.")
        return ok

    # ------------------------------------------------------------------
    # Forbidden file check
    # ------------------------------------------------------------------

    def check_forbidden_files(self, workspace_root: Optional[str] = None) -> bool:
        """Verify that forbidden files are not present on disk.

        Forbidden files may be generated by the referee environment, so their
        mere presence is NOT automatically proof of an algorithm violation.

        Principles:
          - Algorithm code MUST NOT read these files.
          - In competition mode, file presence triggers a warning.
          - In test mode, file presence is logged at info level.
          - NEVER read file contents.
          - NEVER delete files automatically.
          - NEVER run evaluation scripts.
          - File existence alone does NOT block startup unless project
            rules explicitly require it.
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
                # Do not append duplicate warnings
                if msg not in self._validation_errors:
                    self._validation_errors.append(msg)

        self._forbidden_file_check_completed = True
        return ok

    # ------------------------------------------------------------------
    # Aggregate
    # ------------------------------------------------------------------

    def is_ready(self) -> bool:
        """Return True ONLY when all mandatory checks are completed AND passed.

        Readiness requires:
          1. team_scene_info.json loaded successfully
          2. Forbidden topic check completed AND passed
             (test mode: explicitly skipped → completed + passed)
          3. An empty error list alone is NOT sufficient.
        """
        if not self._scene_info_loaded:
            return False
        if not self._forbidden_topic_check_completed:
            return False
        if not self._forbidden_topic_check_passed:
            return False
        return True

    @property
    def errors(self) -> List[str]:
        return list(self._validation_errors)

    def report(self) -> str:
        lines = []
        if self._scene_info_loaded:
            lines.append("Scene info: LOADED")
        else:
            lines.append("Scene info: NOT LOADED")

        if self._forbidden_topic_check_completed:
            status = "PASSED" if self._forbidden_topic_check_passed else "FAILED"
            lines.append(f"Forbidden topic check: {status}")
        else:
            lines.append("Forbidden topic check: NOT EXECUTED")

        if self._validation_errors:
            lines.append("Errors:")
            for e in self._validation_errors:
                lines.append(f"  - {e}")
        else:
            lines.append("Errors: none")

        return "\n".join(lines)
