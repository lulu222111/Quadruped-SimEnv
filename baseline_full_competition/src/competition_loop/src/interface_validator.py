#!/usr/bin/env python3
# =============================================================================
# 中文说明（模块级）：interface_validator.py —— 启动/运行时接口合规校验
#
# 职责：校验算法只通过公开接口工作——加载并严格校验唯一允许的场景文件
#       （team_scene_info.json）、检查禁止话题是否被发布、检查禁止文件是否存在。
# 为什么这样设计：竞赛模式禁止消费真值/裁判通道（/Odometry_gazebo、
#       /ground_truth/*、/gazebo/model_states）与非公开文件（layout_metadata.json、
#       building_config.json、scene_manifest.json、danger_truth.json、
#       door_config.yaml、elevator_config.yaml 等）。
# 就绪语义：必须用显式的 completed/passed 状态标志判定，而不是"错误列表为空"
#       ——ROS Master 查询失败属于瞬态，不能据此认为检查通过，必须可重试。
#       禁止文件检查只做存在性告警，绝不读内容、绝不删文件。
# =============================================================================
"""interface_validator.py — validate that the competition environment is compliant.

In competition mode this checks that forbidden topics are not being published
and that only the allowed scene file is read.

Validation uses explicit state flags rather than relying on an empty error list.
"""

from __future__ import annotations

import json
import math
import os
from pathlib import Path
from typing import Any, List, Optional, Set

import rospy


class InterfaceValidator:
    """Start-up and runtime interface compliance checker.

    Explicit state flags track which checks have been completed and whether
    they passed.  is_ready() requires ALL mandatory checks to be completed
    AND passed — an empty error list alone is never sufficient.

    Warnings (e.g. forbidden file presence) are tracked separately from
    errors and do NOT affect is_ready().
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
        self._validation_warnings: List[str] = []

        # --- Explicit validation state flags ---
        self._scene_info_loaded: bool = False
        self._forbidden_topic_check_completed: bool = False
        self._forbidden_topic_check_passed: bool = False
        self._forbidden_file_check_completed: bool = False

        # Retry tracking for Master query failures
        self._next_topic_retry_time: Optional[rospy.Time] = None
        self._topic_retry_interval = rospy.Duration.from_sec(2.0)  # 2 s between retries

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

        The root of the JSON document MUST be a dict (object).  Lists,
        strings, numbers, and null are rejected.
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

        if not isinstance(data, dict):
            msg = (
                f"Root of {path} is not a JSON object (got {type(data).__name__}). "
                f"Expected a dict with 'schema' and 'referee_only' keys."
            )
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

    def check_forbidden_topics(self) -> bool:
        """Verify that no forbidden topics are currently being published.

        Returns True if the environment is clean, False otherwise.
        Does NOT subscribe to the forbidden topics — only inspects the
        ROS master topic list via get_published_topics().

        In test mode the check is explicitly recorded as completed and
        passed without querying the master.

        ROS Master query failures are treated as transient: the check
        remains incomplete so it will be retried on the next call.
        Only a confirmed detection of forbidden topics sets the check
        to completed + failed permanently.

        NOTE: rospy.get_published_topics(namespace='/') takes a string
        namespace, NOT a float timeout.  The real signature is:
            def get_published_topics(namespace='/'):
        It returns [[topic_name, topic_type], ...].
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

        # Respect retry interval if a previous Master query failed
        if self._next_topic_retry_time is not None:
            if rospy.Time.now() < self._next_topic_retry_time:
                # Not yet time to retry — keep check incomplete so the
                # caller will ask again later
                return False
            self._next_topic_retry_time = None

        rospy.loginfo("Checking for forbidden topics via ROS Master...")

        # rospy.get_published_topics(namespace='/') is the correct API.
        # It takes a string namespace (default '/'), NOT a float timeout.
        # Returns [[topic_name, topic_type], ...].
        try:
            published: List[list] = rospy.get_published_topics()
        except rospy.ROSException as exc:
            msg = f"Failed to query published topics from ROS Master: {exc}"
            rospy.logwarn_throttle(10.0, msg)
            # Do NOT set completed=True — this is a transient error,
            # let the caller retry on the next tick.
            # Schedule next retry after the interval.
            self._next_topic_retry_time = rospy.Time.now() + self._topic_retry_interval
            # Do not append duplicate errors
            if msg not in self._validation_errors:
                self._validation_errors.append(msg)
            return False

        # Extract topic names from [[topic_name, topic_type], ...]
        published_names: Set[str] = {item[0] for item in published}

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

        # On confirmed detection or confirmed clean, mark completed
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
          - In competition mode, file presence triggers a WARNING (not error).
          - In test mode, file presence is logged at info level.
          - NEVER read file contents.
          - NEVER delete files automatically.
          - NEVER run evaluation scripts.
          - File existence alone does NOT affect is_ready().
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
                # Store as warning, NOT error — file presence alone does not
                # prove the algorithm read it.
                if msg not in self._validation_warnings:
                    self._validation_warnings.append(msg)

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

    @property
    def warnings(self) -> List[str]:
        return list(self._validation_warnings)

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

        if self._validation_warnings:
            lines.append("Warnings:")
            for w in self._validation_warnings:
                lines.append(f"  - {w}")

        return "\n".join(lines)
