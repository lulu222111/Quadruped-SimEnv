#!/usr/bin/env python3
"""result_writer.py — write detected_danger.json with atomic file replacement.

This module only accepts HazardCandidates whose frame_id is "world" and
whose position_valid flag is True.

It MUST NOT read danger_truth.json, call the evaluation script, or
access any forbidden files.

中文说明：
    本模块负责生成比赛的最终交付物 detected_danger.json，是评测系统直接读取的
    唯一结果文件，因此输出 schema 必须严格保持官方格式，不得增删字段。

    写入策略：
        先在同目录写临时文件，再用 os.replace 原子替换目标文件。评测/人查看
        结果时永远看到完整 JSON，不会读到写了一半的内容。

    合规约束：
        不读 danger_truth.json、不调用评测脚本、不访问任何禁止文件；
        只接受 frame_id == "world" 且 position_valid == true 的候选。
"""

from __future__ import annotations

import json
import os
import tempfile
from pathlib import Path
from typing import Dict, List, Optional

import rospy
from competition_interfaces.msg import HazardCandidate


class ResultWriter:
    """Collect and write competition detection results.

    Output format (strict):
        {
          "exploration_time": <float>,    // seconds
          "detected_danger_sources": [
            {"position": [x, y, z]},
            ...
          ]
        }

    中文：收集已确认的危险源位置并在结束时写出。输出只包含
    exploration_time 与 detected_danger_sources 两个顶层字段。

    注意：内部 registry 会按楼层关联危险源以避免跨层误去重，但该楼层信息
    只存在于内存，不写入 detected_danger.json —— 官方评测 schema 中没有
    floor 字段，擅自增加会破坏格式兼容性。
    """

    REQUIRED_FRAME = "world"

    def __init__(self, output_path: Optional[str] = None):
        if output_path:
            self._output_path = Path(output_path)
        else:
            self._output_path = Path("results/detected_danger.json")

        self._candidates: List[HazardCandidate] = []
        self._exploration_time: float = 0.0

    # ------------------------------------------------------------------
    # Ingestion
    # ------------------------------------------------------------------

    def accept_candidate(self, candidate: HazardCandidate) -> bool:
        """Accept a candidate if it passes validation.

        Returns True if accepted, False if rejected.

        中文：三重校验（坐标系 / position_valid / 数值有限）全部通过才入列；
        任一项不满足只记 warning 并拒绝，不抛异常中断任务。
        """
        # 1. Frame check
        if candidate.header.frame_id != self.REQUIRED_FRAME:
            rospy.logwarn(
                "Rejected candidate: frame_id='%s' (expected '%s')",
                candidate.header.frame_id,
                self.REQUIRED_FRAME,
            )
            return False

        # 2. position_valid check
        if not candidate.position_valid:
            rospy.logwarn("Rejected candidate: position_valid=false")
            return False

        # 3. NaN / Inf check
        x, y, z = candidate.position.x, candidate.position.y, candidate.position.z
        if not (self._is_finite(x) and self._is_finite(y) and self._is_finite(z)):
            rospy.logwarn("Rejected candidate: non-finite position [%.3f, %.3f, %.3f]", x, y, z)
            return False

        self._candidates.append(candidate)
        rospy.loginfo("Accepted candidate at [%.3f, %.3f, %.3f]", x, y, z)
        return True

    def accept_candidates(self, candidates: List[HazardCandidate]) -> int:
        """Accept a batch of candidates. Returns the number accepted."""
        accepted = 0
        for c in candidates:
            if self.accept_candidate(c):
                accepted += 1
        return accepted

    def set_exploration_time(self, seconds: float) -> None:
        self._exploration_time = float(seconds)

    # ------------------------------------------------------------------
    # Writing
    # ------------------------------------------------------------------

    def write(self) -> Path:
        """Write results using temp file + atomic rename.

        Returns the path to the written file.

        中文：临时文件 + 原子 rename；写失败时清理临时文件后向上抛出，
        避免留下半成品结果文件。
        """
        data = self._build_output()
        self._output_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to a temp file in the same directory, then atomically rename
        fd, tmp_path = tempfile.mkstemp(
            suffix=".json",
            prefix="detected_danger_",
            dir=str(self._output_path.parent),
        )
        try:
            with os.fdopen(fd, "w", encoding="utf-8") as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
                f.write("\n")
            os.replace(tmp_path, str(self._output_path))
        except Exception:
            # Clean up temp file on failure
            try:
                os.unlink(tmp_path)
            except OSError:
                pass
            raise

        rospy.loginfo(
            "Results written to %s (%d candidates, time=%.2f s)",
            self._output_path,
            len(self._candidates),
            self._exploration_time,
        )
        return self._output_path

    def _build_output(self) -> dict:
        return {
            "exploration_time": self._exploration_time,
            "detected_danger_sources": [
                {
                    "position": [
                        float(c.position.x),
                        float(c.position.y),
                        float(c.position.z),
                    ]
                }
                for c in self._candidates
            ],
        }

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _is_finite(value: float) -> bool:
        import math
        return math.isfinite(value)

    @property
    def candidate_count(self) -> int:
        return len(self._candidates)

    def clear(self) -> None:
        self._candidates.clear()
        self._exploration_time = 0.0
