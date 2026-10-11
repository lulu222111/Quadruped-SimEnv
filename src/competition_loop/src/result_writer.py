#!/usr/bin/env python3
# =============================================================================
# 模块说明（中文）：result_writer.py —— detected_danger.json 结果写出
#
# 职责：收集通过门控的危险源候选，按官方 schema 写出 results/detected_danger.json。
# 为什么用"临时文件 + 原子替换"：评估程序可能在任何时刻读取结果文件，必须保证
#       读到的要么是完整旧版本、要么是完整新版本，绝不能看到写了一半的 JSON。
#       因此先在同目录 mkstemp 写全并 flush，再用 os.replace 原子换名；异常时
#       清理临时文件。
# 关键约定：
#   1) 只接受 frame_id == "world" 的候选（世界坐标变换由 router 负责），
#      位置必须是有限数值；不满足即拒绝。
#   2) 本模块绝不读取 danger_truth.json 或任何评估脚本/真值文件，输出格式即
#      官方 schema（exploration_time + detected_danger_sources[].position）。
# 结构导航：ResultWriter（行 23 起）→ accept_candidate（行 57-84，三重校验）
#       → write（行 101-134，原子写）→ _build_output（行 136-152，输出格式唯一
#       权威，可选调用聚合器生成最终源列表）。
# 输入：已被 router 接受的候选、exploration_time
# 输出：results/detected_danger.json
# =============================================================================
"""result_writer.py — write detected_danger.json with atomic file replacement.

This module only accepts HazardCandidates whose frame_id is "world" and
whose position_valid flag is True.

It MUST NOT read danger_truth.json, call the evaluation script, or
access any forbidden files.
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
    """

    REQUIRED_FRAME = "world"

    def __init__(self, output_path: Optional[str] = None,
                 aggregate_fn=None):
        if output_path:
            self._output_path = Path(output_path)
        else:
            self._output_path = Path("results/detected_danger.json")

        self._candidates: List[HazardCandidate] = []
        self._exploration_time: float = 0.0
        # Optional final write-time aggregator (P0-2): a callable that maps the
        # full accepted-candidate list to the source list that is actually
        # written. Kept optional so the writer stays a thin, pure collector by
        # default (all existing tests keep their semantics).
        self._aggregate_fn = aggregate_fn

    # ------------------------------------------------------------------
    # Ingestion
    # ------------------------------------------------------------------

    def accept_candidate(self, candidate: HazardCandidate) -> bool:
        """Accept a candidate if it passes validation.

        Returns True if accepted, False if rejected.
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
        sources = self._candidates
        if self._aggregate_fn is not None:
            sources = self._aggregate_fn(self._candidates)
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
                for c in sources
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
