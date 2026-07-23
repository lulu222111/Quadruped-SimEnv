#!/usr/bin/env python3
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
