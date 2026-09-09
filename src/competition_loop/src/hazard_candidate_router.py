#!/usr/bin/env python3
"""Validation, transformation handoff, and distance de-duplication helpers."""

from __future__ import annotations

import math
from typing import Callable, Iterable, Optional


class CandidateTransformError(Exception):
    """Raised when a candidate cannot be transformed into world coordinates."""


class HazardCandidateRouter:
    """Route validated world-frame candidates to a ResultWriter-like object."""

    def __init__(
        self,
        writer,
        dedup_distance: float = 0.30,
        max_age: float = 5.0,
        future_tolerance: float = 0.50,
        now_fn: Optional[Callable[[], float]] = None,
        log_fn: Optional[Callable[[str], None]] = None,
        allowed_labels: Optional[Iterable[str]] = None,
    ) -> None:
        if not math.isfinite(dedup_distance) or dedup_distance <= 0.0:
            raise ValueError("dedup_distance must be finite and > 0")
        if not math.isfinite(max_age) or max_age <= 0.0:
            raise ValueError("max_age must be finite and > 0")
        if not math.isfinite(future_tolerance) or future_tolerance < 0.0:
            raise ValueError("future_tolerance must be finite and >= 0")
        self.writer = writer
        self.dedup_distance = float(dedup_distance)
        self.max_age = float(max_age)
        self.future_tolerance = float(future_tolerance)
        self.now_fn = now_fn
        self.log_fn = log_fn
        # Optional semantic whitelist (P0-1): when set, only candidates whose
        # label is in this set pass. The only referee-counted danger source is
        # the red sphere; green_sphere / red_box are distractor sources and
        # must never reach the final result (each false report costs far score).
        if allowed_labels is None:
            self._allowed_labels = None
        else:
            self._allowed_labels = set(str(label) for label in allowed_labels)
        self._accepted_positions = []
        self.last_stats = {}

    @staticmethod
    def _stamp_seconds(stamp) -> Optional[float]:
        try:
            value = float(stamp.to_sec())
        except (AttributeError, TypeError, ValueError):
            return None
        return value if math.isfinite(value) and value > 0.0 else None

    @staticmethod
    def _finite_position(candidate) -> bool:
        try:
            values = (
                float(candidate.position.x),
                float(candidate.position.y),
                float(candidate.position.z),
            )
        except (AttributeError, TypeError, ValueError):
            return False
        return all(math.isfinite(value) for value in values)

    @staticmethod
    def _distance_squared(candidate, position) -> float:
        dx = float(candidate.position.x) - position[0]
        dy = float(candidate.position.y) - position[1]
        dz = float(candidate.position.z) - position[2]
        return dx * dx + dy * dy + dz * dz

    def _reject(self, reason: str) -> None:
        self.last_stats[reason] = self.last_stats.get(reason, 0) + 1
        if self.log_fn is not None:
            self.log_fn(reason)

    def process(
        self,
        candidates: Iterable,
        transform_fn: Callable,
    ) -> int:
        """Validate, transform, de-duplicate, and write candidates."""
        self.last_stats = {
            "received": 0,
            "accepted": 0,
            "invalid_position": 0,
            "label_not_allowed": 0,
            "invalid_timestamp": 0,
            "stale_timestamp": 0,
            "future_timestamp": 0,
            "transform_failed": 0,
            "duplicate": 0,
            "writer_rejected": 0,
        }
        now = self.now_fn() if self.now_fn is not None else None
        for candidate in candidates:
            self.last_stats["received"] += 1
            # P0-1: semantic whitelist gate (distractor sources never pass).
            if self._allowed_labels is not None and (
                getattr(candidate, "label", None) not in self._allowed_labels
            ):
                self._reject("label_not_allowed")
                continue
            if not getattr(candidate, "position_valid", False):
                self._reject("invalid_position")
                continue
            if not self._finite_position(candidate):
                self._reject("invalid_position")
                continue
            header = getattr(candidate, "header", None)
            stamp = self._stamp_seconds(
                getattr(header, "stamp", None)
            )
            if stamp is None:
                self._reject("invalid_timestamp")
                continue
            if now is not None and math.isfinite(now) and now > 0.0:
                age = now - stamp
                if age > self.max_age:
                    self._reject("stale_timestamp")
                    continue
                if age < -self.future_tolerance:
                    self._reject("future_timestamp")
                    continue
            try:
                world_candidate = transform_fn(candidate)
            except Exception as exc:
                self._reject("transform_failed")
                if self.log_fn is not None:
                    self.log_fn("candidate transform failed: {}".format(exc))
                continue
            if not self._finite_position(world_candidate):
                self._reject("invalid_position")
                continue
            position = (
                float(world_candidate.position.x),
                float(world_candidate.position.y),
                float(world_candidate.position.z),
            )
            if any(
                self._distance_squared(world_candidate, existing)
                <= self.dedup_distance * self.dedup_distance
                for existing in self._accepted_positions
            ):
                self._reject("duplicate")
                continue
            if not self.writer.accept_candidate(world_candidate):
                self._reject("writer_rejected")
                continue
            self._accepted_positions.append(position)
            self.last_stats["accepted"] += 1
        return self.last_stats["accepted"]

    def clear(self) -> None:
        """Clear the in-memory de-duplication state for a new run."""
        self._accepted_positions.clear()
