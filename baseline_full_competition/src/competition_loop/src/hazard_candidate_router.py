#!/usr/bin/env python3
"""Validation, transformation handoff, and distance de-duplication helpers.

中文说明：
    本模块是感知结果进入比赛输出前的“过滤器 + 全局去重器”。感知只负责产生
    观测候选（HazardCandidate），是否可信、是否与已有危险源重复由这里判定。

    数据流：
        perception HazardCandidateArray
          -> 逐条校验（position_valid / 有限值 / 时间戳新鲜度）
          -> transform_fn 转换到 world 坐标系
          -> 按楼层做距离去重
          -> ResultWriter.accept_candidate -> detected_danger.json

    去重语义（重要）：
        危险源注册表是“全局”的，已确认位置永不因切楼层而丢弃；
        但去重比较只在当前楼层范围内进行 —— 不同楼层的相同 (x, y)
        是两个不同的危险源。这就是 set_floor() 只切换作用域而不清空历史的原因。
"""

from __future__ import annotations

import math
from typing import Callable, Iterable, Optional


class CandidateTransformError(Exception):
    """Raised when a candidate cannot be transformed into world coordinates."""


class HazardCandidateRouter:
    """Route validated world-frame candidates to a ResultWriter-like object.

    中文：把通过校验的 world 坐标系候选交给 writer，并维护跨楼层全局注册表。
    """

    def __init__(
        self,
        writer,
        dedup_distance: float = 0.30,
        max_age: float = 5.0,
        future_tolerance: float = 0.50,
        now_fn: Optional[Callable[[], float]] = None,
        log_fn: Optional[Callable[[str], None]] = None,
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
        # Global hazard registry: accepted world positions grouped per floor
        # so a floor change never discards earlier detections, while
        # de-duplication stays floor-local (the same xy on another floor is
        # a different hazard).
        # 中文：全局危险源注册表按楼层分组保存已确认的世界坐标。切楼层只改变
        # 后续去重的作用域，不清空历史 —— 否则跨楼层会丢失已完成楼层的战果，
        # 最终 detected_danger.json 会缺失已发现的目标。
        self.current_floor = 0
        self._accepted_positions = []
        self._floor_positions = {}
        self.last_stats = {}

    def set_floor(self, floor_id: int) -> None:
        """Associate subsequent candidates with ``floor_id``.

        Keeps every previously accepted position (global registry); only the
        active de-duplication scope changes.

        中文：设置后续候选所属楼层。仅切换去重作用域，历史确认结果全部保留。
        """
        floor = int(floor_id)
        if floor < 0:
            raise ValueError("floor_id must be >= 0")
        self.current_floor = floor

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
        """Validate, transform, de-duplicate, and write candidates.

        中文：单条候选的处理顺序固定为“校验 -> 坐标变换 -> 楼层去重 -> 写入”，
        任一步失败只丢弃当前候选并计入 last_stats，不影响其余候选。
        """
        self.last_stats = {
            "received": 0,
            "accepted": 0,
            "invalid_position": 0,
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
            # 中文：去重只在当前楼层的已确认位置上比较，欧氏距离小于
            # dedup_distance 视为同一危险源的重复观测。
            floor_scope = self._floor_positions.setdefault(
                self.current_floor, []
            )
            if any(
                self._distance_squared(world_candidate, existing)
                <= self.dedup_distance * self.dedup_distance
                for existing in floor_scope
            ):
                self._reject("duplicate")
                continue
            if not self.writer.accept_candidate(world_candidate):
                self._reject("writer_rejected")
                continue
            self._accepted_positions.append(position)
            floor_scope.append(position)
            self.last_stats["accepted"] += 1
        return self.last_stats["accepted"]

    def clear(self) -> None:
        """Clear the in-memory de-duplication state for a new run.

        中文：仅在开始全新一次任务时调用；任务运行期间不得清空注册表。
        """
        self._accepted_positions.clear()
        self._floor_positions.clear()
