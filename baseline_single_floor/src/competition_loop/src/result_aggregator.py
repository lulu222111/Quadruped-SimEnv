#!/usr/bin/env python3
# =============================================================================
# 模块说明（中文）：result_aggregator.py —— 每物理源聚合成一个代表点
#
# 职责：把已接受的候选按 xy 近邻聚类，每个簇输出一个质心点（z 取均值、
#       confidence 取最大、count 记录观测数）。
# 为什么需要它：同一个真实危险源被多次观测时，反投影结果会在 ~0.9 m 内漂移。
#       router 的去重半径较小、无法合并这些漂移点时，同一源会裂成多条，
#       直接抬高误报率（误报分母按条数计）。因此聚类半径必须"大于同源漂移、
#       小于异源最小间距"，默认取 1.0 m 就是这样定出来的。
# 已知前提：候选的 z 已在 loop 侧归位到地面高度，因此仅用 xy 聚类即可。
# 结构导航：AggregatedPoint（行 20-32）→ 取属性助手（行 35-44）→
#       aggregate_candidates（行 47-93，单遍增量均值聚类）。
# 输入：已接受的候选序列
# 输出：每源一个代表点的列表
# =============================================================================
"""result_aggregator.py — collapse accepted hazard candidates into one
representative point per physical source before writing the final result.

Rationale (P0-2): a single real source is re-observed from many views and the
router's small de-dup radius cannot merge candidates that drift apart by more
than the de-dup distance. Every extra candidate inflates the referee's
false-alarm denominator (far = false_alarms / detected_count), so we emit one
point per cluster. The cluster radius must exceed the worst same-source drift
(~0.9 m) while staying well below the minimum spacing between distinct sources
(> 2 m), hence the default 1.0 m. z is already snapped to the source-centre
height by the P2 z-snap step before this runs, so clustering uses the xy plane.
"""

from __future__ import annotations

import math


class AggregatedPoint:
    """Duck-typed candidate: exposes .position (x/y/z), .confidence, .count."""

    __slots__ = ("position", "confidence", "count")

    def __init__(self, x, y, z, confidence=1.0, count=1):
        point = type("Point", (), {})()
        point.x = float(x)
        point.y = float(y)
        point.z = float(z)
        self.position = point
        self.confidence = float(confidence)
        self.count = int(count)


def _candidate_xy(candidate):
    return (float(candidate.position.x), float(candidate.position.y))


def _candidate_confidence(candidate):
    try:
        value = float(candidate.confidence)
    except (AttributeError, TypeError, ValueError):
        return 1.0
    return value if math.isfinite(value) else 1.0


def aggregate_candidates(candidates, radius=1.0):
    """Cluster candidates by xy proximity and return one representative each.

    Candidates are processed in arrival order and each is assigned to the
    first existing cluster whose running xy centroid is within ``radius``
    (single pass, incremental mean). The representative is the cluster
    centroid; z is averaged (already snapped to the source-centre height by
    P2), confidence is the cluster maximum, count is the number of fused
    observations.
    """
    radius = float(radius)
    if not (math.isfinite(radius) and radius > 0.0):
        raise ValueError("radius must be finite and > 0")

    clusters = []
    for candidate in candidates:
        x, y = _candidate_xy(candidate)
        confidence = _candidate_confidence(candidate)
        cluster = None
        for cl in clusters:
            mean_x = cl["sx"] / cl["n"]
            mean_y = cl["sy"] / cl["n"]
            if math.hypot(x - mean_x, y - mean_y) <= radius:
                cluster = cl
                break
        if cluster is None:
            cluster = {"sx": 0.0, "sy": 0.0, "sz": 0.0, "conf": 0.0, "n": 0}
            clusters.append(cluster)
        cluster["sx"] += x
        cluster["sy"] += y
        cluster["sz"] += float(candidate.position.z)
        cluster["conf"] = max(cluster["conf"], confidence)
        cluster["n"] += 1

    result = []
    for cl in clusters:
        n = cl["n"]
        result.append(
            AggregatedPoint(
                cl["sx"] / n,
                cl["sy"] / n,
                cl["sz"] / n,
                confidence=cl["conf"],
                count=n,
            )
        )
    return result
