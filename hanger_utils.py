# -*- coding: utf-8 -*-
from __future__ import annotations

import math
from typing import Iterable

import numpy as np


HANGER_POINT_NAMES = ["GUADIAN"] + [f"GUADIAN{i}" for i in range(1, 11)]


def dedupe_points(points: list[list[float]], tolerance: float = 1e-5) -> list[list[float]]:
    seen = set()
    unique = []
    scale = 1.0 / max(float(tolerance), 1e-12)
    for point in points:
        key = tuple(int(round(float(value) * scale)) for value in point)
        if key in seen:
            continue
        seen.add(key)
        unique.append(point)
    return unique


def as_point_array(points: Iterable[Iterable[float]] | None) -> np.ndarray:
    if points is None:
        return np.zeros((0, 3), dtype=float)
    valid = []
    for point in points:
        try:
            values = [float(point[0]), float(point[1]), float(point[2])]
        except Exception:
            continue
        if all(math.isfinite(value) for value in values):
            valid.append(values)
    valid = dedupe_points(valid)
    return np.asarray(valid, dtype=float) if valid else np.zeros((0, 3), dtype=float)


def settings_hanger_points(settings_module) -> np.ndarray:
    return as_point_array(getattr(settings_module, "HANGER_POINTS", []))


def point_segment_distance(point, a, b) -> float:
    point = np.asarray(point, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return float(np.linalg.norm(point - a))
    t = max(0.0, min(1.0, float(np.dot(point - a, ab) / denom)))
    closest = a + t * ab
    return float(np.linalg.norm(point - closest))


def point_path_distance(point, path: np.ndarray) -> float:
    path = np.asarray(path, dtype=float)
    if len(path) == 0:
        return float("inf")
    if len(path) == 1:
        return float(np.linalg.norm(np.asarray(point, dtype=float) - path[0]))
    return min(point_segment_distance(point, a, b) for a, b in zip(path[:-1], path[1:]))


def hanger_distance_report(path, hanger_points, distance_range: float) -> dict:
    path = np.asarray(path, dtype=float)
    points = as_point_array(hanger_points)
    limit = float(distance_range)
    records = []
    min_distance = float("inf")
    max_distance = 0.0
    satisfied = 0
    for index, point in enumerate(points, start=1):
        distance = point_path_distance(point, path)
        ok = bool(distance <= limit)
        records.append({
            "index": int(index),
            "name": f"GUADIAN{index}",
            "point": point.tolist(),
            "distance": float(distance),
            "within_range": ok,
        })
        min_distance = min(min_distance, distance)
        max_distance = max(max_distance, distance)
        satisfied += int(ok)
    return {
        "range": limit,
        "count": int(len(points)),
        "satisfied_count": int(satisfied),
        "all_within_range": bool(satisfied == len(points)),
        "min_distance": float(min_distance) if len(points) else None,
        "max_distance": float(max_distance) if len(points) else None,
        "points": records,
    }


def hanger_reference_cost(point, hanger_points, distance_range: float, weight: float) -> float:
    points = as_point_array(hanger_points)
    if len(points) == 0 or weight <= 0.0:
        return 0.0
    point = np.asarray(point, dtype=float)
    limit = max(float(distance_range), 1e-6)
    cost = 0.0
    for hanger in points:
        distance = float(np.linalg.norm(point - hanger))
        if distance > limit:
            cost += ((distance - limit) / limit) ** 2
    return float(weight) * cost
