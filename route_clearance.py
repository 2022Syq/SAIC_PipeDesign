"""Sampling and swept-envelope clearance checks for confirmed routes.

Routes are represented by centerline polylines.  Sampling makes the check
independent of the planner's original point spacing; each sampled segment is
then treated as a capsule with the pipe's outside radius.  The minimum gap is
the closest centerline segment distance minus the two outside radii.
"""

from __future__ import annotations

import math
from collections.abc import Mapping, Sequence
from typing import Any

import numpy as np

from pipe_config import PipeInstance, spec_outer_radius


def _point(value: Any, field_name: str) -> np.ndarray:
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{field_name} must contain exactly three coordinates")
    try:
        if len(value) != 3:
            raise ValueError(f"{field_name} must contain exactly three coordinates")
    except TypeError as exc:
        raise ValueError(f"{field_name} must contain exactly three coordinates") from exc
    try:
        result = np.asarray([float(item) for item in value], dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must contain numeric coordinates") from exc
    if not np.all(np.isfinite(result)):
        raise ValueError(f"{field_name} must contain finite coordinates")
    return result


def _path_value(route: Any) -> Any:
    if isinstance(route, Mapping):
        # An engineered path is the final centerline when available.  A
        # planner route normally carries ``path``; a raw sampled path is also
        # accepted for callers that already performed this step.
        for key in ("engineered_path", "path", "sampled_path"):
            if key in route and route[key] is not None:
                return route[key]
        raise ValueError("route object must contain path, engineered_path, or sampled_path")
    return route


def sample_polyline(path: Any, step: float) -> list[list[float]]:
    """Resample a 3-D polyline at no more than ``step`` millimetres per chord."""
    try:
        spacing = float(step)
    except (TypeError, ValueError) as exc:
        raise ValueError("sample step must be a finite positive number") from exc
    if not math.isfinite(spacing) or spacing <= 0.0:
        raise ValueError("sample step must be a finite positive number")
    raw = _path_value(path)
    if isinstance(raw, (str, bytes, Mapping)):
        raise ValueError("route path must be a sequence of points")
    try:
        len(raw)
    except TypeError as exc:
        raise ValueError("route path must be a sequence of points") from exc
    points = [_point(value, f"path[{index}]") for index, value in enumerate(raw)]
    if not points:
        return []
    sampled = [points[0]]
    for start, end in zip(points[:-1], points[1:]):
        delta = end - start
        length = float(np.linalg.norm(delta))
        count = max(1, int(math.ceil(length / spacing)))
        for index in range(1, count + 1):
            sampled.append(start + delta * (index / count))
    return [[float(component) for component in point] for point in sampled]


def sample_route(route: Any, step: float = 20.0) -> list[list[float]]:
    """Sample either a centerline point sequence or a route result mapping."""
    return sample_polyline(route, step)


def _segment_distance(a0: np.ndarray, a1: np.ndarray, b0: np.ndarray, b1: np.ndarray) -> tuple[float, np.ndarray, np.ndarray]:
    """Closest distance and witness points for two finite 3-D segments."""
    # Ericson, Real-Time Collision Detection, closest points of two segments.
    u = a1 - a0
    v = b1 - b0
    w = a0 - b0
    aa = float(np.dot(u, u))
    bb = float(np.dot(v, v))
    ab = float(np.dot(u, v))
    aw = float(np.dot(u, w))
    bw = float(np.dot(v, w))
    eps = 1e-15

    if aa <= eps and bb <= eps:
        pa, pb = a0, b0
    elif aa <= eps:
        tb = min(1.0, max(0.0, bw / bb))
        pa, pb = a0, b0 + tb * v
    elif bb <= eps:
        ta = min(1.0, max(0.0, -aw / aa))
        pa, pb = a0 + ta * u, b0
    else:
        denominator = aa * bb - ab * ab
        if denominator > eps:
            ta = (ab * bw - aw * bb) / denominator
            ta = min(1.0, max(0.0, ta))
        else:
            ta = 0.0
        tb = (ab * ta + bw) / bb
        if tb < 0.0:
            tb = 0.0
            ta = min(1.0, max(0.0, -aw / aa))
        elif tb > 1.0:
            tb = 1.0
            ta = min(1.0, max(0.0, (ab - aw) / aa))
        pa, pb = a0 + ta * u, b0 + tb * v
    return float(np.linalg.norm(pa - pb)), pa, pb


def _segments(points: list[list[float]]) -> list[tuple[np.ndarray, np.ndarray]]:
    values = [np.asarray(point, dtype=float) for point in points]
    if len(values) == 1:
        return [(values[0], values[0])]
    return list(zip(values[:-1], values[1:]))


def minimum_centerline_distance(path_a: Any, path_b: Any, sample_step: float = 20.0) -> dict[str, Any]:
    """Find the minimum distance between two sampled centerline polylines."""
    sampled_a = sample_route(path_a, sample_step)
    sampled_b = sample_route(path_b, sample_step)
    if not sampled_a or not sampled_b:
        raise ValueError("both routes must contain at least one point")
    best_distance = float("inf")
    best_a = best_b = None
    for a0, a1 in _segments(sampled_a):
        for b0, b1 in _segments(sampled_b):
            distance, witness_a, witness_b = _segment_distance(a0, a1, b0, b1)
            if distance < best_distance:
                best_distance, best_a, best_b = distance, witness_a, witness_b
    return {
        "centerline_min_distance": best_distance,
        "closest_point_a": [float(v) for v in best_a],
        "closest_point_b": [float(v) for v in best_b],
        "sampled_path_a": sampled_a,
        "sampled_path_b": sampled_b,
    }


def check_route_clearance(
    path_a: Any,
    radius_a: float,
    path_b: Any,
    radius_b: float,
    required_clearance: float = 0.0,
    sample_step: float = 20.0,
) -> dict[str, Any]:
    """Check the gap between two swept pipe envelopes.

    ``radius_a`` and ``radius_b`` are outside radii in millimetres.  The
    returned ``minimum_clearance`` is the centerline distance minus both
    radii, so zero means touching envelopes.  ``required_clearance`` is an
    additional wall-to-wall gap.
    """
    for name, value in (("radius_a", radius_a), ("radius_b", radius_b), ("required_clearance", required_clearance)):
        if isinstance(value, bool):
            raise ValueError(f"{name} must be a finite non-negative number")
        try:
            number = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{name} must be a finite non-negative number") from exc
        if not math.isfinite(number) or number < 0.0:
            raise ValueError(f"{name} must be a finite non-negative number")
        if name == "radius_a":
            radius_a = number
        elif name == "radius_b":
            radius_b = number
        else:
            required_clearance = number
    report = minimum_centerline_distance(path_a, path_b, sample_step)
    minimum_clearance = float(report["centerline_min_distance"]) - float(radius_a) - float(radius_b)
    report.update(
        {
            "valid": minimum_clearance >= float(required_clearance),
            "minimum_clearance": minimum_clearance,
            "min_clearance": minimum_clearance,
            "required_clearance": float(required_clearance),
            "radius_a": float(radius_a),
            "radius_b": float(radius_b),
            "sample_step": float(sample_step),
            "sample_count_a": len(report["sampled_path_a"]),
            "sample_count_b": len(report["sampled_path_b"]),
        }
    )
    return report


def check_pipe_clearance(
    pipe_a: PipeInstance,
    route_a: Any,
    pipe_b: PipeInstance,
    route_b: Any,
    sample_step: float = 20.0,
) -> dict[str, Any]:
    """Check routes using radii and required gap from two ``PipeInstance``s."""
    if not isinstance(pipe_a, PipeInstance) or not isinstance(pipe_b, PipeInstance):
        raise TypeError("check_pipe_clearance expects two PipeInstance values")
    required = max(pipe_a.required_clearance, pipe_b.required_clearance)
    report = check_route_clearance(route_a, pipe_a.outer_radius, route_b, pipe_b.outer_radius, required, sample_step)
    report.update({"pipe_id_a": pipe_a.pipe_id, "pipe_id_b": pipe_b.pipe_id})
    return report


minimum_swept_clearance = check_route_clearance
swept_envelope_clearance = check_route_clearance


__all__ = [
    "sample_polyline",
    "sample_route",
    "minimum_centerline_distance",
    "check_route_clearance",
    "check_pipe_clearance",
    "minimum_swept_clearance",
    "swept_envelope_clearance",
]
