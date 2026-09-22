# -*- coding: utf-8 -*-
"""
Engineering pipe reconstruction from the main2 guide routes.

Outputs:
- outputs/route_engineered.json
- outputs/route_engineered.html
"""

from __future__ import annotations

import json
import math
import time
from pathlib import Path

import numpy as np

import settings
from common_space import html_aspect_ratio, html_x_range, load_json, load_step_meshes_for_html, save_json
from hanger_utils import hanger_distance_report, settings_hanger_points
from main2 import DualSectionModel, joint_segment_allowed, joint_validate_path, section_polygons


ROUTE_COLORS = ["#ef2929", "#111827", "#2563eb", "#16a34a", "#f97316"]


def norm(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v if n < 1e-9 else v / n


def dist(a, b) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def path_length(path: np.ndarray) -> float:
    return float(sum(dist(a, b) for a, b in zip(path[:-1], path[1:])))


def cumulative_lengths(path: np.ndarray) -> np.ndarray:
    path = np.asarray(path, dtype=float)
    if len(path) == 0:
        return np.asarray([], dtype=float)
    values = [0.0]
    for a, b in zip(path[:-1], path[1:]):
        values.append(values[-1] + dist(a, b))
    return np.asarray(values, dtype=float)


def total_variation(values: np.ndarray) -> float:
    values = np.asarray(values, dtype=float)
    if len(values) <= 1:
        return 0.0
    return float(np.sum(np.abs(np.diff(values))))


def remove_close_points(path: np.ndarray, min_len: float) -> np.ndarray:
    if len(path) <= 1:
        return path
    out = [np.asarray(path[0], dtype=float)]
    for p in np.asarray(path[1:], dtype=float):
        if dist(out[-1], p) >= min_len:
            out.append(p)
    if dist(out[-1], path[-1]) > 1e-6:
        out.append(np.asarray(path[-1], dtype=float))
    return np.asarray(out, dtype=float)


def point_line_distance(p: np.ndarray, a: np.ndarray, b: np.ndarray) -> float:
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return dist(p, a)
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    q = a + t * ab
    return dist(p, q)


def point_segment_distance_2d(p, a, b) -> float:
    p = np.asarray(p, dtype=float)
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    ab = b - a
    denom = float(np.dot(ab, ab))
    if denom < 1e-12:
        return float(np.linalg.norm(p - a))
    t = max(0.0, min(1.0, float(np.dot(p - a, ab) / denom)))
    q = a + t * ab
    return float(np.linalg.norm(p - q))


def rdp(points: np.ndarray, tolerance: float) -> np.ndarray:
    points = np.asarray(points, dtype=float)
    if len(points) <= 2:
        return points
    a = points[0]
    b = points[-1]
    dmax = -1.0
    idx = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], a, b)
        if d > dmax:
            dmax = d
            idx = i
    if dmax > tolerance:
        left = rdp(points[: idx + 1], tolerance)
        right = rdp(points[idx:], tolerance)
        return np.vstack([left[:-1], right])
    return np.vstack([a, b])


def rdp_indices(points: np.ndarray, tolerance: float, offset: int = 0) -> list[int]:
    points = np.asarray(points, dtype=float)
    if len(points) <= 2:
        return list(range(offset, offset + len(points)))

    a = points[0]
    b = points[-1]
    dmax = -1.0
    idx = 0
    for i in range(1, len(points) - 1):
        d = point_line_distance(points[i], a, b)
        if d > dmax:
            dmax = d
            idx = i

    if dmax > tolerance:
        left = rdp_indices(points[: idx + 1], tolerance, offset)
        right = rdp_indices(points[idx:], tolerance, offset + idx)
        return left[:-1] + right
    return [offset, offset + len(points) - 1]


def simplify_to_max_points(points: np.ndarray, max_points: int) -> np.ndarray:
    """Increase RDP tolerance until no more than max_points remain."""
    points = remove_close_points(np.asarray(points, dtype=float), 1e-6)
    max_points = max(2, int(max_points))
    if len(points) <= max_points:
        return points

    low = 0.0
    high = max(path_length(points), 1.0)
    best = np.vstack([points[0], points[-1]])
    for _ in range(48):
        tolerance = 0.5 * (low + high)
        candidate = rdp(points, tolerance)
        if len(candidate) <= max_points:
            best = candidate
            high = tolerance
        else:
            low = tolerance
    return best


def collision_constrained_control_path(
    candidates: np.ndarray,
    max_control_points: int,
    model: DualSectionModel,
    allow_soft: bool,
) -> np.ndarray:
    """Shortest ordered subsequence whose every chord passes clearance checks."""
    points = remove_close_points(np.asarray(candidates, dtype=float), 1e-6)
    max_control_points = max(2, int(max_control_points))
    if len(points) < 2:
        raise RuntimeError("not enough control-point candidates")

    # dp[(count, end_index)] = (length, previous_index)
    dp: dict[tuple[int, int], tuple[float, int | None]] = {(1, 0): (0.0, None)}
    edge_cache = {}

    def edge_allowed(i: int, j: int) -> bool:
        key = (i, j)
        if key not in edge_cache:
            edge_cache[key] = joint_segment_allowed(model, points[i], points[j], allow_soft)[0]
        return edge_cache[key]

    for count in range(2, max_control_points + 1):
        for end in range(1, len(points)):
            best = None
            for previous in range(end):
                old = dp.get((count - 1, previous))
                if old is None or not edge_allowed(previous, end):
                    continue
                cost = old[0] + dist(points[previous], points[end])
                if best is None or cost < best[0]:
                    best = (cost, previous)
            if best is not None:
                dp[(count, end)] = best

    target = len(points) - 1
    feasible = [
        (dp[(count, target)][0], count)
        for count in range(2, max_control_points + 1)
        if (count, target) in dp
    ]
    if not feasible:
        raise RuntimeError(
            f"no clearance-safe control path exists with at most {max_control_points} control points"
        )
    _cost, count = min(feasible)
    indices = [target]
    end = target
    while count > 1:
        previous = dp[(count, end)][1]
        indices.append(int(previous))
        end = int(previous)
        count -= 1
    indices.reverse()
    return points[indices]


def clearance_valid_path(model: DualSectionModel, path: np.ndarray, allow_soft: bool) -> bool:
    validation = joint_validate_path(model, np.asarray(path, dtype=float))
    return validation["hard_bad_segments"] == 0 and (allow_soft or validation["soft_bad_segments"] == 0)


def xy_turn(prev_pt: np.ndarray, vertex: np.ndarray, next_pt: np.ndarray) -> tuple[float, int]:
    a = np.asarray(vertex, dtype=float)[:2] - np.asarray(prev_pt, dtype=float)[:2]
    b = np.asarray(next_pt, dtype=float)[:2] - np.asarray(vertex, dtype=float)[:2]
    la = float(np.linalg.norm(a))
    lb = float(np.linalg.norm(b))
    if la < 1e-9 or lb < 1e-9:
        return 0.0, 0
    dot = max(-1.0, min(1.0, float(np.dot(a, b) / (la * lb))))
    angle = math.degrees(math.acos(dot))
    cross = float(a[0] * b[1] - a[1] * b[0])
    sign = 1 if cross > 0.0 else -1 if cross < 0.0 else 0
    return angle, sign


def corner_included_angle_deg(prev_pt, vertex, next_pt) -> float:
    """Internal angle between two straight segments joined at a corner."""
    p0 = np.asarray(prev_pt, dtype=float)
    p1 = np.asarray(vertex, dtype=float)
    p2 = np.asarray(next_pt, dtype=float)
    len_in = dist(p0, p1)
    len_out = dist(p1, p2)
    if len_in < 1e-6 or len_out < 1e-6:
        return 180.0
    u = norm(p1 - p0)
    v = norm(p2 - p1)
    dot = max(-1.0, min(1.0, float(np.dot(u, v))))
    turn_deg = math.degrees(math.acos(dot))
    return float(180.0 - turn_deg)


def remove_local_s_bends(
    path: np.ndarray,
    min_turn_deg: float,
    max_span: float,
    min_detour: float,
) -> tuple[np.ndarray, int]:
    points = np.asarray(path, dtype=float)
    if len(points) <= 4:
        return points, 0

    removed = 0
    changed = True
    while changed and len(points) > 4:
        changed = False
        for i in range(1, len(points) - 2):
            turn_a, sign_a = xy_turn(points[i - 1], points[i], points[i + 1])
            turn_b, sign_b = xy_turn(points[i], points[i + 1], points[i + 2])
            if sign_a == 0 or sign_b == 0 or sign_a == sign_b:
                continue
            if turn_a < min_turn_deg or turn_b < min_turn_deg:
                continue

            span = float(np.linalg.norm(points[i + 2][:2] - points[i - 1][:2]))
            if span > max_span:
                continue

            local_len = (
                dist(points[i - 1], points[i])
                + dist(points[i], points[i + 1])
                + dist(points[i + 1], points[i + 2])
            )
            chord_len = dist(points[i - 1], points[i + 2])
            if local_len - chord_len < min_detour:
                continue

            points = np.vstack([points[:i], points[i + 2:]])
            removed += 2
            changed = True
            break

    return points, removed


def smooth_z_profile(path: np.ndarray, tolerance: float):
    path = np.asarray(path, dtype=float)
    if len(path) <= 2 or tolerance <= 0.0:
        return path.copy(), list(range(len(path)))

    s = cumulative_lengths(path)
    if len(s) == 0 or s[-1] < 1e-9:
        return path.copy(), list(range(len(path)))

    profile = np.column_stack([s, path[:, 2]])
    keep = sorted(set(rdp_indices(profile, tolerance)))
    smooth = path.copy()
    smooth[:, 2] = np.interp(s, s[keep], path[keep, 2])
    smooth[0, 2] = path[0, 2]
    smooth[-1, 2] = path[-1, 2]
    return smooth, keep


def route_by_name(result: dict, name: str) -> dict:
    routes = result.get("routes") or []
    if not routes:
        raise ValueError("route result has no routes")
    for route in routes:
        if route.get("name") == name:
            return route
    return routes[0]


def selected_source_routes(result: dict) -> list[dict]:
    routes = result.get("routes") or []
    if not routes:
        raise ValueError("route result has no routes")

    source_name = getattr(settings, "ENGINEERING_SOURCE_ROUTE_NAME", None)
    if source_name is None:
        return routes

    source_name = str(source_name).strip()
    if not source_name or source_name.upper() == "ALL":
        return routes

    return [route_by_name(result, source_name)]


def fillet_corner(prev_pt, vertex, next_pt, radius: float, max_arc_deg: float, sample_deg: float):
    p0 = np.asarray(prev_pt, dtype=float)
    p1 = np.asarray(vertex, dtype=float)
    p2 = np.asarray(next_pt, dtype=float)

    u = norm(p1 - p0)
    v = norm(p2 - p1)
    len_in = dist(p0, p1)
    len_out = dist(p1, p2)
    dot = max(-1.0, min(1.0, float(np.dot(u, v))))
    turn = float(math.acos(dot))
    turn_deg = math.degrees(turn)
    if turn_deg < 1e-3 or len_in < 1e-6 or len_out < 1e-6:
        return None

    tan_half = math.tan(turn / 2.0)
    if abs(tan_half) < 1e-9:
        return None

    preferred_d = radius * tan_half
    max_d = 0.45 * min(len_in, len_out)
    d = min(preferred_d, max_d)
    if d <= 1e-6:
        return None

    actual_radius = d / tan_half
    t1 = p1 - u * d
    t2 = p1 + v * d

    center_dir = v - dot * u
    if float(np.linalg.norm(center_dir)) < 1e-9:
        return None
    center_dir = norm(center_dir)
    center = t1 + center_dir * actual_radius

    e1 = norm(t1 - center)
    e2 = norm(t2 - center)
    plane_normal = np.cross(e1, e2)
    if float(np.linalg.norm(plane_normal)) < 1e-9:
        return None
    plane_normal = norm(plane_normal)
    m = np.cross(plane_normal, e1)
    arc_angle = math.atan2(float(np.dot(e2, m)), float(np.dot(e2, e1)))
    if arc_angle < 0:
        arc_angle += 2.0 * math.pi

    step = max(math.radians(sample_deg), 1e-3)
    count = max(2, int(math.ceil(arc_angle / step)))
    samples = [
        center + actual_radius * (math.cos(t) * e1 + math.sin(t) * m)
        for t in np.linspace(0.0, arc_angle, count + 1)
    ]

    split_count = max(1, int(math.ceil(math.degrees(arc_angle) / max(max_arc_deg, 1e-6))))
    arc_segments = []
    for idx in range(split_count):
        a = arc_angle * idx / split_count
        b = arc_angle * (idx + 1) / split_count
        start = center + actual_radius * (math.cos(a) * e1 + math.sin(a) * m)
        end = center + actual_radius * (math.cos(b) * e1 + math.sin(b) * m)
        arc_segments.append({
            "type": "arc",
            "center": center.tolist(),
            "radius": float(actual_radius),
            "angle_deg": float(math.degrees(b - a)),
            "start": start.tolist(),
            "end": end.tolist(),
        })

    return {
        "turn_angle_deg": float(turn_deg),
        "radius": float(actual_radius),
        "radius_reduced": bool(actual_radius < radius - 1e-6),
        "tangent_in": t1,
        "tangent_out": t2,
        "samples": np.asarray(samples, dtype=float),
        "arc_segments": arc_segments,
    }


def build_line_arc_route(
    guide: np.ndarray,
    radius: float,
    max_arc_deg: float,
    sample_deg: float,
    min_corner_angle_deg: float,
):
    segments = []
    sampled = [guide[0]]
    current = guide[0]
    arc_count = 0
    turn_count = 0
    reduced_count = 0
    max_turn = 0.0
    required_corner_angle = max(0.0, min(180.0, float(min_corner_angle_deg)))
    min_corner_angle = 180.0
    corner_angle_violations = []

    for i in range(1, len(guide) - 1):
        included_angle = corner_included_angle_deg(guide[i - 1], guide[i], guide[i + 1])
        min_corner_angle = min(min_corner_angle, included_angle)
        if included_angle + 1e-6 < required_corner_angle:
            corner_angle_violations.append({
                "index": int(i),
                "angle_deg": float(included_angle),
                "required_deg": float(required_corner_angle),
                "point": guide[i].tolist(),
            })

        if i == 1 or i == len(guide) - 2:
            if dist(current, guide[i]) > 1e-6:
                segments.append({"type": "line", "start": current.tolist(), "end": guide[i].tolist()})
                sampled.append(guide[i])
                current = guide[i]
            continue

        fillet = fillet_corner(guide[i - 1], guide[i], guide[i + 1], radius, max_arc_deg, sample_deg)
        if fillet is None:
            if dist(current, guide[i]) > 1e-6:
                segments.append({"type": "line", "start": current.tolist(), "end": guide[i].tolist()})
                sampled.append(guide[i])
                current = guide[i]
            continue

        t1 = fillet["tangent_in"]
        t2 = fillet["tangent_out"]
        if dist(current, t1) > 1e-6:
            segments.append({"type": "line", "start": current.tolist(), "end": t1.tolist()})
            sampled.append(t1)

        segments.extend(fillet["arc_segments"])
        for p in fillet["samples"][1:]:
            sampled.append(p)
        current = t2
        arc_count += len(fillet["arc_segments"])
        turn_count += 1
        reduced_count += int(fillet["radius_reduced"])
        max_turn = max(max_turn, fillet["turn_angle_deg"])

    if dist(current, guide[-1]) > 1e-6:
        segments.append({"type": "line", "start": current.tolist(), "end": guide[-1].tolist()})
        sampled.append(guide[-1])

    sampled = np.asarray(sampled, dtype=float)
    return {
        "segments": segments,
        "sampled_path": sampled,
        "arc_count": int(arc_count),
        "turn_count": int(turn_count),
        "radius_reduced_count": int(reduced_count),
        "max_turn_angle_deg": float(max_turn),
        "min_corner_angle_deg": float(min_corner_angle),
        "required_min_corner_angle_deg": float(required_corner_angle),
        "corner_angle_valid": len(corner_angle_violations) == 0,
        "corner_angle_violations": corner_angle_violations,
    }


def sample_segment(a, b, step: float) -> list[np.ndarray]:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    length = dist(a, b)
    count = max(1, int(math.ceil(length / max(step, 1e-6))))
    return [a + (b - a) * (i / count) for i in range(count + 1)]


def sampled_path(path: list[list[float]], step: float) -> list[list[float]]:
    if not path:
        return []
    samples = [np.asarray(path[0], dtype=float)]
    for a, b in zip(path[:-1], path[1:]):
        for p in sample_segment(a, b, step)[1:]:
            samples.append(p)
    return [p.tolist() for p in samples]


def distance_to_component_lines(yz, comp: dict) -> float:
    best = 1e18
    for line in comp.get("lines", []):
        for a, b in zip(line[:-1], line[1:]):
            best = min(best, point_segment_distance_2d(yz, a, b))
    return best


def component_lines_for_distance(section: dict, mode: str) -> list[dict]:
    if mode == "see_through":
        comps = section.get("soft_obstacle_section_lines", [])
        return [
            comp for comp in comps
            if "see_through" in str(comp.get("name", "")).lower()
        ]
    return [
        comp for comp in section.get("hard_obstacle_section_lines", section.get("obstacle_section_lines", []))
        if "see_through" not in str(comp.get("name", "")).lower()
    ]


def distance_profile_for_route(space: dict, route: dict, mode: str) -> dict:
    samples = sampled_path(route["engineered_path"], float(getattr(settings, "DISTANCE_SAMPLE_STEP", 20.0)))
    sections = space["sections"]
    xs = np.asarray([s["x"] for s in sections], dtype=float)
    y_max = float(getattr(settings, "DISTANCE_Y_MAX", 40.0))

    x_values = []
    clearances = []
    for p in samples:
        si = int(np.argmin(np.abs(xs - float(p[0]))))
        comps = component_lines_for_distance(sections[si], mode)
        yz = [float(p[1]), float(p[2])]
        best = 1e18
        for comp in comps:
            best = min(best, distance_to_component_lines(yz, comp) - settings.PIPE_RADIUS)
        if best >= 1e17:
            clearance = None
        else:
            clearance = max(0.0, min(float(best), y_max))
        x_values.append(float(p[0]))
        clearances.append(clearance)

    return {
        "name": route["name"],
        "color": route["color"],
        "x": x_values,
        "clearance": clearances,
    }


def tube_mesh(path: np.ndarray, radius: float, ring_count: int) -> dict:
    pts = np.asarray(path, dtype=float)
    ring_count = max(6, int(ring_count))
    xs = []
    ys = []
    zs = []
    ii = []
    jj = []
    kk = []

    prev_n = None
    for idx, p in enumerate(pts):
        if idx == 0:
            tangent = norm(pts[1] - pts[0])
        elif idx == len(pts) - 1:
            tangent = norm(pts[-1] - pts[-2])
        else:
            tangent = norm(pts[idx + 1] - pts[idx - 1])

        if prev_n is None:
            ref = np.array([0.0, 0.0, 1.0])
            if abs(float(np.dot(ref, tangent))) > 0.9:
                ref = np.array([0.0, 1.0, 0.0])
            n = norm(ref - np.dot(ref, tangent) * tangent)
        else:
            n = norm(prev_n - np.dot(prev_n, tangent) * tangent)
            if float(np.linalg.norm(n)) < 1e-9:
                n = prev_n
        b = norm(np.cross(tangent, n))
        prev_n = n

        for j in range(ring_count):
            a = 2.0 * math.pi * j / ring_count
            q = p + radius * (math.cos(a) * n + math.sin(a) * b)
            xs.append(float(q[0]))
            ys.append(float(q[1]))
            zs.append(float(q[2]))

    for idx in range(len(pts) - 1):
        base0 = idx * ring_count
        base1 = (idx + 1) * ring_count
        for j in range(ring_count):
            a = base0 + j
            b0 = base0 + (j + 1) % ring_count
            c = base1 + j
            d = base1 + (j + 1) % ring_count
            ii.extend([a, b0])
            jj.extend([c, d])
            kk.extend([b0, c])

    if len(pts) >= 2:
        start_center = len(xs)
        xs.append(float(pts[0][0]))
        ys.append(float(pts[0][1]))
        zs.append(float(pts[0][2]))
        for j in range(ring_count):
            a = j
            b0 = (j + 1) % ring_count
            ii.append(start_center)
            jj.append(b0)
            kk.append(a)

        end_center = len(xs)
        xs.append(float(pts[-1][0]))
        ys.append(float(pts[-1][1]))
        zs.append(float(pts[-1][2]))
        end_base = (len(pts) - 1) * ring_count
        for j in range(ring_count):
            a = end_base + j
            b0 = end_base + (j + 1) % ring_count
            ii.append(end_center)
            jj.append(a)
            kk.append(b0)

    return {"x": xs, "y": ys, "z": zs, "i": ii, "j": jj, "k": kk}


def engineer_route(source_route: dict, color: str, params: dict) -> dict:
    route_name = source_route.get("name", "route")
    guide = np.asarray(source_route["path"], dtype=float)
    guide = remove_close_points(guide, 1e-6)

    tangent_info = source_route.get("search", {}).get("endpoint_tangency", {})
    start_count = int(tangent_info.get("start", {}).get("curve_point_count", 0))
    goal_count = int(tangent_info.get("goal", {}).get("curve_point_count", 0))
    protected = (
        tangent_info.get("applied", False)
        and start_count >= 2
        and goal_count >= 2
        and start_count + goal_count <= len(guide) + 2
    )

    if protected:
        start_curve = guide[:start_count]
        goal_curve = guide[len(guide) - goal_count:]
        central_guide = guide[start_count - 1:len(guide) - goal_count + 1]
    else:
        start_curve = goal_curve = None
        central_guide = guide

    central_z_smoothed, z_keep_indices = smooth_z_profile(central_guide, params["z_tolerance"])
    central_control = rdp(central_z_smoothed, params["simplify_tolerance"])
    central_control = remove_close_points(central_control, params["min_segment_length"])
    central_before_s_bend = central_control.copy()
    central_control, s_bend_removed = remove_local_s_bends(
        central_control,
        params["s_bend_min_turn_deg"],
        params["s_bend_max_span"],
        params["s_bend_min_detour"],
    )
    central_control = remove_close_points(central_control, params["min_segment_length"])
    wiggle_removed = 0
    if route_name in params["wiggle_smooth_route_names"]:
        central_control, wiggle_removed = remove_local_s_bends(
            central_control,
            params["wiggle_min_turn_deg"],
            params["wiggle_max_span"],
            params["wiggle_min_detour"],
        )
        central_control = remove_close_points(central_control, params["min_segment_length"])

    if protected:
        def join_protected(middle: np.ndarray) -> np.ndarray:
            return remove_close_points(
                np.vstack([start_curve[:-1], middle, goal_curve[1:]]),
                1e-6,
            )

        z_smoothed_guide = join_protected(central_z_smoothed)
        control_before_s_bend = join_protected(central_before_s_bend)
        control = join_protected(central_control)
    else:
        z_smoothed_guide = central_z_smoothed
        control_before_s_bend = central_before_s_bend
        control = central_control

    max_turn_count = max(1, int(params["max_turn_count"]))
    collision_model = params.get("collision_model")
    allow_soft = bool(source_route.get("allow_soft", False))
    if protected:
        start = guide[0]
        goal = guide[-1]
        # main2 already builds a protected endpoint chain:
        # endpoint -> 30 mm tangent anchor -> direction-filtered connector.
        # Keep that chain locked, and let the clearance graph choose only the
        # middle portion. Otherwise main3 can delete the connector and create
        # an immediate hard turn right after the endpoint anchor.
        start_count = max(2, min(start_count, len(guide)))
        goal_count = max(2, min(goal_count, len(guide) - start_count + 1))
        middle_start = start_count - 1
        middle_end = len(guide) - goal_count
        middle_candidates = remove_close_points(guide[middle_start:middle_end + 1], 1e-6)
        if collision_model is not None:
            middle_control = collision_constrained_control_path(
                middle_candidates, max_turn_count + 2, collision_model, allow_soft
            )
        else:
            middle_control = simplify_to_max_points(middle_candidates, max_turn_count)
        control = remove_close_points(
            np.vstack([
                guide[:middle_start],
                middle_control,
                guide[middle_end + 1:],
            ]),
            1e-6,
        )
        control_before_s_bend = control.copy()
    else:
        if collision_model is not None:
            control = collision_constrained_control_path(
                z_smoothed_guide, max_turn_count + 2, collision_model, allow_soft
            )
        else:
            control = simplify_to_max_points(control, max_turn_count + 2)
        control_before_s_bend = control.copy()

    preferred_radius = float(params["bend_radius"])
    minimum_radius = min(preferred_radius, float(params["min_bend_radius"]))
    radius_step = max(float(params["bend_radius_step"]), 1.0)
    radii = []
    radius = preferred_radius
    while radius > minimum_radius + 1e-9:
        radii.append(radius)
        radius -= radius_step
    radii.append(minimum_radius)

    control_options = [remove_close_points(control, 1e-6)]
    fallback_control = remove_close_points(guide, 1e-6)
    if len(fallback_control) <= max_turn_count + 2 and not np.array_equal(control_options[0], fallback_control):
        control_options.append(fallback_control)

    engineered = None
    used_radius = None
    used_control = None
    for control_candidate in control_options:
        for radius in radii:
            candidate = build_line_arc_route(
                control_candidate,
                radius,
                params["max_bend_angle_deg"],
                params["arc_sample_angle_deg"],
                params["min_corner_angle_deg"],
            )
            clearance_ok = (
                collision_model is None
                or clearance_valid_path(collision_model, candidate["sampled_path"], allow_soft)
            )
            if candidate["corner_angle_valid"] and clearance_ok:
                engineered = candidate
                used_radius = radius
                used_control = control_candidate
                break
        if engineered is not None:
            break
    if engineered is None:
        raise RuntimeError(
            f"{route_name}: no clearance-safe arc reconstruction exists with "
            f"main_turn_count<={max_turn_count}, radius>={minimum_radius:.1f} mm, "
            f"and corner_angle>={params['min_corner_angle_deg']:.1f} deg"
        )
    control = used_control
    control_before_s_bend = used_control
    sampled = engineered["sampled_path"]
    tube = tube_mesh(sampled, params["pipe_radius"], params["tube_segments"])
    hanger_report = hanger_distance_report(
        sampled,
        params.get("hanger_points", []),
        params.get("hanger_distance_range", 100.0),
    )

    return {
        "name": route_name,
        "color": color,
        "bend_radius": float(used_radius),
        "requested_bend_radius": float(preferred_radius),
        "max_bend_angle_deg": params["max_bend_angle_deg"],
        "arc_sample_angle_deg": params["arc_sample_angle_deg"],
        "max_turn_count": int(max_turn_count),
        "simplify_tolerance": params["simplify_tolerance"],
        "z_simplify_tolerance": params["z_tolerance"],
        "z_simplify_points": len(z_keep_indices),
        "endpoint_tangent_protected": bool(protected),
        "start_tangent_point_count": int(start_count if protected else 0),
        "goal_tangent_point_count": int(goal_count if protected else 0),
        "z_total_variation_before": total_variation(guide[:, 2]),
        "z_total_variation_after": total_variation(z_smoothed_guide[:, 2]),
        "guide_path": guide.tolist(),
        "z_smoothed_guide_path": z_smoothed_guide.tolist(),
        "control_path_before_s_bend": control_before_s_bend.tolist(),
        "control_path": control.tolist(),
        "s_bend_removed_points": int(s_bend_removed),
        "wiggle_removed_points": int(wiggle_removed),
        "engineered_path": sampled.tolist(),
        "segments": engineered["segments"],
        "tube_mesh": tube,
        "guide_length": path_length(guide),
        "engineered_length": path_length(sampled),
        "arc_count": engineered["arc_count"],
        "turn_count": engineered["turn_count"],
        "main_turn_count": int(engineered["turn_count"]),
        "radius_reduced_count": engineered["radius_reduced_count"],
        "max_turn_angle_deg": engineered["max_turn_angle_deg"],
        "min_corner_angle_deg": engineered["min_corner_angle_deg"],
        "required_min_corner_angle_deg": engineered["required_min_corner_angle_deg"],
        "corner_angle_valid": bool(engineered["corner_angle_valid"]),
        "corner_angle_violations": engineered["corner_angle_violations"],
        "hanger_distance": hanger_report,
        "hanger_valid": bool(hanger_report["all_within_range"]),
    }


def export_engineering_html(result: dict, output: Path) -> None:
    print("[HTML] loading STEP meshes...")
    x_range = html_x_range(settings)
    meshes = load_step_meshes_for_html(settings, x_range)
    print("[HTML] computing route distance profiles...")
    space = load_json(settings.SPACE_JSON)
    points = []
    for route in result["routes"]:
        points.extend(route["engineered_path"])
        points.extend(route["guide_path"])
        points.extend(route["z_smoothed_guide_path"])
    points.extend(result.get("hanger_points", []))
    payload = {
        "meshes": meshes,
        "routes": result["routes"],
        "hanger_points": result.get("hanger_points", []),
        "hanger_point_distance_range": result.get(
            "hanger_point_distance_range",
            float(getattr(settings, "HANGER_POINT_DISTANCE_RANGE", 100.0)),
        ),
        "x_range": list(x_range),
        "aspect_ratio": html_aspect_ratio(x_range, meshes, points),
        "distance_y_max": float(getattr(settings, "DISTANCE_Y_MAX", 40.0)),
        "distance_profiles": [distance_profile_for_route(space, route, "parts") for route in result["routes"]],
        "see_through_distance_profiles": [
            distance_profile_for_route(space, route, "see_through")
            for route in result["routes"]
        ],
    }
    data_json = json.dumps(payload, ensure_ascii=False)

    html = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Engineered Pipe Route</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    html, body {
      margin: 0;
      height: 100%;
      overflow: hidden;
      font-family: Arial, sans-serif;
    }
    #app {
      width: 100vw;
      height: 100vh;
      display: grid;
      grid-template-columns: 25vw 75vw;
      background: #ffffff;
    }
    #sidebar {
      box-sizing: border-box;
      padding: 10px;
      overflow-y: auto;
      display: flex;
      flex-direction: column;
      gap: 10px;
      background: #f8fafc;
      border-right: 1px solid #d0d0d0;
    }
    #viewer {
      min-width: 0;
      height: 100vh;
    }
    #plot {
      width: 100%;
      height: 100%;
    }
    #panel, #routeControls, #distanceBox, #seeThroughBox {
      background: rgba(255,255,255,.94); border: 1px solid #d0d0d0;
      box-shadow: 0 1px 4px rgba(0,0,0,.08);
      box-sizing: border-box;
    }
    #panel {
      padding: 10px 12px;
      font-size: 15px;
      line-height: 1.25;
    }
    #routeControls {
      padding: 8px 10px;
      width: 100%;
      max-height: 32vh;
      overflow: auto;
      font-size: 12px;
    }
    #routeControls .title, #distanceBox .title, #seeThroughBox .title {
      font-weight: 700;
      margin-bottom: 6px;
    }
    .routeBlock {
      border-top: 1px solid #e5e7eb;
      padding: 5px 0;
    }
    .routeBlock:first-of-type { border-top: 0; }
    .routeMain, .subItem {
      display: flex; align-items: center; gap: 6px;
      white-space: nowrap;
    }
    .swatch {
      width: 22px; height: 5px; border-radius: 2px;
      display: inline-block; flex: 0 0 auto;
    }
    details {
      margin-left: 28px;
      color: #374151;
    }
    summary {
      cursor: pointer;
      user-select: none;
      margin-top: 2px;
    }
    .subItems {
      display: grid;
      gap: 2px;
      margin-top: 4px;
    }
    #distanceBox, #seeThroughBox {
      padding: 8px 8px 4px;
      width: 100%;
      height: 250px;
      flex: 0 0 auto;
    }
    #distancePlot, #seeThroughPlot {
      width: 100%;
      height: 210px;
    }
  </style>
</head>
<body>
  <div id="app">
  <aside id="sidebar">
    <div id="panel">
      <strong>Engineered Pipe Route</strong>
    </div>
    <div id="routeControls">
      <div class="title">Routes</div>
      <div id="routeList"></div>
    </div>
    <div id="distanceBox">
      <div class="title">Minimum Distance to Parts</div>
      <div id="distancePlot"></div>
    </div>
    <div id="seeThroughBox">
      <div class="title">Distance to see_through</div>
      <div id="seeThroughPlot"></div>
    </div>
  </aside>
  <main id="viewer">
    <div id="plot"></div>
  </main>
  </div>
  <script>
const DATA = __DATA__;
const traces = [];
const routeTraceMap = DATA.routes.map(() => ({}));

function addRouteTrace(routeIndex, key, trace) {
  trace.showlegend = false;
  trace.meta = { routeIndex, key };
  routeTraceMap[routeIndex][key] = traces.length;
  traces.push(trace);
}

for (const mesh of DATA.meshes) {
  traces.push({
    type: "mesh3d",
    name: mesh.name,
    x: mesh.x, y: mesh.y, z: mesh.z,
    i: mesh.i, j: mesh.j, k: mesh.k,
    color: mesh.boundary === "soft" ? "#60a5fa" : mesh.role === "ground" ? "#9ca3af" : "#9b9b9b",
    opacity: mesh.boundary === "soft" ? 0.14 : 0.20,
    flatshading: true,
    hoverinfo: "name",
    showlegend: false,
  });
}

DATA.routes.forEach((route, routeIndex) => {
  addRouteTrace(routeIndex, "pipe", {
    type: "mesh3d",
    name: `${route.name} pipe`,
    x: route.tube_mesh.x, y: route.tube_mesh.y, z: route.tube_mesh.z,
    i: route.tube_mesh.i, j: route.tube_mesh.j, k: route.tube_mesh.k,
    color: route.color,
    opacity: route.name.startsWith("01_") ? 0.72 : 0.42,
    flatshading: false,
  });

  addRouteTrace(routeIndex, "centerline", {
    type: "scatter3d",
    mode: "lines",
    name: `${route.name} centerline`,
    x: route.engineered_path.map(p => p[0]),
    y: route.engineered_path.map(p => p[1]),
    z: route.engineered_path.map(p => p[2]),
    line: { color: route.color, width: route.name.startsWith("01_") ? 5 : 4 },
  });

  addRouteTrace(routeIndex, "guide", {
    type: "scatter3d",
    mode: "lines",
    name: `${route.name} main2 guide`,
    x: route.guide_path.map(p => p[0]),
    y: route.guide_path.map(p => p[1]),
    z: route.guide_path.map(p => p[2]),
    line: { color: route.color, width: 3, dash: "dash" },
    visible: false,
  });

  addRouteTrace(routeIndex, "zGuide", {
    type: "scatter3d",
    mode: "lines",
    name: `${route.name} Z-smoothed guide`,
    x: route.z_smoothed_guide_path.map(p => p[0]),
    y: route.z_smoothed_guide_path.map(p => p[1]),
    z: route.z_smoothed_guide_path.map(p => p[2]),
    line: { color: route.color, width: 3, dash: "dot" },
    visible: false,
  });

  addRouteTrace(routeIndex, "control", {
    type: "scatter3d",
    mode: "markers",
    name: `${route.name} control points`,
    x: route.control_path.map(p => p[0]),
    y: route.control_path.map(p => p[1]),
    z: route.control_path.map(p => p[2]),
    marker: { color: route.color, size: 3 },
    visible: false,
  });
});

if (DATA.hanger_points && DATA.hanger_points.length) {
  traces.push({
    type: "scatter3d",
    mode: "markers",
    name: "hanger points",
    x: DATA.hanger_points.map(p => p[0]),
    y: DATA.hanger_points.map(p => p[1]),
    z: DATA.hanger_points.map(p => p[2]),
    marker: { color: "#9333ea", size: 6, symbol: "circle" },
    hovertemplate: "hanger<br>x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f}<extra></extra>",
    showlegend: false,
  });
}

Plotly.newPlot("plot", traces, {
  margin: { l: 0, r: 0, t: 0, b: 0 },
  showlegend: false,
  scene: {
    aspectmode: "manual",
    aspectratio: DATA.aspect_ratio,
    xaxis: { title: "X mm", range: DATA.x_range },
    yaxis: { title: "Y mm" },
    zaxis: { title: "Z mm" },
  },
  legend: {
    orientation: "v",
    x: 0.17,
    xanchor: "left",
    y: 0.995,
    yanchor: "top",
    bgcolor: "rgba(255,255,255,0.72)",
    bordercolor: "rgba(0,0,0,0)",
  },
}, { responsive: true, scrollZoom: true });

const SUBITEMS = [
  ["pipe", "Pipe"],
  ["centerline", "Centerline"],
  ["guide", "main2 Guide"],
  ["zGuide", "Z-smoothed Guide"],
  ["control", "Control Points"],
];
const defaultSubitemVisible = {
  pipe: true,
  centerline: true,
  guide: false,
  zGuide: false,
  control: false,
};

function routeEnabled(routeIndex) {
  return document.getElementById(`route-main-${routeIndex}`).checked;
}

function subitemEnabled(routeIndex, key) {
  return document.getElementById(`route-sub-${routeIndex}-${key}`).checked;
}

function applyRouteVisibility(routeIndex) {
  const enabled = routeEnabled(routeIndex);
  for (const [key] of SUBITEMS) {
    const traceIndex = routeTraceMap[routeIndex][key];
    Plotly.restyle("plot", { visible: enabled && subitemEnabled(routeIndex, key) }, [traceIndex]);
  }
  Plotly.restyle("distancePlot", { visible: enabled }, [routeIndex]);
  Plotly.restyle("seeThroughPlot", { visible: enabled }, [routeIndex]);
}

function buildRouteControls() {
  const list = document.getElementById("routeList");
  DATA.routes.forEach((route, routeIndex) => {
    const block = document.createElement("div");
    block.className = "routeBlock";

    const main = document.createElement("label");
    main.className = "routeMain";
    const hanger = route.hanger_distance || {};
    const hangerInfo = hanger.count
      ? `Hanger ${route.hanger_valid ? "OK" : "NG"} ${hanger.satisfied_count}/${hanger.count}, max ${hanger.max_distance.toFixed(1)} mm / ${hanger.range.toFixed(1)} mm`
      : "Hanger: none";
    main.innerHTML = `
      <input id="route-main-${routeIndex}" type="checkbox" checked>
      <span class="swatch" style="background:${route.color}"></span>
      <span>${route.name}</span>
    `;
    block.appendChild(main);
    const hangerDiv = document.createElement("div");
    hangerDiv.className = "subItem";
    hangerDiv.textContent = hangerInfo;
    block.appendChild(hangerDiv);

    const details = document.createElement("details");
    details.innerHTML = `<summary>Subitems</summary><div class="subItems"></div>`;
    const subItems = details.querySelector(".subItems");
    for (const [key, label] of SUBITEMS) {
      const sub = document.createElement("label");
      sub.className = "subItem";
      const checked = defaultSubitemVisible[key] ? "checked" : "";
      sub.innerHTML = `<input id="route-sub-${routeIndex}-${key}" type="checkbox" ${checked}> ${label}`;
      subItems.appendChild(sub);
    }
    block.appendChild(details);
    list.appendChild(block);

    document.getElementById(`route-main-${routeIndex}`).addEventListener("change", () => applyRouteVisibility(routeIndex));
    for (const [key] of SUBITEMS) {
      document.getElementById(`route-sub-${routeIndex}-${key}`).addEventListener("change", () => applyRouteVisibility(routeIndex));
    }
  });
}

function distanceTracesFromProfiles(profiles) {
  return profiles.map(route => ({
    type: "scatter",
    mode: "lines",
    name: route.name,
    x: route.x,
    y: route.clearance,
    line: { color: route.color, width: 2 },
    hovertemplate: "X=%{x:.0f} mm<br>distance=%{y:.1f} mm<extra>" + route.name + "</extra>",
  }));
}

const distanceLayoutBase = {
  margin: { l: 42, r: 8, t: 8, b: 34 },
  showlegend: false,
  xaxis: {
    title: "X mm",
    range: DATA.x_range,
    gridcolor: "#e5e7eb",
    zeroline: false,
  },
  yaxis: {
    title: "mm",
    range: [0, DATA.distance_y_max],
    gridcolor: "#e5e7eb",
    zeroline: false,
  },
  paper_bgcolor: "rgba(255,255,255,0)",
  plot_bgcolor: "rgba(255,255,255,0)",
};

const distanceTraces = distanceTracesFromProfiles(DATA.distance_profiles);
const seeThroughTraces = distanceTracesFromProfiles(DATA.see_through_distance_profiles);

Plotly.newPlot("distancePlot", distanceTraces, distanceLayoutBase, {
  responsive: true,
  displayModeBar: false,
});

Plotly.newPlot("seeThroughPlot", seeThroughTraces, distanceLayoutBase, {
  responsive: true,
  displayModeBar: false,
});

buildRouteControls();
  </script>
</body>
</html>
""".replace("__DATA__", data_json)

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"[HTML] saved: {output}")


def main() -> None:
    t0 = time.time()
    print("=== Main3: line + tangent arc engineering route ===")
    route_result = load_json(settings.BIDIR_RESULT_JSON)
    source_routes = selected_source_routes(route_result)
    planning_space = load_json(settings.SPACE_JSON)
    validation_extra = float(getattr(settings, "ENGINEERING_VALIDATION_EXTRA_CLEARANCE", 2.0))
    if validation_extra < 0.0:
        raise ValueError("ENGINEERING_VALIDATION_EXTRA_CLEARANCE must be non-negative")
    collision_model = DualSectionModel(
        section_polygons(planning_space, "x", extra_clearance=validation_extra),
        section_polygons(planning_space, "y", extra_clearance=validation_extra),
    )
    params = {
        "bend_radius": float(getattr(settings, "ENGINEERING_BEND_RADIUS", 150.0)),
        "max_bend_angle_deg": float(getattr(settings, "ENGINEERING_MAX_BEND_ANGLE_DEG", 45.0)),
        "min_corner_angle_deg": float(getattr(settings, "ENGINEERING_MAX_BEND_ANGLE_DEG", 90.0)),
        "arc_sample_angle_deg": float(getattr(settings, "ENGINEERING_ARC_SAMPLE_ANGLE_DEG", 4.0)),
        "max_turn_count": int(getattr(settings, "ENGINEERING_MAX_TURN_COUNT", 4)),
        "min_bend_radius": float(getattr(settings, "ENGINEERING_MIN_BEND_RADIUS", 25.0)),
        "bend_radius_step": float(getattr(settings, "ENGINEERING_BEND_RADIUS_STEP", 10.0)),
        "collision_model": collision_model,
        "simplify_tolerance": float(getattr(settings, "ENGINEERING_SIMPLIFY_TOLERANCE", 25.0)),
        "z_tolerance": float(getattr(settings, "ENGINEERING_Z_SIMPLIFY_TOLERANCE", 80.0)),
        "min_segment_length": float(getattr(settings, "ENGINEERING_MIN_SEGMENT_LENGTH", 40.0)),
        "s_bend_min_turn_deg": float(getattr(settings, "ENGINEERING_S_BEND_MIN_TURN_DEG", 110.0)),
        "s_bend_max_span": float(getattr(settings, "ENGINEERING_S_BEND_MAX_SPAN", 420.0)),
        "s_bend_min_detour": float(getattr(settings, "ENGINEERING_S_BEND_MIN_DETOUR", 60.0)),
        "wiggle_smooth_route_names": set(getattr(settings, "ENGINEERING_WIGGLE_SMOOTH_ROUTE_NAMES", [])),
        "wiggle_min_turn_deg": float(getattr(settings, "ENGINEERING_WIGGLE_MIN_TURN_DEG", 55.0)),
        "wiggle_max_span": float(getattr(settings, "ENGINEERING_WIGGLE_MAX_SPAN", 430.0)),
        "wiggle_min_detour": float(getattr(settings, "ENGINEERING_WIGGLE_MIN_DETOUR", 45.0)),
        "pipe_radius": float(getattr(settings, "PIPE_RADIUS", 25.0)),
        "tube_segments": int(getattr(settings, "ENGINEERING_TUBE_SEGMENTS", 16)),
        "hanger_points": settings_hanger_points(settings),
        "hanger_distance_range": float(getattr(settings, "HANGER_POINT_DISTANCE_RANGE", 100.0)),
    }

    engineered_routes = []
    failed_routes = []
    for idx, source_route in enumerate(source_routes):
        color = ROUTE_COLORS[idx % len(ROUTE_COLORS)]
        try:
            engineered_routes.append(engineer_route(source_route, color, params))
        except RuntimeError as exc:
            failed_routes.append({
                "name": source_route.get("name", f"route_{idx + 1}"),
                "reason": str(exc),
            })
            print(f"[SKIP] {source_route.get('name', f'route_{idx + 1}')}: {exc}")
    if not engineered_routes:
        raise RuntimeError("no engineered route satisfies the current hard constraints")

    result = {
        "method": "line_arc_tangent_engineering_route",
        "source_route_json": str(settings.BIDIR_RESULT_JSON),
        "source_route_name": getattr(settings, "ENGINEERING_SOURCE_ROUTE_NAME", None),
        "route_count": len(engineered_routes),
        "failed_route_count": len(failed_routes),
        "failed_routes": failed_routes,
        "bend_radius": params["bend_radius"],
        "max_bend_angle_deg": params["max_bend_angle_deg"],
        "required_min_corner_angle_deg": params["min_corner_angle_deg"],
        "arc_sample_angle_deg": params["arc_sample_angle_deg"],
        "simplify_tolerance": params["simplify_tolerance"],
        "z_simplify_tolerance": params["z_tolerance"],
        "hanger_point_distance_range": params["hanger_distance_range"],
        "hanger_points": params["hanger_points"].tolist(),
        "routes": engineered_routes,
    }

    print("[SAVE] engineered route json...")
    save_json(result, settings.ENGINEERING_RESULT_JSON)
    export_engineering_html(result, settings.ENGINEERING_HTML)

    elapsed = time.time() - t0
    print("=== Done ===")
    print(f"ENGINEERING_RESULT_JSON: {settings.ENGINEERING_RESULT_JSON}")
    print(f"ENGINEERING_HTML: {settings.ENGINEERING_HTML}")
    print(f"engineered route count: {len(engineered_routes)}")
    for route in engineered_routes:
        hanger = route.get("hanger_distance", {})
        hanger_text = ""
        if hanger.get("count", 0):
            hanger_text = (
                f", hanger={hanger.get('satisfied_count', 0)}/{hanger.get('count', 0)} "
                f"max={hanger.get('max_distance', float('nan')):.1f}mm"
            )
        print(
            f"{route['name']}: guide={len(route['guide_path'])}, "
            f"z trend={route['z_simplify_points']}, control={len(route['control_path'])}, "
            f"s_removed={route['s_bend_removed_points']}, "
            f"wiggle_removed={route['wiggle_removed_points']}, "
            f"samples={len(route['engineered_path'])}, arcs={route['arc_count']}, "
            f"main_turns={route['main_turn_count']}/{route['max_turn_count']} "
            f"(total={route['turn_count']}), "
            f"z variation={route['z_total_variation_before']:.1f}->{route['z_total_variation_after']:.1f}"
            f"{hanger_text}"
        )
    print(f"elapsed={elapsed:.1f}s")


if __name__ == "__main__":
    main()
