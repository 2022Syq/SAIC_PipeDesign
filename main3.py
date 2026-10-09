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


def active_constraint_profile() -> dict:
    """Resolve the selected air-conditioning pipe rules for this run.

    The planning space still uses one conservative route envelope.  Segment
    assignments are carried in the output for the engineer's next test; until
    coordinates are supplied, engineering uses the active/default core spec.
    """
    spec_id = str(getattr(settings, "ACTIVE_PIPE_SPEC", ""))
    specs = getattr(settings, "PIPE_SPECS", {})
    if spec_id not in specs:
        raise ValueError(f"ACTIVE_PIPE_SPEC is not defined: {spec_id}")
    spec = dict(specs[spec_id])
    if spec.get("kind") != "metal":
        raise ValueError("ACTIVE_PIPE_SPEC must identify the continuous metal core")
    spec_ids = sorted(getattr(settings, "selected_pipe_spec_ids", lambda: {spec_id})())
    # 当前运行按一个主动金属规格工程化；胶管规格先保留在分段元数据中，
    # 等工程师提供区间后再对局部中心线应用胶管半径。
    preferred = max(
        float(spec.get("recommended_bend_radius", 0.0)),
        float(spec.get("minimum_bend_radius", 0.0)),
    )
    hard_min = float(spec.get("minimum_bend_radius", preferred))
    cover_ids = [
        str(segment.get("cover_spec"))
        for segment in getattr(settings, "PIPE_SEGMENTS", [])
        if segment.get("cover_spec")
    ]
    return {
        "active_spec": spec_id,
        "core_spec": spec_id,
        "cover_spec": cover_ids[0] if len(cover_ids) == 1 else cover_ids,
        "kind": spec.get("kind", "metal"),
        "flexibility": spec.get("flexibility", "rigid"),
        "preferred_bend_radius": preferred,
        "minimum_bend_radius": hard_min,
        "envelope_radius": float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", float(spec["outer_diameter"]) / 2.0)),
        "core_outer_radius": float(spec["outer_diameter"]) / 2.0,
        "straight_recommended": spec.get("straight_recommended"),
        "straight_minimum": spec.get("straight_minimum"),
        "spec_ids": spec_ids,
        "pipe_segments": list(getattr(settings, "PIPE_SEGMENTS", [])),
    }


def turn_angle_degrees(prev_pt: np.ndarray, vertex: np.ndarray, next_pt: np.ndarray) -> float:
    """Return the physical direction change at a control point."""
    incoming = norm(np.asarray(vertex, dtype=float) - np.asarray(prev_pt, dtype=float))
    outgoing = norm(np.asarray(next_pt, dtype=float) - np.asarray(vertex, dtype=float))
    dot = max(-1.0, min(1.0, float(np.dot(incoming, outgoing))))
    return float(math.degrees(math.acos(dot)))


def merge_small_turns(path: np.ndarray, minimum_turn_deg: float) -> np.ndarray:
    """Remove small interior turns without changing the endpoint directions."""
    points = remove_close_points(np.asarray(path, dtype=float), 1e-6)
    if len(points) <= 2:
        return points
    changed = True
    while changed and len(points) > 2:
        changed = False
        keep = [points[0]]
        for index in range(1, len(points) - 1):
            # 两端相邻控制点定义用户指定的切向，不能按小角度抖动删除。
            if index == 1 or index == len(points) - 2:
                keep.append(points[index])
                continue
            angle = turn_angle_degrees(points[index - 1], points[index], points[index + 1])
            if angle + 1e-6 < float(minimum_turn_deg):
                changed = True
                continue
            keep.append(points[index])
        keep.append(points[-1])
        points = np.asarray(keep, dtype=float)
    return points


def adjust_endpoint_small_turns(path: np.ndarray, minimum_turn_deg: float) -> np.ndarray:
    """Slide a small end bend along its prescribed tangent to the minimum angle."""
    points = np.asarray(path, dtype=float).copy()
    if len(points) < 4:
        return points
    for endpoint, anchor, connector in ((0, 1, 2), (-1, -2, -3)):
        turn = turn_angle_degrees(points[endpoint], points[anchor], points[connector])
        if not 1e-3 <= turn < minimum_turn_deg:
            continue
        direction = norm(points[anchor] - points[endpoint])
        offset = points[connector] - points[endpoint]
        axial = float(np.dot(offset, direction))
        lateral = float(np.linalg.norm(offset - axial * direction))
        # 30 mm 是规划锚点长度，不是成形端部直线硬下限。只沿原切向移动
        # 转弯控制点，使圆角达到可制造最小弯角；端点与方向完全保持。
        distance = axial - lateral / math.tan(math.radians(minimum_turn_deg))
        if distance > 1e-6:
            points[anchor] = points[endpoint] + distance * direction
    return points


def physical_bends(segments: list[dict]) -> list[dict]:
    """Group contiguous pieces of the same circle into one manufacturing bend."""
    bends = []
    for index, segment in enumerate(segments):
        if segment.get("type") != "arc":
            continue
        if (bends and bends[-1]["last_segment"] == index - 1
                and dist(bends[-1]["center"], segment["center"]) < 1e-5
                and abs(bends[-1]["radius"] - float(segment["radius"])) < 1e-6):
            bends[-1]["last_segment"] = index
            bends[-1]["angle_deg"] += abs(float(segment["angle_deg"]))
        else:
            bends.append({
                "first_segment": index, "last_segment": index,
                "center": segment["center"], "radius": float(segment["radius"]),
                "angle_deg": abs(float(segment["angle_deg"])),
            })
    return bends


def project_point_to_path(point, path: np.ndarray) -> tuple[float, float]:
    """Return (distance along centerline, distance to centerline)."""
    point = np.asarray(point, dtype=float)
    best_s = 0.0
    best_d = float("inf")
    lengths = cumulative_lengths(path)
    for index, (a, b) in enumerate(zip(path[:-1], path[1:])):
        ab = b - a
        denom = float(np.dot(ab, ab))
        t = 0.0 if denom < 1e-12 else max(0.0, min(1.0, float(np.dot(point - a, ab) / denom)))
        candidate = a + t * ab
        distance = dist(point, candidate)
        if distance < best_d:
            best_d = distance
            best_s = float(lengths[index] + t * math.sqrt(denom))
    return best_s, best_d


def validate_route_constraints(path: np.ndarray, engineered: dict, profile: dict, collision_model=None) -> dict:
    """Check the confirmed air-conditioning constraints on one centerline."""
    path = np.asarray(path, dtype=float)
    hard_failures = []
    warnings = []

    arc_radii = [float(seg.get("radius", 0.0)) for seg in engineered.get("segments", []) if seg.get("type") == "arc"]
    minimum_radius = min(arc_radii) if arc_radii else None
    if minimum_radius is not None and minimum_radius + 1e-6 < profile["minimum_bend_radius"]:
        hard_failures.append({
            "rule": "bend_radius",
            "actual_mm": minimum_radius,
            "minimum_mm": profile["minimum_bend_radius"],
        })

    # 验收真实成形弯角，不能以控制点已简化代替验收；同圆分弧仍是一个弯。
    bends = physical_bends(engineered.get("segments", []))
    minimum_bend_angle = float(getattr(settings, "BEND_ANGLE_MINIMUM_DEG", 5.0))
    recommended_bend_angle = float(getattr(settings, "BEND_ANGLE_RECOMMENDED_DEG", 10.0))
    for index, bend in enumerate(bends):
        if bend["angle_deg"] + 1e-6 < minimum_bend_angle:
            hard_failures.append({"rule": "bend_angle", "bend_index": index,
                                  "actual_deg": bend["angle_deg"], "minimum_deg": minimum_bend_angle})
        elif bend["angle_deg"] + 1e-6 < recommended_bend_angle:
            warnings.append({"rule": "bend_angle", "bend_index": index,
                             "actual_deg": bend["angle_deg"], "recommended_deg": recommended_bend_angle})
    for left, right in zip(bends[:-1], bends[1:]):
        line_length = sum(
            dist(middle["start"], middle["end"])
            for middle in engineered["segments"][left["last_segment"] + 1:right["first_segment"]]
            if middle.get("type") == "line"
        )
        required = profile.get("straight_minimum")
        recommended = profile.get("straight_recommended")
        if required is not None and line_length + 1e-6 < float(required):
            hard_failures.append({"rule": "bend_spacing", "actual_mm": line_length, "minimum_mm": float(required)})
        elif recommended is not None and line_length < float(recommended):
            warnings.append({"rule": "bend_spacing", "actual_mm": line_length, "recommended_mm": float(recommended)})

    # 焊点、阀座和铝套位置由 settings 明确提供后才执行，不从 CATProduct 猜测。
    for point in getattr(settings, "WELD_POINTS", []):
        s, error = project_point_to_path(point, path)
        if error > 5.0:
            warnings.append({"rule": "weld_point_projection", "distance_mm": error, "s_mm": s})
    weld_s = sorted(project_point_to_path(point, path)[0] for point in getattr(settings, "WELD_POINTS", []))
    for a, b in zip(weld_s[:-1], weld_s[1:]):
        spacing = b - a
        if spacing < float(getattr(settings, "WELD_SPACING_MINIMUM", 25.0)):
            hard_failures.append({"rule": "weld_spacing", "actual_mm": spacing, "minimum_mm": float(getattr(settings, "WELD_SPACING_MINIMUM", 25.0))})
        elif spacing < float(getattr(settings, "WELD_SPACING_RECOMMENDED", 30.0)):
            warnings.append({"rule": "weld_spacing", "actual_mm": spacing, "recommended_mm": float(getattr(settings, "WELD_SPACING_RECOMMENDED", 30.0))})

    for point in getattr(settings, "VALVE_SEATS", []):
        valve_s, error = project_point_to_path(point, path)
        if error > 5.0:
            warnings.append({"rule": "valve_seat_projection", "distance_mm": error})
        bend_intervals = []
        segment_s = 0.0
        for segment in engineered.get("segments", []):
            if segment.get("type") == "arc":
                arc_start = segment_s
                segment_s += abs(math.radians(float(segment.get("angle_deg", 0.0))) * float(segment.get("radius", 0.0)))
                bend_intervals.append((arc_start, segment_s))
            else:
                segment_s += dist(segment["start"], segment["end"])
        left_boundaries = [end for _start, end in bend_intervals if end <= valve_s]
        right_boundaries = [start for start, _end in bend_intervals if start >= valve_s]
        left = valve_s - max(left_boundaries or [0.0])
        right = min(right_boundaries or [path_length(path)]) - valve_s
        minimum = float(getattr(settings, "VALVE_STRAIGHT_MINIMUM", 5.0))
        recommended = float(getattr(settings, "VALVE_STRAIGHT_RECOMMENDED", 25.0))
        for side, distance in (("left", left), ("right", right)):
            if distance < minimum:
                hard_failures.append({"rule": "valve_straight", "side": side, "actual_mm": distance, "minimum_mm": minimum})
            elif distance < recommended:
                warnings.append({"rule": "valve_straight", "side": side, "actual_mm": distance, "recommended_mm": recommended})

    sleeve_specs = getattr(settings, "ALUMINUM_SLEEVES", [])
    for sleeve in sleeve_specs:
        spec_id = str(sleeve.get("spec", profile["active_spec"]))
        lengths = getattr(settings, "ALUMINUM_SLEEVE_BACK_LENGTHS", {}).get(spec_id)
        if lengths is None:
            warnings.append({"rule": "aluminum_sleeve_back_length_missing", "spec": spec_id})
        else:
            actual = float(sleeve.get("back_length", 0.0))
            if actual < float(lengths[1]):
                hard_failures.append({"rule": "aluminum_sleeve_back_length", "actual_mm": actual, "minimum_mm": float(lengths[1]), "spec": spec_id})
            elif actual < float(lengths[0]):
                warnings.append({"rule": "aluminum_sleeve_back_length", "actual_mm": actual, "recommended_mm": float(lengths[0]), "spec": spec_id})

        start_s = sleeve.get("start_s")
        end_s = sleeve.get("end_s")
        if start_s is None or end_s is None:
            start_point = sleeve.get("start_point")
            end_point = sleeve.get("end_point")
            if start_point is not None and end_point is not None:
                start_s = project_point_to_path(start_point, path)[0]
                end_s = project_point_to_path(end_point, path)[0]
        if start_s is None or end_s is None:
            warnings.append({"rule": "aluminum_sleeve_location_missing", "spec": spec_id})
            continue
        if collision_model is None:
            warnings.append({"rule": "aluminum_sleeve_clearance_unchecked", "spec": spec_id})
            continue
        recommended_clearance = float(getattr(settings, "ALUMINUM_SLEEVE_AXIS_CLEARANCE_RECOMMENDED", 80.0))
        minimum_clearance = float(getattr(settings, "ALUMINUM_SLEEVE_AXIS_CLEARANCE_MINIMUM", 65.0))
        lo_s, hi_s = sorted([float(start_s), float(end_s)])
        path_s = cumulative_lengths(path)
        for index, point in enumerate(path):
            if not lo_s - 1e-6 <= path_s[index] <= hi_s + 1e-6:
                continue
            best = float("inf")
            best_name = None
            x_index = int(np.argmin(np.abs(collision_model.x_values - float(point[0]))))
            x_raw = collision_model.x_sections[x_index].get("raw", {})
            for component in x_raw.get("hard_obstacle_section_lines", []):
                distance = distance_to_component_lines([float(point[1]), float(point[2])], component)
                if distance < best:
                    best, best_name = distance, component.get("name")
            y_index = int(np.argmin(np.abs(collision_model.y_values - float(point[1]))))
            y_raw = collision_model.y_sections[y_index].get("raw", {})
            for component in y_raw.get("hard_obstacle_section_lines", []):
                distance = distance_to_component_lines([float(point[0]), float(point[2])], component)
                if distance < best:
                    best, best_name = distance, component.get("name")
            if best < minimum_clearance:
                hard_failures.append({"rule": "aluminum_sleeve_axis_clearance", "actual_mm": best, "minimum_mm": minimum_clearance, "component": best_name})
            elif best < recommended_clearance:
                warnings.append({"rule": "aluminum_sleeve_axis_clearance", "actual_mm": best, "recommended_mm": recommended_clearance, "component": best_name})

    return {
        "valid": not hard_failures,
        "hard_failures": hard_failures,
        "warnings": warnings,
        "active_spec": profile["active_spec"],
        "minimum_bend_radius": profile["minimum_bend_radius"],
        "minimum_arc_radius": minimum_radius,
        "minimum_bend_angle_deg": minimum_bend_angle,
        "minimum_actual_bend_angle_deg": min((bend["angle_deg"] for bend in bends), default=None),
        "aluminum_sleeve_axis_clearance_recommended": float(getattr(settings, "ALUMINUM_SLEEVE_AXIS_CLEARANCE_RECOMMENDED", 80.0)),
        "aluminum_sleeve_axis_clearance_minimum": float(getattr(settings, "ALUMINUM_SLEEVE_AXIS_CLEARANCE_MINIMUM", 65.0)),
    }


def validate_engineered_geometry(segments: list[dict], start, goal, start_direction, goal_direction) -> dict:
    """Measure the actual line/arc geometry, including endpoint and G1 continuity."""
    position_tolerance = 1e-5
    angle_tolerance = 1e-4

    def tangent(segment: dict, at_end: bool) -> np.ndarray:
        a = np.asarray(segment["start"], dtype=float)
        b = np.asarray(segment["end"], dtype=float)
        if segment["type"] == "line":
            return norm(b - a)
        center = np.asarray(segment["center"], dtype=float)
        normal = norm(np.cross(a - center, b - center))
        return norm(np.cross(normal, (b if at_end else a) - center))

    def angle(a, b) -> float:
        a, b = norm(a), norm(b)
        if float(np.linalg.norm(a)) < 1e-9 or float(np.linalg.norm(b)) < 1e-9:
            return 180.0
        return float(math.degrees(math.atan2(float(np.linalg.norm(np.cross(a, b))), float(np.dot(a, b)))))

    if not segments:
        return {"valid": False, "violations": [{"rule": "empty_geometry"}]}

    start_error = dist(segments[0]["start"], start)
    goal_error = dist(segments[-1]["end"], goal)
    start_angle = angle(tangent(segments[0], False), start_direction)
    goal_angle = angle(tangent(segments[-1], True), goal_direction)
    joints = [
        {
            "after_segment": index,
            "gap_mm": dist(left["end"], right["start"]),
            "tangent_error_deg": angle(tangent(left, True), tangent(right, False)),
        }
        for index, (left, right) in enumerate(zip(segments[:-1], segments[1:]), start=1)
    ]
    violations = []
    for side, error, tangent_error in (("start", start_error, start_angle), ("goal", goal_error, goal_angle)):
        if error > position_tolerance:
            violations.append({"rule": "endpoint_position", "side": side, "error_mm": error})
        if tangent_error > angle_tolerance:
            violations.append({"rule": "endpoint_direction", "side": side, "error_deg": tangent_error})
    for joint in joints:
        if joint["gap_mm"] > position_tolerance:
            violations.append({"rule": "joint_gap", **joint})
        if joint["tangent_error_deg"] > angle_tolerance:
            violations.append({"rule": "joint_tangent", **joint})
    return {
        "valid": not violations,
        "position_tolerance_mm": position_tolerance,
        "angle_tolerance_deg": angle_tolerance,
        "start_position_error_mm": start_error,
        "goal_position_error_mm": goal_error,
        "start_tangent_error_deg": start_angle,
        "goal_tangent_error_deg": goal_angle,
        "start_straight_length_mm": dist(segments[0]["start"], segments[0]["end"]) if segments[0]["type"] == "line" else 0.0,
        "goal_straight_length_mm": dist(segments[-1]["start"], segments[-1]["end"]) if segments[-1]["type"] == "line" else 0.0,
        "max_joint_gap_mm": max((joint["gap_mm"] for joint in joints), default=0.0),
        "max_joint_tangent_error_deg": max((joint["tangent_error_deg"] for joint in joints), default=0.0),
        "joints": joints,
        "violations": violations,
    }


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

        # 首末锚点也必须圆角，否则 30 mm 方向直线与后续导引线之间会留下尖角。
        # 圆角沿已有端部直线切入，不移动端点，也不改变其切向。
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
    constraint_profile = params.get("constraint_profile") or active_constraint_profile()

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
        # Preserve its connector while choosing the middle portion. The anchor
        # may subsequently slide along its prescribed tangent for a small bend;
        # every resulting line/arc candidate still undergoes clearance checks.
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

    # 合并内部小角度抖动，保留决定起终点方向的首末控制点。
    control = merge_small_turns(
        control,
        float(getattr(settings, "BEND_ANGLE_MINIMUM_DEG", 5.0)),
    )
    control_before_s_bend = control.copy()

    preferred_radius = float(constraint_profile["preferred_bend_radius"])
    minimum_radius = float(constraint_profile["minimum_bend_radius"])
    radius_step = max(float(params["bend_radius_step"]), 1.0)
    radii = []
    radius = preferred_radius
    while radius > minimum_radius + 1e-9:
        radii.append(radius)
        radius -= radius_step
    radii.append(minimum_radius)

    control_options = [remove_close_points(control, 1e-6)]
    fallback_control = merge_small_turns(
        remove_close_points(guide, 1e-6),
        float(getattr(settings, "BEND_ANGLE_MINIMUM_DEG", 5.0)),
    )
    if len(fallback_control) <= max_turn_count + 2 and not np.array_equal(control_options[0], fallback_control):
        control_options.append(fallback_control)
    control_options = [
        adjust_endpoint_small_turns(option, float(getattr(settings, "BEND_ANGLE_MINIMUM_DEG", 5.0)))
        for option in control_options
    ]

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
            collision_report = (
                joint_validate_path(collision_model, candidate["sampled_path"])
                if collision_model is not None else None
            )
            clearance_ok = collision_report is None or (
                collision_report["hard_bad_segments"] == 0
                and (allow_soft or collision_report["soft_bad_segments"] == 0)
            )
            geometry_report = validate_engineered_geometry(
                candidate["segments"], settings.START, settings.GOAL,
                np.asarray(settings.START_DIR_POINT) - np.asarray(settings.START),
                np.asarray(settings.GOAL_DIR_POINT) - np.asarray(settings.GOAL),
            )
            constraint_report = validate_route_constraints(
                candidate["sampled_path"], candidate, constraint_profile, collision_model=collision_model,
            )
            if (candidate["corner_angle_valid"] and clearance_ok
                    and constraint_report["valid"] and geometry_report["valid"]):
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
            f"corner_angle>={params['min_corner_angle_deg']:.1f} deg, "
            f"bend_angle>={float(getattr(settings, 'BEND_ANGLE_MINIMUM_DEG', 5.0)):.1f} deg, "
            "manufacturing spacing, and continuous endpoint-aligned geometry"
        )
    control = used_control
    control_before_s_bend = used_control
    sampled = engineered["sampled_path"]
    tube = tube_mesh(
        sampled,
        float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", constraint_profile["envelope_radius"])),
        params["tube_segments"],
    )
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
        "active_pipe_spec": constraint_profile["active_spec"],
        "core_spec": constraint_profile["core_spec"],
        "cover_spec": constraint_profile["cover_spec"],
        "pipe_spec_ids": constraint_profile["spec_ids"],
        "core_outer_radius": constraint_profile["core_outer_radius"],
        "route_max_envelope_radius": float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", constraint_profile["envelope_radius"])),
        "pipe_segments": constraint_profile["pipe_segments"],
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
        "constraint_report": constraint_report,
        "geometry_report": geometry_report,
        "collision_report": collision_report,
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
    constraint_profile = active_constraint_profile()
    params = {
        "constraint_profile": constraint_profile,
        "bend_radius": constraint_profile["preferred_bend_radius"],
        "max_bend_angle_deg": float(getattr(settings, "ENGINEERING_MAX_BEND_ANGLE_DEG", 45.0)),
        "min_corner_angle_deg": float(getattr(settings, "ENGINEERING_MAX_BEND_ANGLE_DEG", 90.0)),
        "arc_sample_angle_deg": float(getattr(settings, "ENGINEERING_ARC_SAMPLE_ANGLE_DEG", 4.0)),
        "max_turn_count": int(getattr(settings, "ENGINEERING_MAX_TURN_COUNT", 4)),
        "min_bend_radius": constraint_profile["minimum_bend_radius"],
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
        "pipe_radius": constraint_profile["envelope_radius"],
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
        "active_pipe_spec": constraint_profile["active_spec"],
        "pipe_spec_ids": constraint_profile["spec_ids"],
        "route_max_envelope_radius": float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", constraint_profile["envelope_radius"])),
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
