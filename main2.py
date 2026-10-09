# -*- coding: utf-8 -*-
"""
Joint X/Y-section exhaust pipe route planner.

Outputs:
- outputs/route_bidirectional.json
- outputs/route_bidirectional.html
- outputs/route_bidirectional_distance.png
"""

from __future__ import annotations

import json
import heapq
import itertools
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

import settings
from common_space import html_aspect_ratio, html_x_range, load_json, load_step_meshes_for_html, save_json
from hanger_utils import hanger_distance_report, settings_hanger_points


def import_shapely():
    try:
        from shapely.geometry import Point, Polygon
        from shapely.ops import nearest_points
        return Point, Polygon, nearest_points
    except Exception as exc:
        raise ImportError("缺少 shapely，请安装：python -m pip install shapely") from exc


Point, Polygon, nearest_points = import_shapely()


OUTPUT_DIR = Path(getattr(settings, "OUTPUT_DIR", settings.PROJECT_DIR / "outputs"))
BIDIR_RESULT_JSON = Path(getattr(settings, "BIDIR_RESULT_JSON", OUTPUT_DIR / "route_bidirectional.json"))
BIDIR_HTML = Path(getattr(settings, "BIDIR_HTML", OUTPUT_DIR / "route_bidirectional.html"))
BIDIR_DISTANCE_PNG = Path(getattr(settings, "BIDIR_DISTANCE_PNG", OUTPUT_DIR / "route_bidirectional_distance.png"))

STATE_KEEP = int(getattr(settings, "BIDIR_STATE_KEEP", 12))
SEGMENT_SAMPLE_STEP = float(getattr(settings, "ROUTE_SEGMENT_SAMPLE_STEP", 8.0))
LOOKAHEAD_SECTIONS = int(getattr(settings, "BIDIR_LOOKAHEAD_SECTIONS", 4))

SOFT_BAD_WEIGHT = float(getattr(settings, "SOFT_BAD_WEIGHT", 18000.0))
ENGINEER_SOFT_BAD_WEIGHT = float(getattr(settings, "ENGINEER_SOFT_BAD_WEIGHT", 1800.0))
HARD_BAD_WEIGHT = float(getattr(settings, "HARD_BAD_WEIGHT", 260000.0))
BEND_ANGLE_RECOMMENDED_DEG = float(getattr(settings, "BEND_ANGLE_RECOMMENDED_DEG", 10.0))
BEND_ANGLE_MINIMUM_DEG = float(getattr(settings, "BEND_ANGLE_MINIMUM_DEG", 5.0))

SMALL_STEP = float(getattr(settings, "BIDIR_SMALL_STEP", 42.0))
MEDIUM_STEP = float(getattr(settings, "BIDIR_MEDIUM_STEP", 78.0))
LARGE_Y_STEP = float(getattr(settings, "BIDIR_LARGE_Y_STEP", 125.0))

TRUNK_STEP_DY_LIMIT = float(getattr(settings, "BIDIR_TRUNK_STEP_DY_LIMIT", 45.0))
TRUNK_STEP_DZ_LIMIT = float(getattr(settings, "BIDIR_TRUNK_STEP_DZ_LIMIT", 28.0))
TRUNK_CLEAR_STRAIGHT_BONUS = float(getattr(settings, "BIDIR_TRUNK_CLEAR_STRAIGHT_BONUS", 520.0))

MIN_BRANCH_PROGRESS = float(getattr(settings, "BIDIR_MIN_BRANCH_PROGRESS", 0.58))
BRANCH_FORCE_AFTER_PROGRESS = float(getattr(settings, "BIDIR_BRANCH_FORCE_AFTER_PROGRESS", 0.86))
BRANCH_BLEND_DISTANCE = float(getattr(settings, "BIDIR_BRANCH_BLEND_DISTANCE", 520.0))
BRANCH_TRIGGER_AREA_RATIO = float(getattr(settings, "BIDIR_BRANCH_TRIGGER_AREA_RATIO", 0.30))

Z_OVERSHOOT_ALLOW = float(getattr(settings, "BIDIR_Z_OVERSHOOT_ALLOW", 28.0))
Z_UP_PENALTY = float(getattr(settings, "BIDIR_Z_UP_PENALTY", 18.0))
Z_ABOVE_GOAL_PENALTY = float(getattr(settings, "BIDIR_Z_ABOVE_GOAL_PENALTY", 120.0))
END_DIRECTION_WEIGHT = float(getattr(settings, "BIDIR_END_DIRECTION_WEIGHT", 1800.0))
DYNAMIC_CONNECT_DISTANCE = float(getattr(settings, "BIDIR_DYNAMIC_CONNECT_DISTANCE", 80.0))
DYNAMIC_MAX_STEPS = int(getattr(settings, "BIDIR_DYNAMIC_MAX_STEPS", 1000))


def dist(a, b) -> float:
    return float(np.linalg.norm(np.asarray(a, dtype=float) - np.asarray(b, dtype=float)))


def norm(v):
    v = np.asarray(v, dtype=float)
    n = float(np.linalg.norm(v))
    return v if n < 1e-9 else v / n


def angle_degrees(a, b) -> float:
    a = norm(a)
    b = norm(b)
    if np.linalg.norm(a) < 1e-9 or np.linalg.norm(b) < 1e-9:
        return 0.0
    c = max(-1.0, min(1.0, float(np.dot(a, b))))
    return float(math.degrees(math.acos(c)))


def path_length(path) -> float:
    pts = np.asarray(path, dtype=float)
    return float(sum(dist(a, b) for a, b in zip(pts[:-1], pts[1:])))


def remove_duplicate_points(path: np.ndarray, eps: float = 1e-6) -> np.ndarray:
    path = np.asarray(path, dtype=float)
    if len(path) <= 1:
        return path
    out = [path[0]]
    for p in path[1:]:
        if dist(out[-1], p) > eps:
            out.append(p)
    return np.asarray(out, dtype=float)


def sample_segment(a, b, step: float) -> list[np.ndarray]:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    length = dist(a, b)
    count = max(1, int(math.ceil(length / max(float(step), 1e-6))))
    return [a + (b - a) * (i / count) for i in range(count + 1)]


def interpolate_point_at_x(a, b, x: float) -> np.ndarray:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    dx = b[0] - a[0]
    if abs(dx) < 1e-9:
        return a.copy()
    t = (float(x) - a[0]) / dx
    return a + t * (b - a)


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


def smoothstep(t: float) -> float:
    t = max(0.0, min(1.0, float(t)))
    return t * t * (3.0 - 2.0 * t)


def half_progress(source: np.ndarray, target: np.ndarray, x: float) -> float:
    dx = float(target[0] - source[0])
    if abs(dx) < 1e-9:
        return 1.0
    return max(0.0, min(1.0, (float(x) - float(source[0])) / dx))


def get_start_dir() -> np.ndarray:
    start = np.asarray(settings.START, dtype=float)
    p = np.asarray(getattr(settings, "START_DIR_POINT", [start[0] + 10.0, start[1], start[2]]), dtype=float)
    return norm(p - start)


def get_goal_dir() -> np.ndarray:
    goal = np.asarray(settings.GOAL, dtype=float)
    p = np.asarray(getattr(settings, "GOAL_DIR_POINT", [goal[0] + 10.0, goal[1], goal[2]]), dtype=float)
    # Direction helper points consistently mean "from endpoint toward helper".
    # Therefore GOAL_DIR_POINT.y < GOAL.y requests a Y-negative arrival tangent.
    return norm(p - goal)


def load_space():
    if settings.SPACE_JSON.exists():
        return load_json(settings.SPACE_JSON)
    if settings.SPACE_CHECKPOINT_JSON.exists():
        return load_json(settings.SPACE_CHECKPOINT_JSON)
    raise FileNotFoundError("没有找到 outputs/planning_space.json 或 outputs/planning_space_checkpoint.json")


def section_polygons(space: dict, axis: str = "x", extra_clearance: float = 0.0) -> list[dict]:
    axis = str(axis).lower()
    records = space.get(f"{axis}_sections")
    if records is None and axis == "x":
        records = space.get("sections", [])
    records = records or []
    sections = []
    for sec in records:
        polys = []
        for reg in sec.get("free_regions", []):
            poly = Polygon(reg["polygon"], reg.get("holes", []))
            if not poly.is_valid:
                poly = poly.buffer(0)
            if extra_clearance > 0.0 and not poly.is_empty:
                poly = poly.buffer(-float(extra_clearance))
            if not poly.is_empty:
                polys.append(poly)
        area = sum(float(p.area) for p in polys)
        value = float(sec.get(axis, sec.get("value", 0.0)))
        item = {"axis": axis, "value": value, "polys": polys, "free_area": area, "raw": sec}
        item[axis] = value
        sections.append(item)
    return sections


def point_inside(section: dict, yz) -> bool:
    p = Point(float(yz[0]), float(yz[1]))
    return any(poly.contains(p) or poly.touches(p) for poly in section["polys"])


def nearest_free_point(section: dict, yz) -> Optional[np.ndarray]:
    p = Point(float(yz[0]), float(yz[1]))
    best = None
    best_d = 1e18
    for poly in section["polys"]:
        if poly.contains(p) or poly.touches(p):
            return np.array([float(p.x), float(p.y)], dtype=float)
        _a, b = nearest_points(p, poly)
        d = float(p.distance(b))
        if d < best_d:
            best_d = d
            best = np.array([float(b.x), float(b.y)], dtype=float)
    return best


def nearest_section_index(sections: list[dict], x: float) -> int:
    xs = np.array([s["x"] for s in sections], dtype=float)
    return int(np.argmin(np.abs(xs - float(x))))


def section_indices_between(sections: list[dict], x0: float, x1: float) -> list[int]:
    lo, hi = sorted([float(x0), float(x1)])
    return [i for i, s in enumerate(sections) if lo - 1e-6 <= s["x"] <= hi + 1e-6]


def hard_boundary_lines(section: dict) -> list[dict]:
    return section["raw"].get("hard_obstacle_section_lines", section["raw"].get("obstacle_section_lines", []))


def soft_boundary_lines(section: dict) -> list[dict]:
    return section["raw"].get("soft_obstacle_section_lines", [])


def point_hits_soft_boundary(section: dict, yz) -> tuple[bool, str | None]:
    if not bool(getattr(settings, "SOFT_OBSTACLE_ENABLED", True)):
        return False, None
    p = np.asarray(yz, dtype=float)
    best_name = None
    best_margin = 1e18
    for comp in soft_boundary_lines(section):
        buffer_radius = float(comp.get("buffer_radius", settings.PIPE_RADIUS + comp.get("clearance", 0.0)))
        for line in comp.get("lines", []):
            if len(line) < 2:
                continue
            for a, b in zip(line[:-1], line[1:]):
                d = point_segment_distance_2d(p, np.asarray(a, dtype=float), np.asarray(b, dtype=float))
                margin = d - buffer_radius
                if margin < best_margin:
                    best_margin = margin
                    best_name = comp.get("name", "soft")
                if d <= buffer_radius:
                    return True, comp.get("name", "soft")
    return False, best_name


def segment_check(sections: list[dict], a, b, sample_step: float = SEGMENT_SAMPLE_STEP) -> dict:
    hard_bad = 0
    soft_bad = 0
    total = 0
    names = []

    points = sample_segment(a, b, sample_step)
    for si in section_indices_between(sections, float(a[0]), float(b[0])):
        x = sections[si]["x"]
        points.append(interpolate_point_at_x(a, b, x))

    for p in points:
        si = nearest_section_index(sections, p[0])
        sec = sections[si]
        yz = [float(p[1]), float(p[2])]
        total += 1

        if not point_inside(sec, yz):
            hard_bad += 1
            names.append("hard_boundary")

        hit_soft, soft_name = point_hits_soft_boundary(sec, yz)
        if hit_soft:
            soft_bad += 1
            names.append(soft_name or "soft_boundary")

    return {
        "valid": hard_bad == 0 and soft_bad == 0,
        "engineering_valid": hard_bad == 0,
        "hard_bad": int(hard_bad),
        "soft_bad": int(soft_bad),
        "total": int(total),
        "names": names[:8],
    }


def local_area_ratio(sections: list[dict], idx: int, window: int = 6) -> float:
    lo = max(0, idx - window)
    hi = min(len(sections), idx + window + 1)
    areas = [float(s.get("free_area", 0.0)) for s in sections[lo:hi]]
    if not areas:
        return 1.0
    return float(sections[idx].get("free_area", 0.0)) / max(max(areas), 1.0)


@dataclass
class RouteProfile:
    name: str
    description: str
    allow_soft: bool
    meet_ratio: float
    meet_y_offset: float = 0.0
    meet_z_offset: float = 0.0
    y_bias: float = 0.0
    z_bias: float = 0.0
    goal_pull_w: float = 7.0
    length_w: float = 0.14
    turn_w: float = 7.0
    second_w: float = 12.0
    reverse_w: float = 850.0
    y_move_w: float = 3.6
    z_move_w: float = 3.8
    big_y_w: float = 55.0
    hard_w: float = HARD_BAD_WEIGHT
    soft_w: float = SOFT_BAD_WEIGHT
    joint_length_w: float = 1.0
    joint_turn_w: float = 0.0
    joint_diagonal_xy_w: float = 0.0


@dataclass
class State:
    cost: float
    point: np.ndarray
    prev: np.ndarray
    path: list[list[float]]
    logs: list[dict]
    branched: bool = False
    branch_x: Optional[float] = None


def make_profiles() -> list[RouteProfile]:
    return [
        RouteProfile(
            "01_bidir_balanced",
            "综合策略：硬约束优先，兼顾长度、转折数量和X/Y分布。",
            False,
            0.54,
            goal_pull_w=8.0,
            joint_length_w=1.0,
            joint_turn_w=18.0,
            joint_diagonal_xy_w=8.0,
        ),
        RouteProfile(
            "02_bidir_min_turn",
            "少转折策略：强惩罚方向变化，优先生成更少主转折点的路线。",
            False,
            0.54,
            goal_pull_w=7.2,
            joint_length_w=1.08,
            joint_turn_w=65.0,
            joint_diagonal_xy_w=10.0,
        ),
        RouteProfile(
            "03_bidir_shortest",
            "短距离策略：更重视总长度，允许为缩短距离接受更多局部调整。",
            False,
            0.52,
            goal_pull_w=9.0,
            joint_length_w=0.82,
            joint_turn_w=6.0,
            joint_diagonal_xy_w=3.0,
        ),
        RouteProfile(
            "04_bidir_axis_xy",
            "X/Y分布策略：抑制XY平面斜向步，倾向用X向和Y向分段完成绕行。",
            False,
            0.55,
            goal_pull_w=7.0,
            joint_length_w=1.02,
            joint_turn_w=14.0,
            joint_diagonal_xy_w=42.0,
        ),
        RouteProfile(
            "05_bidir_engineer_soft",
            "工程软边界策略：允许软边界惩罚通过，用于保留可工程评估的备选路线。",
            True,
            0.52,
            meet_z_offset=20.0,
            z_bias=12.0,
            goal_pull_w=9.5,
            length_w=0.12,
            turn_w=6.0,
            second_w=9.0,
            y_move_w=3.0,
            z_move_w=3.0,
            soft_w=ENGINEER_SOFT_BAD_WEIGHT,
        ),
    ]


def baseline_point(start: np.ndarray, goal: np.ndarray, ratio: float) -> np.ndarray:
    return start + float(ratio) * (goal - start)


def choose_meeting_point(profile: RouteProfile, sections: list[dict], start: np.ndarray, goal: np.ndarray) -> np.ndarray:
    base = baseline_point(start, goal, profile.meet_ratio)
    base[1] += profile.meet_y_offset
    base[2] += profile.meet_z_offset
    si = nearest_section_index(sections, base[0])
    sec = sections[si]
    yz = nearest_free_point(sec, [base[1], base[2]])
    if yz is None:
        yz = np.array([base[1], base[2]], dtype=float)
    return np.array([sec["x"], float(yz[0]), float(yz[1])], dtype=float)


def trunk_projection_point(source: np.ndarray, direction: np.ndarray, x: float) -> np.ndarray:
    d = norm(direction)
    if abs(float(d[0])) < 1e-9:
        return np.array([float(x), float(source[1]), float(source[2])], dtype=float)
    t = (float(x) - float(source[0])) / float(d[0])
    p = source + t * d
    return np.array([float(x), float(p[1]), float(p[2])], dtype=float)


def branch_blend_factor(state: State, x: float) -> float:
    if not state.branched or state.branch_x is None:
        return 0.0
    return smoothstep(abs(float(x) - float(state.branch_x)) / max(BRANCH_BLEND_DISTANCE, 1e-6))


def target_yz_for_state(profile: RouteProfile, state: State, source: np.ndarray, target: np.ndarray, direction: np.ndarray, x: float) -> np.ndarray:
    trunk = trunk_projection_point(source, direction, x)
    trunk_yz = np.array([trunk[1], trunk[2]], dtype=float)
    meet_yz = np.array([target[1] + profile.y_bias, target[2] + profile.z_bias], dtype=float)
    f = branch_blend_factor(state, x)
    return (1.0 - f) * trunk_yz + f * meet_yz


def projected_target_yz(current_yz: np.ndarray, target_yz: np.ndarray, max_dy: float, max_dz: float, scale: float = 1.0) -> np.ndarray:
    delta = target_yz - current_yz
    return current_yz + scale * np.array([
        float(np.clip(delta[0], -max_dy, max_dy)),
        float(np.clip(delta[1], -max_dz, max_dz)),
    ])


def straight_candidate_from_direction(source: np.ndarray, direction: np.ndarray, section_x: float) -> np.ndarray:
    p = trunk_projection_point(source, direction, section_x)
    return np.array([p[1], p[2]], dtype=float)


def straight_line_clear_ahead(sections: list[dict], section_index: int, current: np.ndarray, source: np.ndarray, direction: np.ndarray) -> tuple[bool, str]:
    max_i = min(len(sections) - 1, section_index + LOOKAHEAD_SECTIONS)
    for j in range(section_index, max_i + 1):
        sec = sections[j]
        yz = straight_candidate_from_direction(source, direction, sec["x"])
        p_free = nearest_free_point(sec, yz)
        if p_free is None:
            return False, "no_free_point"
        q = np.array([sec["x"], p_free[0], p_free[1]], dtype=float)
        chk = segment_check(sections, current, q)
        if chk["hard_bad"] > 0:
            return False, f"straight_hard={chk['hard_bad']}"
    return True, "straight_clear"


def should_trigger_branch(profile, sections, section_index, state, source, target, direction) -> tuple[bool, str]:
    if state.branched:
        return True, "already"

    section = sections[section_index]
    progress = half_progress(source, target, section["x"])

    clear, reason = straight_line_clear_ahead(sections, section_index, state.point, source, direction)

    # 关键：只要方向直行是安全的，就不允许分流，除非半程已经非常靠后。
    if clear and progress < BRANCH_FORCE_AFTER_PROGRESS:
        return False, "straight_clear_hold"

    if progress >= BRANCH_FORCE_AFTER_PROGRESS:
        return True, "force_progress"

    if not point_inside(section, [state.point[1], state.point[2]]):
        return True, "current_outside"

    area_ratio = local_area_ratio(sections, section_index)
    if (not clear) and progress >= MIN_BRANCH_PROGRESS:
        return True, reason

    if (not clear) and area_ratio < BRANCH_TRIGGER_AREA_RATIO:
        return True, f"{reason}|area={area_ratio:.2f}"

    return False, "hold"


def append_candidate(cands, section, kind: str, yz) -> None:
    p = nearest_free_point(section, yz)
    if p is None:
        return
    key = (kind, round(float(p[0]), 1), round(float(p[1]), 1))
    for old_key, _kind, _p in cands:
        if old_key == key:
            return
    cands.append((key, kind, p))


def generate_candidates(profile, sections, section_index, state, source, target, direction):
    section = sections[section_index]
    cur_yz = np.array([state.point[1], state.point[2]], dtype=float)

    trigger, reason = should_trigger_branch(profile, sections, section_index, state, source, target, direction)
    branched = state.branched or trigger
    branch_x = state.branch_x if state.branch_x is not None else (section["x"] if trigger else None)

    temp_state = State(state.cost, state.point, state.prev, state.path, state.logs, branched, branch_x)
    tgt_yz = target_yz_for_state(profile, temp_state, source, target, direction, section["x"])
    f = branch_blend_factor(temp_state, section["x"])

    cands = []

    trunk_yz = straight_candidate_from_direction(source, direction, section["x"])
    append_candidate(cands, section, "dir_trunk", trunk_yz)

    if point_inside(section, cur_yz):
        append_candidate(cands, section, "local_straight", cur_yz)

    # 分流前只允许很小的DZ，避免提前爬高
    for scale in (0.45, 0.75, 1.00):
        raw = projected_target_yz(
            cur_yz,
            tgt_yz,
            max_dy=TRUNK_STEP_DY_LIMIT + 45.0 * f,
            max_dz=TRUNK_STEP_DZ_LIMIT + 30.0 * f,
            scale=scale,
        )
        append_candidate(cands, section, f"target_projection_{scale:.2f}", raw)

    if branched:
        td = norm(tgt_yz - cur_yz)
        if np.linalg.norm(td) < 1e-9:
            td = np.array([0.0, 0.0])

        for length in (SMALL_STEP, SMALL_STEP * 1.25):
            append_candidate(cands, section, f"small_target_diag_{length:.0f}", cur_yz + td * length)

        # 斜向优先，减少纯Z/纯Y
        dirs = [
            norm(td + np.array([+0.55, +0.35])),
            norm(td + np.array([+0.55, -0.35])),
            norm(td + np.array([-0.35, +0.35])),
            norm(td + np.array([-0.35, -0.35])),
        ]
        if profile.z_bias < 0:
            dirs.append(norm(td + np.array([0.30, -0.80])))
        if profile.z_bias > 0 and f > 0.55:
            dirs.append(norm(td + np.array([0.30, +0.60])))
        if abs(profile.y_bias) > 1e-6:
            dirs.append(norm(td + np.array([math.copysign(0.75, profile.y_bias), 0.15])))

        for d in dirs:
            if np.linalg.norm(d) < 1e-9:
                continue
            for length in (MEDIUM_STEP,):
                append_candidate(cands, section, f"branch_diag_{length:.0f}", cur_yz + d * length)

        # Z兜底延后，且上升比下降更克制
        if f > 0.60:
            for dz in (-40.0, +30.0, -70.0):
                append_candidate(cands, section, f"z_fallback_{dz:+.0f}", cur_yz + np.array([0.0, dz]))

        if f > 0.70:
            for dy in (+LARGE_Y_STEP, -LARGE_Y_STEP):
                append_candidate(cands, section, f"large_y_{dy:+.0f}", cur_yz + np.array([dy, 0.0]))

    if len(cands) < 3:
        for poly in section["polys"]:
            rp = poly.representative_point()
            append_candidate(cands, section, "inside", np.array([float(rp.x), float(rp.y)]))

    return [(kind, p, branched, branch_x, reason) for _key, kind, p in cands]


def score_candidate(profile, sections, state, candidate, kind, source, target, direction, branched, branch_x, trigger_reason, end_direction=None):
    current = state.point
    prev = state.prev

    temp_state = State(state.cost, state.point, state.prev, state.path, state.logs, branched, branch_x)
    f = branch_blend_factor(temp_state, candidate[0])
    tgt_yz = target_yz_for_state(profile, temp_state, source, target, direction, candidate[0])
    trunk_yz = straight_candidate_from_direction(source, direction, candidate[0])

    step_len = dist(current, candidate)
    turn = angle_degrees(current - prev, candidate - current)
    second = float(np.linalg.norm((candidate - 2 * current + prev)[1:3]))

    dy = abs(float(candidate[1] - current[1]))
    dz = abs(float(candidate[2] - current[2]))

    reverse = 0
    for axis in (1, 2):
        a = current[axis] - prev[axis]
        b = candidate[axis] - current[axis]
        if abs(a) > 1e-6 and abs(b) > 1e-6 and a * b < 0:
            reverse += 1

    chk = segment_check(sections, current, candidate)

    if profile.allow_soft:
        bad_cost = chk["hard_bad"] * profile.hard_w + chk["soft_bad"] * profile.soft_w
    else:
        bad_cost = (chk["hard_bad"] + chk["soft_bad"]) * profile.hard_w

    target_dist = float(np.linalg.norm(candidate[1:3] - tgt_yz))
    target_cost = (0.25 + f) * profile.goal_pull_w * target_dist

    trunk_dist = float(np.linalg.norm(candidate[1:3] - trunk_yz))
    trunk_cost = (1.0 - f) * 13.0 * trunk_dist

    # Z 抑制：向上比向下更贵；高于目标高度太多更贵
    z_up = max(0.0, float(candidate[2] - current[2]))
    z_up_cost = Z_UP_PENALTY * z_up

    z_ref = max(float(source[2]), float(target[2])) + Z_OVERSHOOT_ALLOW
    z_above = max(0.0, float(candidate[2]) - z_ref)
    z_above_cost = Z_ABOVE_GOAL_PENALTY * z_above * z_above

    z_lower = min(float(source[2]), float(target[2])) - 120.0
    z_too_low = max(0.0, z_lower - float(candidate[2]))
    z_low_cost = 35.0 * z_too_low * z_too_low

    big_y_cost = profile.big_y_w * max(0.0, dy - 55.0) ** 2
    yz_move_cost = profile.y_move_w * dy + profile.z_move_w * dz

    axis_cost = 0.0
    if dy > 15.0 and dz < 8.0:
        axis_cost += 70.0 * (dy - dz)
    if dz > 15.0 and dy < 8.0:
        axis_cost += 100.0 * (dz - dy)

    diagonal_bonus = -70.0 * min(dy, dz) / max(dy, dz, 1e-6) if dy > 12.0 and dz > 12.0 else 0.0

    dir_cost = 0.0
    if end_direction is not None:
        seg_dir = candidate - current
        dir_angle = angle_degrees(seg_dir, end_direction)
        dir_cost = END_DIRECTION_WEIGHT * (dir_angle / 90.0) ** 2

    kind_bonus = 0.0
    if kind == "dir_trunk":
        kind_bonus = -TRUNK_CLEAR_STRAIGHT_BONUS * (1.0 - f) + 100.0 * f
    elif kind == "local_straight":
        kind_bonus = -180.0 * (1.0 - f) + 120.0 * f
    elif kind.startswith("target_projection"):
        kind_bonus = -100.0 * (0.2 + f)
    elif kind.startswith("small_target"):
        kind_bonus = -85.0 * f
    elif kind.startswith("branch"):
        kind_bonus = -70.0 * f
    elif kind.startswith("z_fallback"):
        kind_bonus = 80.0
    elif kind.startswith("large_y"):
        kind_bonus = 240.0
    elif kind == "connect_target":
        kind_bonus = -80.0

    small_angle_cost = 0.0
    if BEND_ANGLE_MINIMUM_DEG <= turn < BEND_ANGLE_RECOMMENDED_DEG:
        small_angle_cost = float(getattr(settings, "SMALL_BEND_PENALTY", 120.0))

    score = (
        profile.length_w * step_len
        + profile.turn_w * turn
        + profile.second_w * second
        + profile.reverse_w * reverse
        + yz_move_cost
        + big_y_cost
        + axis_cost
        + diagonal_bonus
        + target_cost
        + trunk_cost
        + z_up_cost
        + z_above_cost
        + z_low_cost
        + dir_cost
        + small_angle_cost
        + bad_cost
        + kind_bonus
    )

    log = {
        "x": float(candidate[0]),
        "kind": kind,
        "branched": bool(branched),
        "branch_x": None if branch_x is None else float(branch_x),
        "branch_factor": float(f),
        "trigger_reason": trigger_reason,
        "dy": float(dy),
        "dz": float(dz),
        "hard_bad": int(chk["hard_bad"]),
        "soft_bad": int(chk["soft_bad"]),
        "target_dist": float(target_dist),
        "trunk_dist": float(trunk_dist),
        "z_up_cost": float(z_up_cost),
        "z_above_cost": float(z_above_cost),
        "dir_cost": float(dir_cost),
        "turn": float(turn),
        "small_angle_cost": float(small_angle_cost),
        "second": float(second),
        "point": candidate.tolist(),
        "increment_cost": float(score),
    }
    return float(score), log


def plan_half(profile, all_sections, source, target, direction, label, terminal_direction=None):
    forward = target[0] >= source[0]
    xmin, xmax = sorted([float(source[0]), float(target[0])])
    sections = [s for s in all_sections if xmin - 1e-6 <= s["x"] <= xmax + 1e-6]
    sections.sort(key=lambda s: s["x"], reverse=not forward)

    pseudo_prev = source - norm(direction) * max(float(getattr(settings, "SECTION_DX", 20.0)), 1.0)
    states = [State(0.0, source.copy(), pseudo_prev.copy(), [source.tolist()], [], False, None)]

    print(f"[BIDIR:{profile.name}:{label}] start sections={len(sections)} state_keep={STATE_KEEP}")

    for si, section in enumerate(sections):
        if (forward and section["x"] <= source[0] + 1e-6) or ((not forward) and section["x"] >= source[0] - 1e-6):
            continue

        next_states = []
        for st in states:
            candidates = generate_candidates(profile, sections, si, st, source, target, direction)
            for kind, yz, branched, branch_x, trigger_reason in candidates:
                p = np.array([section["x"], yz[0], yz[1]], dtype=float)
                inc, log = score_candidate(profile, sections, st, p, kind, source, target, direction, branched, branch_x, trigger_reason)
                log["half"] = label
                next_states.append(State(st.cost + inc, p, st.point, st.path + [p.tolist()], st.logs + [log], branched, branch_x))

        if not next_states:
            print(f"[BIDIR:{profile.name}:{label}] section {si+1}/{len(sections)} no candidates")
            continue

        next_states.sort(key=lambda s: s.cost)

        kept = []
        seen = set()
        for st in next_states:
            key = (round(float(st.point[1]) / 25.0), round(float(st.point[2]) / 25.0), bool(st.branched))
            if key in seen:
                continue
            seen.add(key)
            kept.append(st)
            if len(kept) >= STATE_KEEP:
                break

        states = kept
        best = states[0]
        last = best.logs[-1]
        print(
            f"[BIDIR:{profile.name}:{label}] section {si+1:>3}/{len(sections)} "
            f"states={len(states):>2} cand={len(next_states):>4} "
            f"kind={last['kind']} branched={last['branched']} reason={last['trigger_reason']} "
            f"factor={last['branch_factor']:.2f} hard={last['hard_bad']} soft={last['soft_bad']} "
            f"zup={last['z_up_cost']:.0f} cost={best.cost:.1f}"
        )

    final_states = []
    for st in states:
        inc, log = score_candidate(
            profile, sections, st, target, "connect_target", source, target, direction,
            True, st.branch_x or target[0], "connect", end_direction=terminal_direction
        )
        log["half"] = label
        final_states.append(State(st.cost + inc, target.copy(), st.point, st.path + [target.tolist()], st.logs + [log], True, st.branch_x))

    final_states.sort(key=lambda s: s.cost)
    best = final_states[0]
    return {
        "path": remove_duplicate_points(np.asarray(best.path, dtype=float)).tolist(),
        "logs": best.logs,
        "cost": float(best.cost),
        "sections": len(sections),
        "branch_x": best.branch_x,
    }


def validate_path(sections: list[dict], path: np.ndarray) -> dict:
    hard_bad_segments = soft_bad_segments = hard_bad_samples = soft_bad_samples = total_samples = 0
    for a, b in zip(path[:-1], path[1:]):
        chk = segment_check(sections, a, b)
        total_samples += chk["total"]
        if chk["hard_bad"] > 0:
            hard_bad_segments += 1
        if chk["soft_bad"] > 0:
            soft_bad_segments += 1
        hard_bad_samples += chk["hard_bad"]
        soft_bad_samples += chk["soft_bad"]
    return {
        "valid": hard_bad_segments == 0 and soft_bad_segments == 0,
        "engineering_valid": hard_bad_segments == 0,
        "hard_bad_segments": int(hard_bad_segments),
        "soft_bad_segments": int(soft_bad_segments),
        "hard_bad_samples": int(hard_bad_samples),
        "soft_bad_samples": int(soft_bad_samples),
        "total_samples": int(total_samples),
    }


def movement_sections_between(sections: list[dict], x0: float, x1: float, forward: bool) -> list[dict]:
    lo, hi = sorted([float(x0), float(x1)])
    selected = [s for s in sections if lo - 1e-6 <= float(s["x"]) <= hi + 1e-6]
    selected.sort(key=lambda s: float(s["x"]), reverse=not forward)
    return selected


def next_section_index(movement_sections: list[dict], current_x: float, target_x: float, forward: bool) -> Optional[int]:
    eps = 1e-6
    for idx, sec in enumerate(movement_sections):
        x = float(sec["x"])
        if forward and current_x + eps < x < target_x - eps:
            return idx
        if (not forward) and target_x + eps < x < current_x - eps:
            return idx
    return None


def dedupe_states(states: list[State]) -> list[State]:
    states.sort(key=lambda s: s.cost)
    kept = []
    seen = set()
    dx = max(float(getattr(settings, "SECTION_DX", 20.0)), 1.0)
    for st in states:
        key = (
            round(float(st.point[0]) / dx),
            round(float(st.point[1]) / 25.0),
            round(float(st.point[2]) / 25.0),
            bool(st.branched),
        )
        if key in seen:
            continue
        seen.add(key)
        kept.append(st)
        if len(kept) >= STATE_KEEP:
            break
    return kept


def expand_dynamic_side(
    profile: RouteProfile,
    all_sections: list[dict],
    states: list[State],
    source: np.ndarray,
    target_state: State,
    direction: np.ndarray,
    label: str,
    forward: bool,
) -> tuple[list[State], bool]:
    best = states[0]
    target = target_state.point.copy()
    movement_sections = movement_sections_between(all_sections, best.point[0], target[0], forward)
    si = next_section_index(movement_sections, float(best.point[0]), float(target[0]), forward)
    if si is None:
        return states, False

    section = movement_sections[si]
    next_states = []
    for st in states:
        candidates = generate_candidates(profile, movement_sections, si, st, source, target, direction)
        for kind, yz, branched, branch_x, trigger_reason in candidates:
            p = np.array([section["x"], yz[0], yz[1]], dtype=float)
            inc, log = score_candidate(
                profile,
                movement_sections,
                st,
                p,
                kind,
                source,
                target,
                direction,
                branched,
                branch_x,
                trigger_reason,
            )
            log["half"] = label
            log["dynamic_target"] = target.tolist()
            next_states.append(
                State(
                    st.cost + inc,
                    p,
                    st.point,
                    st.path + [p.tolist()],
                    st.logs + [log],
                    branched,
                    branch_x,
                )
            )

    if not next_states:
        print(f"[BIDIR:{profile.name}:{label}] no candidates at x={section['x']:.1f}")
        return states, False

    kept = dedupe_states(next_states)
    best = kept[0]
    last = best.logs[-1]
    print(
        f"[BIDIR:{profile.name}:{label}] x={best.point[0]:.1f} "
        f"target_x={target[0]:.1f} states={len(kept):>2} cand={len(next_states):>4} "
        f"kind={last['kind']} branched={last['branched']} hard={last['hard_bad']} "
        f"soft={last['soft_bad']} cost={best.cost:.1f}"
    )
    return kept, True


def dynamic_sides_connected(left: np.ndarray, right: np.ndarray, start: np.ndarray, goal: np.ndarray) -> bool:
    if dist(left, right) <= DYNAMIC_CONNECT_DISTANCE:
        return True
    if start[0] <= goal[0]:
        return float(left[0]) >= float(right[0]) - 1e-6
    return float(left[0]) <= float(right[0]) + 1e-6


def build_bidirectional_route(profile, sections, start, goal, start_dir, goal_dir):
    print(f"\n--- Planning {profile.name} ---")
    print(f"[BIDIR:{profile.name}] dynamic alternating bidirectional search")
    print(f"[BIDIR:{profile.name}] start_dir={start_dir.tolist()}, goal_dir={goal_dir.tolist()}")

    forward = bool(start[0] <= goal[0])
    sections = sorted(sections, key=lambda s: float(s["x"]))
    step_dx = max(float(getattr(settings, "SECTION_DX", 20.0)), 1.0)

    left_prev = start - norm(start_dir) * step_dx
    left_states = [State(0.0, start.copy(), left_prev.copy(), [start.tolist()], [], False, None)]

    # GOAL端反向规划：从goal往外走用 -goal_dir，但最后拼接后进入goal应符合 goal_dir
    rear_source_dir = -goal_dir
    right_prev = goal - norm(rear_source_dir) * step_dx
    right_states = [State(0.0, goal.copy(), right_prev.copy(), [goal.tolist()], [], False, None)]

    print(
        f"[BIDIR:{profile.name}] connect_distance={DYNAMIC_CONNECT_DISTANCE:.1f} "
        f"max_steps={DYNAMIC_MAX_STEPS}"
    )

    moved_left = moved_right = True
    for step in range(1, DYNAMIC_MAX_STEPS + 1):
        left_best = left_states[0]
        right_best = right_states[0]
        if dynamic_sides_connected(left_best.point, right_best.point, start, goal):
            print(f"[BIDIR:{profile.name}] connected before step {step}")
            break

        left_states, moved_left = expand_dynamic_side(
            profile,
            sections,
            left_states,
            start,
            right_states[0],
            start_dir,
            "front",
            forward,
        )
        left_best = left_states[0]
        right_best = right_states[0]
        if dynamic_sides_connected(left_best.point, right_best.point, start, goal):
            print(f"[BIDIR:{profile.name}] connected after front step {step}")
            break

        right_states, moved_right = expand_dynamic_side(
            profile,
            sections,
            right_states,
            goal,
            left_states[0],
            rear_source_dir,
            "rear",
            not forward,
        )

        if not moved_left and not moved_right:
            print(f"[BIDIR:{profile.name}] stopped: neither side can advance")
            break
    else:
        print(f"[BIDIR:{profile.name}] stopped: reached max dynamic steps")

    left_best = left_states[0]
    right_best = right_states[0]

    front_path = remove_duplicate_points(np.asarray(left_best.path, dtype=float))
    rear_path = remove_duplicate_points(np.asarray(right_best.path, dtype=float))
    rear_rev = rear_path[::-1]

    full = np.vstack([front_path, rear_rev[1:]])
    full = remove_duplicate_points(full)
    validation = validate_path(sections, full)
    meeting_point = ((left_best.point + right_best.point) * 0.5).tolist()
    connect_check = segment_check(sections, left_best.point, right_best.point)

    return {
        "name": profile.name,
        "description": profile.description,
        "allow_soft": bool(profile.allow_soft),
        "method": "dynamic_alternating_bidirectional_route",
        "meeting_point": meeting_point,
        "connection_segment": [left_best.point.tolist(), right_best.point.tolist()],
        "front_branch_x": None if left_best.branch_x is None else float(left_best.branch_x),
        "rear_branch_x": None if right_best.branch_x is None else float(right_best.branch_x),
        "path": full.tolist(),
        "front_path": front_path.tolist(),
        "rear_path": rear_path.tolist(),
        "logs": left_best.logs + right_best.logs,
        "cost": float(left_best.cost + right_best.cost),
        "connect_hard_bad": int(connect_check["hard_bad"]),
        "connect_soft_bad": int(connect_check["soft_bad"]),
        "length": float(path_length(full)),
        "point_count": int(len(full)),
        **validation,
    }


# ---------------------------------------------------------------------------
# Joint X/Y section lattice planner
# ---------------------------------------------------------------------------


class DualSectionModel:
    """Conservative collision model formed by X and Y section families."""

    def __init__(self, x_sections: list[dict], y_sections: list[dict]):
        if not x_sections or not y_sections:
            raise ValueError("Joint planner requires both x_sections and y_sections; run main1.py again")
        self.x_sections = sorted(x_sections, key=lambda s: s["value"])
        self.y_sections = sorted(y_sections, key=lambda s: s["value"])
        self.x_values = np.asarray([s["value"] for s in self.x_sections], dtype=float)
        self.y_values = np.asarray([s["value"] for s in self.y_sections], dtype=float)
        self._point_status_cache: dict[tuple[float, float, float], tuple[bool, bool, tuple[str, ...]]] = {}
        self._segment_check_cache: dict[tuple, dict] = {}

    @staticmethod
    def _nearest_index(values: np.ndarray, value: float) -> int:
        return int(np.argmin(np.abs(values - float(value))))

    @staticmethod
    def _planar_point(axis: str, point) -> list[float]:
        if axis == "x":
            return [float(point[1]), float(point[2])]
        return [float(point[0]), float(point[2])]

    def _section_status(self, section: dict, point) -> tuple[bool, bool, str | None]:
        planar = self._planar_point(section["axis"], point)
        hard = not point_inside(section, planar)
        soft, soft_name = point_hits_soft_boundary(section, planar)
        return hard, soft, soft_name

    def point_status(self, point) -> tuple[bool, bool, list[str]]:
        point = np.asarray(point, dtype=float)
        cache_key = tuple(round(float(v), 5) for v in point)
        cached = self._point_status_cache.get(cache_key)
        if cached is not None:
            return cached[0], cached[1], list(cached[2])
        xi = self._nearest_index(self.x_values, point[0])
        yi = self._nearest_index(self.y_values, point[1])
        x_hard, x_soft, x_name = self._section_status(self.x_sections[xi], point)
        y_hard, y_soft, y_name = self._section_status(self.y_sections[yi], point)
        names = []
        if x_hard:
            names.append("x_hard_boundary")
        if y_hard:
            names.append("y_hard_boundary")
        if x_soft:
            names.append(x_name or "x_soft_boundary")
        if y_soft:
            names.append(y_name or "y_soft_boundary")
        result = (bool(x_hard or y_hard), bool(x_soft or y_soft), tuple(names))
        self._point_status_cache[cache_key] = result
        return result[0], result[1], list(result[2])

    @staticmethod
    def _point_at_axis(a, b, axis_index: int, value: float) -> np.ndarray | None:
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        delta = float(b[axis_index] - a[axis_index])
        if abs(delta) < 1e-9:
            return None
        t = (float(value) - float(a[axis_index])) / delta
        if -1e-9 <= t <= 1.0 + 1e-9:
            return a + t * (b - a)
        return None

    def segment_check(self, a, b, sample_step: float = SEGMENT_SAMPLE_STEP) -> dict:
        a = np.asarray(a, dtype=float)
        b = np.asarray(b, dtype=float)
        ka = tuple(round(float(v), 5) for v in a)
        kb = tuple(round(float(v), 5) for v in b)
        cache_key = (tuple(sorted((ka, kb))), round(float(sample_step), 5))
        cached = self._segment_check_cache.get(cache_key)
        if cached is not None:
            return cached
        points = sample_segment(a, b, sample_step)

        xlo, xhi = sorted([float(a[0]), float(b[0])])
        ylo, yhi = sorted([float(a[1]), float(b[1])])
        for value in self.x_values:
            if xlo - 1e-9 <= value <= xhi + 1e-9:
                p = self._point_at_axis(a, b, 0, float(value))
                if p is not None:
                    points.append(p)
        for value in self.y_values:
            if ylo - 1e-9 <= value <= yhi + 1e-9:
                p = self._point_at_axis(a, b, 1, float(value))
                if p is not None:
                    points.append(p)

        unique = {}
        for p in points:
            unique[tuple(round(float(v), 5) for v in p)] = p

        hard_bad = soft_bad = 0
        names = []
        for p in unique.values():
            hard, soft, point_names = self.point_status(p)
            hard_bad += int(hard)
            soft_bad += int(soft)
            names.extend(point_names)
        result = {
            "valid": hard_bad == 0 and soft_bad == 0,
            "engineering_valid": hard_bad == 0,
            "hard_bad": int(hard_bad),
            "soft_bad": int(soft_bad),
            "total": len(unique),
            "names": names[:8],
        }
        self._segment_check_cache[cache_key] = result
        return result


def joint_validate_path(model: DualSectionModel, path: np.ndarray) -> dict:
    hard_bad_segments = soft_bad_segments = hard_bad_samples = soft_bad_samples = total_samples = 0
    for a, b in zip(path[:-1], path[1:]):
        chk = model.segment_check(a, b)
        total_samples += chk["total"]
        hard_bad_segments += int(chk["hard_bad"] > 0)
        soft_bad_segments += int(chk["soft_bad"] > 0)
        hard_bad_samples += chk["hard_bad"]
        soft_bad_samples += chk["soft_bad"]
    return {
        "valid": hard_bad_segments == 0 and soft_bad_segments == 0,
        "engineering_valid": hard_bad_segments == 0,
        "hard_bad_segments": int(hard_bad_segments),
        "soft_bad_segments": int(soft_bad_segments),
        "hard_bad_samples": int(hard_bad_samples),
        "soft_bad_samples": int(soft_bad_samples),
        "total_samples": int(total_samples),
    }


def joint_segment_allowed(model: DualSectionModel, a, b, allow_soft: bool) -> tuple[bool, dict]:
    chk = model.segment_check(a, b)
    allowed = chk["hard_bad"] == 0 and (allow_soft or chk["soft_bad"] == 0)
    return allowed, chk


def endpoint_anchor(model: DualSectionModel, endpoint, travel_direction, sign: float, allow_soft: bool) -> np.ndarray:
    length = float(getattr(settings, "JOINT_ENDPOINT_TANGENT_LENGTH", 30.0))
    endpoint = np.asarray(endpoint, dtype=float)
    candidate = endpoint + float(sign) * norm(travel_direction) * length
    ranges_ok = (
        model.x_values[0] <= candidate[0] <= model.x_values[-1]
        and model.y_values[0] <= candidate[1] <= model.y_values[-1]
    )
    if not ranges_ok:
        raise RuntimeError(
            f"endpoint tangent anchor is outside the planning range: {candidate.tolist()}"
        )
    allowed, _chk = joint_segment_allowed(model, endpoint, candidate, allow_soft)
    if not allowed:
        raise RuntimeError(
            f"endpoint tangent anchor segment is blocked: {endpoint.tolist()} -> {candidate.tolist()}"
        )
    return candidate


def joint_lattice_axes(space: dict, model: DualSectionModel) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    xs = model.x_values.copy()
    ys = model.y_values.copy()
    zmin, zmax = [float(v) for v in space["ranges"]["z"]]
    dz = max(float(getattr(settings, "JOINT_GRID_DZ", 20.0)), 1.0)
    zs = np.arange(zmin, zmax + dz * 0.25, dz, dtype=float)
    if len(zs) == 0:
        zs = np.asarray([zmin], dtype=float)
    return xs, ys, zs


def nearest_axis_index(values: np.ndarray, value: float) -> int:
    return int(np.argmin(np.abs(values - float(value))))


def nearby_connector_nodes(
    point: np.ndarray,
    axes: tuple[np.ndarray, np.ndarray, np.ndarray],
    model: DualSectionModel,
    allow_soft: bool,
    radius: int = 2,
    keep: int = 24,
    connection_direction: np.ndarray | None = None,
    direction_from_point: bool = True,
    segment_model: DualSectionModel | None = None,
) -> list[tuple[tuple[int, int, int], float]]:
    segment_model = segment_model or model
    centers = [nearest_axis_index(values, point[i]) for i, values in enumerate(axes)]
    candidates = []
    for ix in range(max(0, centers[0] - radius), min(len(axes[0]), centers[0] + radius + 1)):
        for iy in range(max(0, centers[1] - radius), min(len(axes[1]), centers[1] + radius + 1)):
            for iz in range(max(0, centers[2] - radius), min(len(axes[2]), centers[2] + radius + 1)):
                node = (ix, iy, iz)
                p = np.array([axes[0][ix], axes[1][iy], axes[2][iz]], dtype=float)
                hard, soft, _names = model.point_status(p)
                if hard or (soft and not allow_soft):
                    continue
                allowed, chk = joint_segment_allowed(segment_model, point, p, allow_soft)
                if not allowed:
                    continue
                direction_error = 0.0
                if connection_direction is not None:
                    connector_vector = p - point if direction_from_point else point - p
                    direction_error = angle_degrees(connector_vector, connection_direction)
                    max_angle = float(getattr(settings, "JOINT_ENDPOINT_CONNECTOR_MAX_ANGLE_DEG", 35.0))
                    if direction_error > max_angle:
                        continue
                connector_cost = (
                    dist(point, p)
                    + 8.0 * direction_error
                    + (chk["soft_bad"] * 25.0 if allow_soft else 0.0)
                )
                candidates.append((node, connector_cost))
    candidates.sort(key=lambda item: item[1])
    return candidates[:keep]


def reconstruct_nodes(came_from: dict, node: tuple[int, int, int]) -> list[tuple[int, int, int]]:
    result = [node]
    while node in came_from:
        node = came_from[node]
        result.append(node)
    result.reverse()
    return result


def smooth_joint_path(model: DualSectionModel, path: np.ndarray, allow_soft: bool) -> np.ndarray:
    path = remove_duplicate_points(np.asarray(path, dtype=float))
    if len(path) <= 2:
        return path
    # Keep the endpoint tangent anchor and its first lattice connector on each side.
    # The connector is direction-filtered in joint_astar_path; allowing smoothing to
    # remove it can create an immediate sharp turn right after the 30 mm endpoint
    # straight section.
    preserve = 2
    if len(path) > 2 * preserve + 1:
        prefix = path[:preserve]
        middle = smooth_joint_path_unlocked(model, path[preserve - 1:len(path) - preserve + 1], allow_soft)
        suffix = path[len(path) - preserve + 1:]
        return remove_duplicate_points(np.vstack([prefix, middle[1:], suffix]))
    return smooth_joint_path_unlocked(model, path, allow_soft)


def smooth_joint_path_unlocked(model: DualSectionModel, path: np.ndarray, allow_soft: bool) -> np.ndarray:
    """Line-of-sight smoothing for a path range without endpoint locks."""
    path = remove_duplicate_points(np.asarray(path, dtype=float))
    if len(path) <= 2:
        return path
    lookahead = max(2, int(getattr(settings, "JOINT_SMOOTH_LOOKAHEAD", 80)))
    result = [path[0]]
    current = 0
    while current < len(path) - 1:
        furthest = min(len(path) - 1, current + lookahead)
        chosen = current + 1
        for candidate in range(furthest, current, -1):
            allowed, _chk = joint_segment_allowed(model, path[current], path[candidate], allow_soft)
            if allowed:
                chosen = candidate
                break
        result.append(path[chosen])
        current = chosen
    return remove_duplicate_points(np.asarray(result, dtype=float))


def cubic_hermite_points(p0, p1, direction0, direction1, handle0: float, handle1: float, sample_step: float) -> np.ndarray:
    """Sample a cubic Hermite curve with prescribed endpoint directions."""
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    m0 = norm(direction0) * float(handle0)
    m1 = norm(direction1) * float(handle1)
    count = max(8, int(math.ceil(dist(p0, p1) / max(float(sample_step), 1.0))))
    points = []
    for index in range(count + 1):
        t = index / count
        t2 = t * t
        t3 = t2 * t
        h00 = 2.0 * t3 - 3.0 * t2 + 1.0
        h10 = t3 - 2.0 * t2 + t
        h01 = -2.0 * t3 + 3.0 * t2
        h11 = t3 - t2
        points.append(h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1)
    points = np.asarray(points, dtype=float)
    # The analytic derivative is already exact. Project the first/last sampled
    # chords as well, so polyline/CATIA exports are exactly tangent instead of
    # carrying a small finite-sampling angle error.
    if len(points) >= 3:
        # Keep these exact tangent chords short; the following original curve
        # sample then retains the analytic Hermite curvature.
        first_length = 0.2 * dist(points[0], points[1])
        last_length = 0.2 * dist(points[-2], points[-1])
        points[1] = p0 + norm(direction0) * first_length
        points[-2] = p1 - norm(direction1) * last_length
    return points


def circumradius(a, b, c) -> float:
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    c = np.asarray(c, dtype=float)
    ab = dist(a, b)
    bc = dist(b, c)
    ca = dist(c, a)
    twice_area = float(np.linalg.norm(np.cross(b - a, c - a)))
    if twice_area < 1e-9:
        return float("inf")
    return float(ab * bc * ca / (2.0 * twice_area))


def minimum_path_bend_radius(path: np.ndarray) -> float:
    path = remove_duplicate_points(np.asarray(path, dtype=float))
    radii = [circumradius(a, b, c) for a, b, c in zip(path[:-2], path[1:-1], path[2:])]
    finite = [radius for radius in radii if math.isfinite(radius)]
    return min(finite) if finite else float("inf")


def maximum_path_turn(path: np.ndarray) -> float:
    path = remove_duplicate_points(np.asarray(path, dtype=float))
    turns = [angle_degrees(b - a, c - b) for a, b, c in zip(path[:-2], path[1:-1], path[2:])]
    return max(turns) if turns else 0.0


def curve_allowed(model: DualSectionModel, curve: np.ndarray, allow_soft: bool) -> bool:
    for a, b in zip(curve[:-1], curve[1:]):
        allowed, _chk = joint_segment_allowed(model, a, b, allow_soft)
        if not allowed:
            return False
    return True


def choose_start_tangent_transition(
    model: DualSectionModel,
    path: np.ndarray,
    start_direction: np.ndarray,
    allow_soft: bool,
) -> tuple[np.ndarray, dict]:
    sample_step = float(getattr(settings, "JOINT_ENDPOINT_CURVE_SAMPLE_STEP", 6.0))
    lookahead = max(2, int(getattr(settings, "JOINT_ENDPOINT_TRANSITION_LOOKAHEAD", 5)))
    target_radius = float(getattr(settings, "JOINT_ENDPOINT_MIN_BEND_RADIUS", 100.0))
    max_error = float(getattr(settings, "JOINT_ENDPOINT_MAX_TANGENT_ERROR_DEG", 3.0))
    factors = (0.50, 0.75, 1.00, 1.50, 2.00, 3.00, 4.00)
    candidates = []
    relaxed_candidates = []
    max_join = min(len(path) - 2, lookahead)
    for join_index in range(2, max_join + 1):
        join_direction = path[join_index + 1] - path[join_index]
        chord = dist(path[0], path[join_index])
        if chord < 1e-6 or np.linalg.norm(join_direction) < 1e-9:
            continue
        for factor0 in factors:
            for factor1 in factors:
                curve = cubic_hermite_points(
                    path[0], path[join_index], start_direction, join_direction,
                    chord * factor0, chord * factor1, sample_step,
                )
                local = np.vstack([curve, path[join_index + 1]])
                if not curve_allowed(model, local, allow_soft):
                    continue
                tangent_error = angle_degrees(curve[1] - curve[0], start_direction)
                join_error = angle_degrees(curve[-1] - curve[-2], join_direction)
                radius = minimum_path_bend_radius(local)
                relaxed_candidates.append((radius, tangent_error, join_error, join_index))
                radius_tolerance = max(0.5, 0.02 * target_radius)
                if radius + radius_tolerance < target_radius:
                    continue
                if tangent_error > max_error or join_error > max_error:
                    continue
                radius_shortfall = max(0.0, target_radius - radius)
                error_excess = max(0.0, tangent_error - max_error) + max(0.0, join_error - max_error)
                score = 5000.0 * radius_shortfall + 2000.0 * error_excess + path_length(curve)
                candidates.append((score, curve, join_index, radius, tangent_error, join_error))
    if not candidates:
        if relaxed_candidates:
            best_radius = max(relaxed_candidates, key=lambda item: item[0])
            best_tangent = min(relaxed_candidates, key=lambda item: item[1] + item[2])
            print(
                "[TANGENT] no strict transition; "
                f"best_radius={best_radius[0]:.1f}mm "
                f"(errors={best_radius[1]:.2f}/{best_radius[2]:.2f}deg, join={best_radius[3]}), "
                f"best_errors={best_tangent[1]:.2f}/{best_tangent[2]:.2f}deg "
                f"(radius={best_tangent[0]:.1f}mm, join={best_tangent[3]})"
            )
        raise RuntimeError("no collision-free start tangent transition was found")
    candidates.sort(key=lambda item: item[0])
    _score, curve, join_index, radius, tangent_error, join_error = candidates[0]
    result = remove_duplicate_points(np.vstack([curve, path[join_index + 1:]]))
    return result, {
        "join_index": int(join_index),
        "curve_point_count": int(len(curve)),
        "min_bend_radius": float(radius),
        "tangent_error_deg": float(tangent_error),
        "join_error_deg": float(join_error),
    }


def choose_goal_tangent_transition(
    model: DualSectionModel,
    path: np.ndarray,
    goal_direction: np.ndarray,
    allow_soft: bool,
) -> tuple[np.ndarray, dict]:
    # Reverse the route: arriving at GOAL along goal_direction becomes leaving
    # GOAL along -goal_direction. Reuse the exact same transition search.
    reversed_path = path[::-1].copy()
    transitioned, info = choose_start_tangent_transition(
        model, reversed_path, -norm(goal_direction), allow_soft
    )
    return transitioned[::-1].copy(), info


def enforce_endpoint_tangency(
    model: DualSectionModel,
    path: np.ndarray,
    start_direction: np.ndarray,
    goal_direction: np.ndarray,
    allow_soft: bool,
) -> tuple[np.ndarray, dict]:
    try:
        with_start, start_info = choose_start_tangent_transition(
            model, path, start_direction, allow_soft
        )
        with_both, goal_info = choose_goal_tangent_transition(
            model, with_start, goal_direction, allow_soft
        )
    except RuntimeError:
        if bool(getattr(settings, "JOINT_REQUIRE_SMOOTH_ENDPOINTS", True)):
            raise
        return path, {"required": False, "applied": False}

    start_error = angle_degrees(with_both[1] - with_both[0], start_direction)
    goal_error = angle_degrees(with_both[-1] - with_both[-2], goal_direction)
    validation = joint_validate_path(model, with_both)
    if validation["hard_bad_segments"] or (validation["soft_bad_segments"] and not allow_soft):
        raise RuntimeError("endpoint tangent transition failed final collision validation")
    return with_both, {
        "required": True,
        "applied": True,
        "start": start_info,
        "goal": goal_info,
        "start_tangent_error_deg": float(start_error),
        "goal_tangent_error_deg": float(goal_error),
        "minimum_bend_radius": float(minimum_path_bend_radius(with_both)),
        "maximum_sample_turn_deg": float(maximum_path_turn(with_both)),
    }


def endpoint_anchor_tangency_info(path: np.ndarray, start: np.ndarray, goal: np.ndarray, start_anchor: np.ndarray, goal_anchor: np.ndarray, start_direction: np.ndarray, goal_direction: np.ndarray) -> dict:
    path = remove_duplicate_points(np.asarray(path, dtype=float))
    start_anchor_length = dist(start, start_anchor)
    goal_anchor_length = dist(goal_anchor, goal)
    start_applied = len(path) >= 2 and start_anchor_length > 1e-6 and dist(path[1], start_anchor) < 1e-4
    goal_applied = len(path) >= 2 and goal_anchor_length > 1e-6 and dist(path[-2], goal_anchor) < 1e-4
    start_error = angle_degrees(path[1] - path[0], start_direction) if len(path) >= 2 else float("nan")
    goal_error = angle_degrees(path[-1] - path[-2], goal_direction) if len(path) >= 2 else float("nan")
    start_count = 3 if start_applied and len(path) >= 3 else (2 if start_applied else 0)
    goal_count = 3 if goal_applied and len(path) >= 3 else (2 if goal_applied else 0)
    return {
        "required": True,
        "applied": bool(start_applied and goal_applied),
        "mode": "straight_endpoint_anchor",
        "start": {
            "curve_point_count": int(start_count),
            "anchor": start_anchor.tolist(),
            "length": float(start_anchor_length),
            "tangent_error_deg": float(start_error),
        },
        "goal": {
            "curve_point_count": int(goal_count),
            "anchor": goal_anchor.tolist(),
            "length": float(goal_anchor_length),
            "tangent_error_deg": float(goal_error),
        },
        "start_tangent_error_deg": float(start_error),
        "goal_tangent_error_deg": float(goal_error),
    }


def joint_astar_path(
    profile: RouteProfile,
    space: dict,
    model: DualSectionModel,
    start: np.ndarray,
    goal: np.ndarray,
    start_dir: np.ndarray,
    goal_dir: np.ndarray,
    validation_model: DualSectionModel | None = None,
) -> tuple[np.ndarray, dict]:
    validation_model = validation_model or model
    axes = joint_lattice_axes(space, model)
    start_anchor = endpoint_anchor(validation_model, start, start_dir, +1.0, profile.allow_soft)
    goal_anchor = endpoint_anchor(validation_model, goal, goal_dir, -1.0, profile.allow_soft)

    starts = nearby_connector_nodes(
        start_anchor, axes, model, profile.allow_soft,
        radius=3,
        connection_direction=start_dir,
        direction_from_point=True,
        segment_model=validation_model,
    )
    goals = nearby_connector_nodes(
        goal_anchor, axes, model, profile.allow_soft,
        radius=3,
        connection_direction=goal_dir,
        direction_from_point=False,
        segment_model=validation_model,
    )
    if not starts or not goals:
        raise RuntimeError(
            f"{profile.name}: endpoint cannot connect to joint lattice "
            f"(start_candidates={len(starts)}, goal_candidates={len(goals)})"
        )
    goal_cost = {node: cost for node, cost in goals}

    max_nodes = int(getattr(settings, "JOINT_MAX_EXPANDED_NODES", 250000))
    heuristic_weight = float(getattr(settings, "JOINT_HEURISTIC_WEIGHT", 1.08))
    counter = itertools.count()
    heap = []
    g_score = {}
    came_from = {}
    point_cache = {}
    edge_cache = {}

    def node_point(node):
        return np.array([axes[0][node[0]], axes[1][node[1]], axes[2][node[2]]], dtype=float)

    def heuristic(node):
        return dist(node_point(node), goal_anchor)

    for node, connector_cost in starts:
        if connector_cost < g_score.get(node, float("inf")):
            g_score[node] = connector_cost
            heapq.heappush(heap, (connector_cost + heuristic_weight * heuristic(node), next(counter), node))

    neighbor_offsets = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    expanded = 0
    reached = None
    reached_total = float("inf")

    while heap and expanded < max_nodes:
        f_value, _order, current = heapq.heappop(heap)
        current_g = g_score.get(current, float("inf"))
        if f_value > current_g + heuristic_weight * heuristic(current) + 1e-8:
            continue
        expanded += 1

        if current in goal_cost:
            reached = current
            reached_total = current_g + goal_cost[current]
            break

        if expanded % 5000 == 0:
            print(
                f"[JOINT:{profile.name}] expanded={expanded} open={len(heap)} "
                f"distance_to_goal={heuristic(current):.1f}"
            )

        cp = node_point(current)
        for dx, dy, dz_index in neighbor_offsets:
            neighbor = (current[0] + dx, current[1] + dy, current[2] + dz_index)
            if not (0 <= neighbor[0] < len(axes[0]) and 0 <= neighbor[1] < len(axes[1]) and 0 <= neighbor[2] < len(axes[2])):
                continue

            if neighbor not in point_cache:
                point_cache[neighbor] = model.point_status(node_point(neighbor))
            hard, soft, _names = point_cache[neighbor]
            if hard or (soft and not profile.allow_soft):
                continue

            edge_key = tuple(sorted((current, neighbor)))
            if edge_key not in edge_cache:
                npnt = node_point(neighbor)
                edge_cache[edge_key] = model.segment_check(cp, npnt, sample_step=max(SEGMENT_SAMPLE_STEP, 10.0))
            chk = edge_cache[edge_key]
            if chk["hard_bad"] or (chk["soft_bad"] and not profile.allow_soft):
                continue

            npnt = node_point(neighbor)
            step = dist(cp, npnt)
            turn_cost = 0.0
            previous = came_from.get(current)
            if previous is not None and profile.joint_turn_w > 0.0:
                pp = node_point(previous)
                v0 = cp - pp
                v1 = npnt - cp
                n0 = float(np.linalg.norm(v0))
                n1 = float(np.linalg.norm(v1))
                if n0 > 1e-9 and n1 > 1e-9:
                    c = max(-1.0, min(1.0, float(np.dot(v0, v1) / (n0 * n1))))
                    turn_cost = profile.joint_turn_w * math.degrees(math.acos(c)) / 90.0
            diagonal_xy_cost = 0.0
            if dx != 0 and dy != 0 and profile.joint_diagonal_xy_w > 0.0:
                diagonal_xy_cost = profile.joint_diagonal_xy_w * math.hypot(
                    float(npnt[0] - cp[0]),
                    float(npnt[1] - cp[1]),
                ) / 20.0
            horizontal = max(math.hypot(float(goal_anchor[0] - start_anchor[0]), float(goal_anchor[1] - start_anchor[1])), 1.0)
            progress = max(0.0, min(1.0, math.hypot(float(npnt[0] - start_anchor[0]), float(npnt[1] - start_anchor[1])) / horizontal))
            desired_z = (1.0 - progress) * float(start_anchor[2]) + progress * float(goal_anchor[2])
            desired_z += profile.z_bias * math.sin(math.pi * progress)
            desired_y = (1.0 - progress) * float(start_anchor[1]) + progress * float(goal_anchor[1])
            desired_y += profile.y_bias * math.sin(math.pi * progress)

            preference = 0.012 * profile.goal_pull_w * abs(float(npnt[2]) - desired_z)
            preference += 0.004 * profile.goal_pull_w * abs(float(npnt[1]) - desired_y)
            upward = max(0.0, float(npnt[2] - cp[2]))
            upward_cost = 0.025 * Z_UP_PENALTY * upward
            soft_cost = (0.02 * profile.soft_w * chk["soft_bad"]) if profile.allow_soft else 0.0
            tentative = (
                current_g
                + profile.joint_length_w * step
                + preference
                + upward_cost
                + soft_cost
                + turn_cost
                + diagonal_xy_cost
            )
            if tentative + 1e-8 < g_score.get(neighbor, float("inf")):
                g_score[neighbor] = tentative
                came_from[neighbor] = current
                priority = tentative + heuristic_weight * heuristic(neighbor)
                heapq.heappush(heap, (priority, next(counter), neighbor))

    if reached is None:
        raise RuntimeError(f"{profile.name}: joint A* found no route after expanding {expanded} nodes")

    nodes = reconstruct_nodes(came_from, reached)
    lattice_path = np.asarray([node_point(node) for node in nodes], dtype=float)
    interior = [start_anchor]
    interior.extend(lattice_path)
    interior.append(goal_anchor)
    raw_interior = remove_duplicate_points(np.asarray(interior, dtype=float))
    smooth_interior = smooth_joint_path(model, raw_interior, profile.allow_soft)
    # Keep endpoint anchors outside line-of-sight smoothing so the requested
    # start/end tangent segments cannot be skipped.
    raw = remove_duplicate_points(np.vstack([start, raw_interior, goal]))
    smooth = remove_duplicate_points(np.vstack([start, smooth_interior, goal]))
    validation = joint_validate_path(validation_model, smooth)
    if validation["hard_bad_segments"] or (validation["soft_bad_segments"] and not profile.allow_soft):
        raise RuntimeError("straight endpoint anchor path failed final collision validation")
    tangent_path = smooth
    tangent_info = endpoint_anchor_tangency_info(
        tangent_path, start, goal, start_anchor, goal_anchor, start_dir, goal_dir
    )
    return tangent_path, {
        "expanded_nodes": int(expanded),
        "raw_point_count": int(len(raw)),
        "line_of_sight_point_count": int(len(smooth)),
        "smoothed_point_count": int(len(tangent_path)),
        "lattice_cost": float(reached_total),
        "start_anchor": start_anchor.tolist(),
        "goal_anchor": goal_anchor.tolist(),
        "endpoint_tangency": tangent_info,
    }


def build_joint_route(profile, space, model, start, goal, start_dir, goal_dir, validation_model=None):
    print(f"\n--- Planning {profile.name} with joint X/Y sections ---")
    validation_model = validation_model or model
    path, search_info = joint_astar_path(
        profile, space, model, start, goal, start_dir, goal_dir,
        validation_model=validation_model,
    )
    validation = joint_validate_path(validation_model, path)
    hanger_report = hanger_distance_report(
        path,
        settings_hanger_points(settings),
        float(getattr(settings, "HANGER_POINT_DISTANCE_RANGE", 100.0)),
    )
    half = max(1, len(path) // 2)
    front_path = path[: half + 1]
    rear_path = path[half:]
    logs = []
    previous_axis = None
    axis_switches = 0
    for index, (a, b) in enumerate(zip(path[:-1], path[1:])):
        delta = np.abs(b - a)
        axis = "y" if delta[1] > delta[0] else "x"
        if previous_axis is not None and axis != previous_axis:
            axis_switches += 1
        previous_axis = axis
        chk = validation_model.segment_check(a, b)
        logs.append({
            "index": index,
            "planner_axis": axis,
            "point": b.tolist(),
            "hard_bad": chk["hard_bad"],
            "soft_bad": chk["soft_bad"],
        })

    meeting_point = path[half].tolist()
    return {
        "name": profile.name,
        "description": profile.description,
        "strategy_weights": {
            "joint_length_w": float(profile.joint_length_w),
            "joint_turn_w": float(profile.joint_turn_w),
            "joint_diagonal_xy_w": float(profile.joint_diagonal_xy_w),
            "goal_pull_w": float(profile.goal_pull_w),
            "allow_soft": bool(profile.allow_soft),
        },
        "method": "joint_xy_section_lattice_astar",
        "active_pipe_spec": str(getattr(settings, "ACTIVE_PIPE_SPEC", "")),
        "pipe_spec_ids": sorted(getattr(settings, "selected_pipe_spec_ids", lambda: set())()),
        "route_max_envelope_radius": float(getattr(settings, "ROUTE_MAX_ENVELOPE_RADIUS", settings.PIPE_RADIUS)),
        "pipe_segments": list(getattr(settings, "PIPE_SEGMENTS", [])),
        "meeting_point": meeting_point,
        "connection_segment": [meeting_point, meeting_point],
        "front_branch_x": None,
        "rear_branch_x": None,
        "axis_switches": int(axis_switches),
        "path": path.tolist(),
        "front_path": front_path.tolist(),
        "rear_path": rear_path.tolist(),
        "logs": logs,
        "cost": float(search_info["lattice_cost"]),
        "connect_hard_bad": 0,
        "connect_soft_bad": 0,
        "length": float(path_length(path)),
        "point_count": int(len(path)),
        "hanger_distance": hanger_report,
        "hanger_valid": bool(hanger_report["all_within_range"]),
        "search": search_info,
        **validation,
    }


def distance_to_lines(yz, comp: dict) -> float:
    best = 1e18
    for line in comp.get("lines", []):
        for a, b in zip(line[:-1], line[1:]):
            best = min(best, point_segment_distance_2d(yz, a, b))
    return best


def sampled_path(path: list[list[float]], step: float) -> tuple[list[list[float]], list[float]]:
    samples = [path[0]]
    along = [0.0]
    total = 0.0
    for a, b in zip(path[:-1], path[1:]):
        seg_samples = sample_segment(a, b, step)
        length = dist(a, b)
        count = max(1, len(seg_samples) - 1)
        for idx, p in enumerate(seg_samples[1:], start=1):
            samples.append(p.tolist())
            along.append(total + length * idx / count)
        total += length
    return samples, along


def export_distance_png(space: dict, path: list[list[float]], output: Path) -> None:
    print("[DISTANCE] exporting bidirectional distance png...")
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    samples, along = sampled_path(path, float(getattr(settings, "DISTANCE_SAMPLE_STEP", 20.0)))
    sections = space["sections"]
    xs = np.array([s["x"] for s in sections], dtype=float)

    names = sorted({
        comp.get("name", "part")
        for sec in sections
        for comp in sec.get("all_obstacle_section_lines", sec.get("obstacle_section_lines", []))
    })
    profiles = {name: [] for name in names}

    for p in samples:
        si = int(np.argmin(np.abs(xs - float(p[0]))))
        comps = {
            c.get("name", "part"): c
            for c in sections[si].get("all_obstacle_section_lines", sections[si].get("obstacle_section_lines", []))
        }
        yz = [float(p[1]), float(p[2])]
        for name in names:
            comp = comps.get(name)
            if comp is None:
                profiles[name].append(float("nan"))
            else:
                profiles[name].append(max(distance_to_lines(yz, comp) - settings.PIPE_RADIUS, 0.0))

    fig, ax = plt.subplots(figsize=(12, 4.8))
    for name, values in profiles.items():
        arr = np.asarray(values, dtype=float)
        if np.any(np.isfinite(arr)):
            ax.plot(along, np.minimum(arr, settings.DISTANCE_Y_MAX), linewidth=1.4, label=name)

    ax.axhline(settings.DEFAULT_OBSTACLE_CLEARANCE, color="black", linestyle="--", linewidth=1.1)
    ax.set_xlim(0.0, max(along) if along else 1.0)
    ax.set_ylim(0.0, settings.DISTANCE_Y_MAX)
    ax.set_xlabel("Distance along bidirectional pipe route (mm)")
    ax.set_ylabel("Outer wall distance to STEP (mm)")
    ax.grid(True, color="0.88")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    plt.close(fig)
    print(f"[DISTANCE] saved: {output}")


def export_html(routes: list[dict], output: Path) -> None:
    print("[HTML] loading STEP meshes...")
    x_range = html_x_range(settings)
    meshes = load_step_meshes_for_html(settings, x_range)
    hanger_points = settings_hanger_points(settings).tolist()
    route_points = [
        p
        for r in routes
        for key in ("path", "front_path", "rear_path")
        for p in r.get(key, [])
    ]
    route_points.extend(hanger_points)
    payload = {
        "meshes": meshes,
        "hanger_points": hanger_points,
        "hanger_point_distance_range": float(getattr(settings, "HANGER_POINT_DISTANCE_RANGE", 100.0)),
        "routes": [
            {
                "name": r["name"],
                "description": r["description"],
                "path": r["path"],
                "front_path": r["front_path"],
                "rear_path": r["rear_path"],
                "meeting_point": r["meeting_point"],
                "front_branch_x": r["front_branch_x"],
                "rear_branch_x": r["rear_branch_x"],
                "length": r["length"],
                "valid": r["valid"],
                "engineering_valid": r["engineering_valid"],
                "hard_bad_segments": r["hard_bad_segments"],
                "soft_bad_segments": r["soft_bad_segments"],
                "hanger_distance": r.get("hanger_distance", {}),
                "hanger_valid": r.get("hanger_valid", True),
            }
            for r in routes
        ],
        "x_range": list(x_range),
        "aspect_ratio": html_aspect_ratio(x_range, meshes, route_points),
    }

    data_json = json.dumps(payload, ensure_ascii=False)
    colors = ["#ef2929", "#111111", "#2563eb", "#16a34a", "#f97316"]
    colors_json = json.dumps(colors)

    html = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Joint X/Y Section Route Preview</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    html, body { margin: 0; height: 100%; font-family: Arial, sans-serif; }
    #plot { width: 100vw; height: 100vh; }
    #panel {
      position: fixed; left: 12px; top: 12px; z-index: 10;
      background: rgba(255,255,255,.88); border: 1px solid #d0d0d0;
      padding: 8px 10px; font-size: 13px; line-height: 1.45;
      max-width: 420px;
    }
  </style>
</head>
<body>
  <div id="panel">
    <strong>Joint X/Y Section Route</strong><br>
    Drag: rotate / Shift+drag: pan / Wheel: zoom<br>
    Red route is the default route.
  </div>
  <div id="plot"></div>
  <script>
const DATA = __DATA__;
const COLORS = __COLORS__;
const traces = [];

for (const mesh of DATA.meshes) {
  traces.push({
    type: "mesh3d",
    name: mesh.name,
    x: mesh.x, y: mesh.y, z: mesh.z,
    i: mesh.i, j: mesh.j, k: mesh.k,
    color: mesh.boundary === "soft" ? "#60a5fa" : mesh.role === "ground" ? "#9ca3af" : "#9b9b9b",
    opacity: mesh.boundary === "soft" ? 0.16 : 0.24,
    flatshading: true,
    hoverinfo: "name",
  });
}

for (let idx = 0; idx < DATA.routes.length; idx++) {
  const route = DATA.routes[idx];
  const path = route.path || [];
  const color = COLORS[idx % COLORS.length];
  const hanger = route.hanger_distance || {};
  const hangerText = hanger.count ? " | Hmax=" + hanger.max_distance.toFixed(1) : "";
  traces.push({
    type: "scatter3d",
    mode: "lines+markers",
    name: route.name + " | L=" + route.length.toFixed(1) + hangerText,
    x: path.map(p => p[0]),
    y: path.map(p => p[1]),
    z: path.map(p => p[2]),
    line: { color, width: idx === 0 ? 8 : 5 },
    marker: { color, size: idx === 0 ? 3 : 2 },
    hovertemplate:
      route.name + "<br>x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f}<extra></extra>",
  });

  if (route.front_path && route.front_path.length) {
    traces.push({
      type: "scatter3d",
      mode: "lines",
      name: route.name + " front",
      x: route.front_path.map(p => p[0]),
      y: route.front_path.map(p => p[1]),
      z: route.front_path.map(p => p[2]),
      line: { color, width: 2, dash: "dash" },
      showlegend: false,
      hoverinfo: "skip",
    });
  }
  if (route.rear_path && route.rear_path.length) {
    traces.push({
      type: "scatter3d",
      mode: "lines",
      name: route.name + " rear",
      x: route.rear_path.map(p => p[0]),
      y: route.rear_path.map(p => p[1]),
      z: route.rear_path.map(p => p[2]),
      line: { color, width: 2, dash: "dot" },
      showlegend: false,
      hoverinfo: "skip",
    });
  }
  if (route.meeting_point) {
    traces.push({
      type: "scatter3d",
      mode: "markers",
      name: route.name + " meeting",
      x: [route.meeting_point[0]],
      y: [route.meeting_point[1]],
      z: [route.meeting_point[2]],
      marker: { color, size: 5, symbol: "diamond" },
      showlegend: false,
      hoverinfo: "name",
    });
  }
}

if (DATA.hanger_points && DATA.hanger_points.length) {
  traces.push({
    type: "scatter3d",
    mode: "markers",
    name: "hanger points <= " + DATA.hanger_point_distance_range.toFixed(1) + "mm",
    x: DATA.hanger_points.map(p => p[0]),
    y: DATA.hanger_points.map(p => p[1]),
    z: DATA.hanger_points.map(p => p[2]),
    marker: { color: "#9333ea", size: 6, symbol: "circle" },
    hovertemplate: "hanger<br>x=%{x:.1f}<br>y=%{y:.1f}<br>z=%{z:.1f}<extra></extra>",
  });
}

Plotly.newPlot("plot", traces, {
  margin: { l: 0, r: 0, t: 0, b: 0 },
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
  </script>
</body>
</html>
""".replace("__DATA__", data_json).replace("__COLORS__", colors_json)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"[HTML] saved: {output}")


def main() -> None:
    t0 = time.time()
    print("=== Main2 Joint X/Y Section Lattice Planner ===")

    space_path = settings.SPACE_JSON if settings.SPACE_JSON.exists() else settings.SPACE_CHECKPOINT_JSON
    print(f"[LOAD] space: {space_path}")
    space = load_json(space_path)
    extra_clearance = float(getattr(settings, "MAIN2_EXTRA_CLEARANCE", 0.0))
    if not 0.0 <= extra_clearance <= 20.0:
        raise ValueError("MAIN2_EXTRA_CLEARANCE must be between 0 and 20 mm")
    x_sections = section_polygons(space, "x", extra_clearance=extra_clearance)
    y_sections = section_polygons(space, "y", extra_clearance=extra_clearance)
    if not y_sections:
        raise RuntimeError(
            "planning_space.json does not contain y_sections. "
            "Run the updated main1.py before main2.py."
        )
    model = DualSectionModel(x_sections, y_sections)
    # Fixed endpoints and their prescribed tangent curves may lie inside the
    # optional extra-margin band. Validate those transitions against the
    # original engineering clearance, while the A* trunk uses the full margin.
    validation_model = DualSectionModel(
        section_polygons(space, "x"), section_polygons(space, "y")
    )

    start = np.asarray(space.get("start", settings.START), dtype=float)
    goal = np.asarray(space.get("goal", settings.GOAL), dtype=float)
    start_dir = get_start_dir()
    goal_dir = get_goal_dir()

    all_sections = x_sections + y_sections
    soft_total = sum(len(sec["raw"].get("soft_obstacle_section_lines", [])) for sec in all_sections)
    hard_total = sum(len(hard_boundary_lines(sec)) for sec in all_sections)

    print(f"[LOAD] x_sections={len(x_sections)}, y_sections={len(y_sections)}")
    print(f"[LOAD] main2_extra_clearance={extra_clearance:.1f} mm")
    print(f"[LOAD] start={start.tolist()}")
    print(f"[LOAD] goal={goal.tolist()}")
    print(f"[LOAD] start_dir={start_dir.tolist()}")
    print(f"[LOAD] goal_dir={goal_dir.tolist()}")
    print(f"[LOAD] hard_section_records={hard_total}, soft_section_records={soft_total}")
    print(
        f"[PARAM] dx={getattr(settings, 'SECTION_DX', 20.0)}, "
        f"dy={getattr(settings, 'SECTION_DY', 20.0)}, "
        f"dz={getattr(settings, 'JOINT_GRID_DZ', 20.0)}"
    )

    routes = []
    for profile in make_profiles():
        route = build_joint_route(
            profile, space, model, start, goal, start_dir, goal_dir,
            validation_model=validation_model,
        )
        route["requested_extra_clearance"] = float(extra_clearance)
        routes.append(route)
        tangent = route["search"].get("endpoint_tangency", {})
        hanger = route.get("hanger_distance", {})
        hanger_text = ""
        if hanger.get("count", 0):
            hanger_text = (
                f" | hanger={hanger.get('satisfied_count', 0)}/{hanger.get('count', 0)} "
                f"max={hanger.get('max_distance', float('nan')):.1f}mm"
            )
        print(
            f"[JOINT:{profile.name}] done | length={route['length']:.1f} | cost={route['cost']:.1f} | "
            f"valid={route['valid']} | engineering_valid={route['engineering_valid']} | "
            f"axis_switches={route['axis_switches']} expanded={route['search']['expanded_nodes']} | "
            f"startDirErr={tangent.get('start_tangent_error_deg', float('nan')):.2f}deg "
            f"goalDirErr={tangent.get('goal_tangent_error_deg', float('nan')):.2f}deg | "
            f"hardSeg={route['hard_bad_segments']} softSeg={route['soft_bad_segments']} | "
            f"hardSamples={route['hard_bad_samples']} softSamples={route['soft_bad_samples']}"
            f"{hanger_text}"
        )

    default_route = routes[0]
    result = {
        "method": "joint_xy_section_lattice_astar",
        "space_json": str(space_path),
        "hanger_points": settings_hanger_points(settings).tolist(),
        "hanger_point_distance_range": float(getattr(settings, "HANGER_POINT_DISTANCE_RANGE", 100.0)),
        "path": default_route["path"],
        "logs": default_route["logs"],
        "cost": default_route["cost"],
        "valid_segments": 0 if default_route["hard_bad_segments"] or default_route["soft_bad_segments"] else max(0, len(default_route["path"]) - 1),
        "total_segments": max(0, len(default_route["path"]) - 1),
        "routes": routes,
    }

    print("[SAVE] bidirectional route result json...")
    save_json(result, BIDIR_RESULT_JSON)

    export_html(routes, BIDIR_HTML)

    try:
        export_distance_png(space, default_route["path"], BIDIR_DISTANCE_PNG)
    except Exception as exc:
        print(f"[WARN] distance png failed: {type(exc).__name__}: {exc}")

    elapsed = time.time() - t0
    print("\n=== Done ===")
    print(f"[DONE] BIDIR_RESULT_JSON: {BIDIR_RESULT_JSON}")
    print(f"[DONE] BIDIR_HTML: {BIDIR_HTML}")
    print(f"[DONE] BIDIR_DISTANCE_PNG: {BIDIR_DISTANCE_PNG}")
    print(f"[DONE] elapsed={elapsed:.1f}s")


if __name__ == "__main__":
    main()
