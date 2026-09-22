# -*- coding: utf-8 -*-
from __future__ import annotations

import json
import math
import time
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

from occ_mesh import component_rule, infer_ground_z, load_components, read_cad_shape, scan_step_files, triangulate_shape_to_mesh
from occ_section import section_mesh_to_xz_lines, section_mesh_to_yz_lines


def save_json(data, path: Path) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def load_json(path: Path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def point_list(point) -> list[float]:
    return [float(v) for v in point]


def normalize_rule_key(value: str) -> str:
    return str(value).lower().strip()


def component_rule_dict(step_path: Path, settings) -> dict:
    key_file = normalize_rule_key(step_path.name)
    key_stem = normalize_rule_key(step_path.stem)
    rules = {
        normalize_rule_key(key): value
        for key, value in getattr(settings, "SPECIAL_COMPONENT_RULES", {}).items()
    }
    return rules.get(key_file) or rules.get(key_stem) or {}


def component_boundary_mode(step_path: Path, role: str, settings) -> str:
    rule = component_rule_dict(step_path, settings)
    if "boundary" in rule:
        boundary = normalize_rule_key(rule["boundary"])
    elif role == "obstacle":
        boundary = "hard"
    else:
        boundary = "none"
    if boundary not in {"hard", "soft", "none"}:
        raise ValueError(f"Invalid boundary mode for {step_path.name}: {boundary}")
    return boundary


def component_soft_penalty(step_path: Path, settings) -> float:
    rule = component_rule_dict(step_path, settings)
    return float(rule.get("soft_penalty", getattr(settings, "DEFAULT_SOFT_PENALTY", 1.0)))


def import_shapely():
    try:
        from shapely.geometry import LineString, MultiPolygon, Polygon, box
        from shapely.ops import polygonize, unary_union, triangulate
        return LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate
    except Exception as exc:
        raise ImportError("缺少 shapely，请在当前 Python 环境安装：python -m pip install shapely") from exc


LineString = MultiPolygon = Polygon = box = polygonize = unary_union = triangulate = None


def ensure_shapely() -> None:
    global LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate
    if Polygon is None:
        LineString, MultiPolygon, Polygon, box, polygonize, unary_union, triangulate = import_shapely()


def get_x_range(settings) -> list[float]:
    if settings.X_RANGE is not None:
        return [float(settings.X_RANGE[0]), float(settings.X_RANGE[1])]
    sx, gx = float(settings.START[0]), float(settings.GOAL[0])
    return [
        min(sx, gx) - float(settings.X_PADDING_BEFORE_START),
        max(sx, gx) + float(settings.X_PADDING_AFTER_GOAL),
    ]


def get_yz_range(settings, ground_z: float) -> tuple[list[float], list[float]]:
    half = float(settings.SECTION_WIDTH_Y) / 2.0
    y_range = [float(settings.SECTION_CENTER_Y) - half, float(settings.SECTION_CENTER_Y) + half]
    z_range = [
        float(ground_z) + float(settings.DEFAULT_GROUND_CLEARANCE) + float(settings.PIPE_RADIUS),
        float(ground_z) + float(settings.SECTION_HEIGHT_ABOVE_GROUND),
    ]
    return y_range, z_range


def frange(vmin: float, vmax: float, step: float) -> list[float]:
    count = int(math.floor((float(vmax) - float(vmin)) / float(step))) + 1
    return [float(vmin) + i * float(step) for i in range(max(count, 1))]


def polygon_to_record(poly) -> dict:
    poly = poly.buffer(0)
    if float(poly.area) <= 0:
        return {}
    exterior = [[float(y), float(z)] for y, z in list(poly.exterior.coords)]
    holes = [
        [[float(y), float(z)] for y, z in list(ring.coords)]
        for ring in poly.interiors
        if len(ring.coords) >= 4
    ]
    return {
        "polygon": exterior,
        "holes": holes,
        "area": float(poly.area),
        "centroid": [float(poly.centroid.x), float(poly.centroid.y)],
    }


def explode_polygons(geom, min_area: float) -> list:
    ensure_shapely()
    if geom.is_empty:
        return []
    if isinstance(geom, Polygon):
        parts = [geom]
    elif isinstance(geom, MultiPolygon):
        parts = list(geom.geoms)
    else:
        parts = [g for g in getattr(geom, "geoms", []) if isinstance(g, Polygon)]
    return [p.buffer(0) for p in parts if not p.is_empty and float(p.area) >= float(min_area)]



def line_to_linestring(line):
    ensure_shapely()
    try:
        if len(line) < 2:
            return None
        geom = LineString([(float(y), float(z)) for y, z in line])
        if geom.is_empty or geom.length <= 1e-6:
            return None
        return geom
    except Exception:
        return None


def closed_line_polygons(lines, close_tol: float, min_area: float):
    ensure_shapely()
    polys = []
    for line in lines:
        if len(line) < 4:
            continue
        dy = float(line[0][0]) - float(line[-1][0])
        dz = float(line[0][1]) - float(line[-1][1])
        if math.hypot(dy, dz) > close_tol:
            continue
        coords = [(float(y), float(z)) for y, z in line]
        if coords[0] != coords[-1]:
            coords.append(coords[0])
        try:
            poly = Polygon(coords).buffer(0)
            if not poly.is_empty and float(poly.area) >= float(min_area):
                polys.append(poly)
        except Exception:
            continue
    return polys


def polygonize_from_lines(lines, min_area: float):
    ensure_shapely()
    line_geoms = []
    for line in lines:
        geom = line_to_linestring(line)
        if geom is not None:
            line_geoms.append(geom)
    if not line_geoms:
        return []
    try:
        merged = unary_union(line_geoms)
        polys = []
        for poly in polygonize(merged):
            poly = poly.buffer(0)
            if not poly.is_empty and float(poly.area) >= float(min_area):
                polys.append(poly)
        return polys
    except Exception:
        return []


def solid_fill_area_for_component(comp: dict, settings_values: dict):
    ensure_shapely()
    lines = comp.get("lines", [])
    if not lines:
        return None, {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "none"}

    close_tol = float(settings_values["SOLID_LOOP_CLOSE_TOLERANCE"])
    min_area = float(settings_values["SOLID_MIN_FILL_AREA"])
    buffer_radius = float(comp["buffer_radius"])

    fill_polys = []
    fill_polys.extend(closed_line_polygons(lines, close_tol, min_area))
    fill_polys.extend(polygonize_from_lines(lines, min_area))
    solid_mode = "closed_loop"

    if settings_values.get("SOLID_FILL_FROM_BUFFER", True):
        line_geoms = [line_to_linestring(line) for line in lines]
        line_geoms = [g for g in line_geoms if g is not None]
        if line_geoms:
            try:
                buffered = unary_union([g.buffer(buffer_radius, cap_style=1, join_style=1) for g in line_geoms]).buffer(0)
                for poly in explode_polygons(buffered, min_area):
                    fill_polys.append(poly)
                solid_mode = "closed_or_buffer_sealed"
            except Exception:
                pass

    if not fill_polys:
        return None, {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "none"}

    area = unary_union(fill_polys).buffer(0)
    return area, {
        "solid_fill_area": float(area.area) if area and not area.is_empty else 0.0,
        "solid_loop_count": len(fill_polys),
        "solid_mode": solid_mode,
    }


def obstacle_lines_to_area(obstacle_section_lines: list[dict], settings_values: dict) -> tuple[object | None, list[dict]]:
    ensure_shapely()
    blockers = []
    debug = []

    for comp in obstacle_section_lines:
        radius = float(comp["buffer_radius"])
        line_geoms = []
        for line in comp.get("lines", []):
            geom = line_to_linestring(line)
            if geom is not None:
                line_geoms.append(geom)

        if not line_geoms:
            continue

        buffered_lines = None
        try:
            buffered_lines = unary_union([g.buffer(radius, cap_style=1, join_style=1) for g in line_geoms]).buffer(0)
            if not buffered_lines.is_empty:
                blockers.append(buffered_lines)
        except Exception:
            pass

        solid_info = {"solid_fill_area": 0.0, "solid_loop_count": 0, "solid_mode": "not_solid"}
        if comp.get("solid", True):
            solid_area, solid_info = solid_fill_area_for_component(comp, settings_values)
            if solid_area is not None and not solid_area.is_empty:
                blockers.append(solid_area)

        debug.append({
            "name": comp.get("name", ""),
            "solid": bool(comp.get("solid", True)),
            "line_count": len(line_geoms),
            "buffer_radius": radius,
            "buffer_area": float(buffered_lines.area) if buffered_lines is not None and not buffered_lines.is_empty else 0.0,
            **solid_info,
        })

    if not blockers:
        return None, debug
    return unary_union(blockers).buffer(0), debug



def compute_axis_section(
    axis: str,
    value: float,
    obstacle_components: list[dict],
    lateral_range: list[float],
    z_range: list[float],
    settings_values: dict,
) -> dict:
    """Build a free-space section normal to X or Y.

    X sections store polygons in Y/Z coordinates.  Y sections store polygons
    in X/Z coordinates.  Both therefore share all buffering/polygon logic.
    """
    ensure_shapely()
    axis = str(axis).lower()
    if axis not in {"x", "y"}:
        raise ValueError(f"Unsupported section axis: {axis}")
    hard_obstacle_section_lines = []
    soft_obstacle_section_lines = []

    for comp in obstacle_components:
        try:
            mesh = comp.get("mesh")
            if mesh is None:
                print(f"[WARN] section skip {axis}={float(value):.1f}, {comp['name']}: mesh is None")
                continue

            if axis == "x":
                lines = section_mesh_to_yz_lines(mesh, float(value))
            else:
                lines = section_mesh_to_xz_lines(mesh, float(value))
            if not lines:
                continue

            record = {
                "name": comp["name"],
                "file": comp["file"],
                "clearance": float(comp["clearance"]),
                "buffer_radius": float(settings_values["PIPE_RADIUS"]) + float(comp["clearance"]),
                "boundary": comp.get("boundary", "hard"),
                "solid": bool(comp.get("solid", True)),
                "soft_penalty": float(comp.get("soft_penalty", 1.0)),
                "lines": lines,
            }

            if record["boundary"] == "soft":
                soft_obstacle_section_lines.append(record)
            elif record["boundary"] == "hard":
                hard_obstacle_section_lines.append(record)

        except Exception as exc:
            print(f"[WARN] section failed {axis}={float(value):.1f}, {comp['name']}: {type(exc).__name__}: {exc}")

    rect = box(float(lateral_range[0]), float(z_range[0]), float(lateral_range[1]), float(z_range[1]))

    # 核心：只扣除 hard 边界；soft 边界不裁剪 free_regions，只记录截面线。
    hard_obstacle_area, solid_debug = obstacle_lines_to_area(hard_obstacle_section_lines, settings_values)
    free_geom = rect if hard_obstacle_area is None else rect.difference(hard_obstacle_area).buffer(0)

    free_regions = []
    for poly in explode_polygons(free_geom, settings_values["MIN_FREE_REGION_AREA"]):
        if settings_values["POLYGON_SIMPLIFY_TOLERANCE"] > 0:
            poly = poly.simplify(settings_values["POLYGON_SIMPLIFY_TOLERANCE"], preserve_topology=True)
        rec = polygon_to_record(poly)
        if rec:
            free_regions.append(rec)

    result = {
        "axis": axis,
        "value": float(value),
        "free_regions": free_regions,

        # 兼容旧接口：obstacle_section_lines 仍代表硬障碍线
        "obstacle_section_lines": hard_obstacle_section_lines,

        # 新接口：软硬边界分开存储
        "hard_obstacle_section_lines": hard_obstacle_section_lines,
        "soft_obstacle_section_lines": soft_obstacle_section_lines,
        "all_obstacle_section_lines": hard_obstacle_section_lines + soft_obstacle_section_lines,
        "solid_debug": solid_debug,
    }
    result[axis] = float(value)
    return result


def compute_section(x: float, obstacle_components: list[dict], y_range: list[float], z_range: list[float], settings_values: dict) -> dict:
    """Backward-compatible X-section worker."""
    return compute_axis_section("x", x, obstacle_components, y_range, z_range, settings_values)




def serialize_space(
    sections: list[dict],
    components: list,
    ignored: list,
    settings,
    ranges: dict,
    axes: dict,
    ground_z: float,
    is_checkpoint: bool,
    y_sections: list[dict] | None = None,
) -> dict:
    y_sections = list(y_sections or [])
    total_area = sum(float(r.get("area", 0.0)) for sec in sections for r in sec.get("free_regions", []))
    design_area = (ranges["y"][1] - ranges["y"][0]) * (ranges["z"][1] - ranges["z"][0])
    total_sections = len(axes["xs"])

    component_records = []
    hard_count = 0
    soft_count = 0

    for c in components:
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        if boundary == "hard":
            hard_count += 1
        elif boundary == "soft":
            soft_count += 1

        component_records.append({
            "name": c.name,
            "file": c.file,
            "role": c.role,
            "boundary": boundary,
            "clearance": float(c.clearance),
            "solid": bool(c.solid),
            "mesh_triangles": int(c.mesh.get("triangle_count", 0)) if c.mesh else 0,
        })

    return {
        "meta": {
            "method": "dual_axis_mesh_section_soft_hard_boundary_space",
            "is_checkpoint": bool(is_checkpoint),
            "completed_sections": len(sections),
            "total_sections": total_sections,
            "completed_x_sections": len(sections),
            "completed_y_sections": len(y_sections),
            "total_x_sections": len(axes.get("xs", [])),
            "total_y_sections": len(axes.get("ys", [])),
            "base_step_dir": str(settings.BASE_STEP_DIR),
            "pipe_radius": float(settings.PIPE_RADIUS),
            "default_obstacle_clearance": float(settings.DEFAULT_OBSTACLE_CLEARANCE),
            "default_ground_clearance": float(settings.DEFAULT_GROUND_CLEARANCE),
            "ground_z": float(ground_z),
            "section_width_y": float(settings.SECTION_WIDTH_Y),
            "section_height_above_ground": float(settings.SECTION_HEIGHT_ABOVE_GROUND),
            "section_dx": float(settings.SECTION_DX),
            "section_dy": float(getattr(settings, "SECTION_DY", settings.SECTION_DX)),
            "hard_obstacle_count": int(hard_count),
            "soft_obstacle_count": int(soft_count),
            "free_area_ratio": float(total_area / max(design_area * max(len(sections), 1), 1.0)),
        },
        "ranges": ranges,
        "axes": axes,
        "components": {
            "all": component_records,
            "ignored": [
                {"name": c.name, "file": c.file, "role": c.role, "boundary": "none", "clearance": float(c.clearance), "solid": bool(c.solid)}
                for c in ignored
            ],
        },
        # ``sections`` remains an alias for X sections so older consumers keep
        # working.  New planners should read both explicit families.
        "sections": sections,
        "x_sections": sections,
        "y_sections": y_sections,
        "start": point_list(settings.START),
        "goal": point_list(settings.GOAL),
    }



def checkpoint(sections, components, ignored, settings, ranges, axes, ground_z, final: bool = False, y_sections=None) -> None:
    if not settings.ENABLE_SPACE_CHECKPOINT and not final:
        return
    data = serialize_space(sections, components, ignored, settings, ranges, axes, ground_z, not final, y_sections=y_sections)
    save_json(data, settings.SPACE_CHECKPOINT_JSON)


def progress(label: str, current: int, total: int) -> None:
    total = max(int(total), 1)
    ratio = min(max(current / total, 0.0), 1.0)
    print(f"\r{label} {ratio * 100:6.2f}% {current}/{total}", end="", flush=True)



def compute_planning_space(settings) -> dict:
    t0 = time.time()
    print("\n=== Main1 Debug: Soft/Hard Planning Space ===")
    print("[LOAD] Start loading CAD components...")
    components, ignored = load_components(settings, with_shape=True, with_mesh=True)
    obstacles = [c for c in components if c.role == "obstacle"]
    grounds = [c for c in components if c.role == "ground"]
    if not obstacles:
        raise RuntimeError("No obstacle CAD components found.")

    ground_values = [infer_ground_z(g.mesh) for g in grounds]
    ground_z = float(sum(ground_values) / len(ground_values)) if ground_values else -300.0
    x_range = get_x_range(settings)
    y_range, z_range = get_yz_range(settings, ground_z)
    xs = frange(x_range[0], x_range[1], settings.SECTION_DX)
    section_dy = float(getattr(settings, "SECTION_DY", settings.SECTION_DX))
    ys = frange(y_range[0], y_range[1], section_dy)

    ranges = {"x": [float(x_range[0]), float(x_range[1])], "y": y_range, "z": z_range}
    axes = {
        "xs": [float(x) for x in (xs.tolist() if hasattr(xs, "tolist") else xs)],
        "ys": [float(y) for y in (ys.tolist() if hasattr(ys, "tolist") else ys)],
    }

    obstacle_components = []
    hard_count = 0
    soft_count = 0

    for c in obstacles:
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        if boundary == "hard":
            hard_count += 1
        elif boundary == "soft":
            soft_count += 1

        obstacle_components.append({
            "name": c.name,
            "file": c.file,
            "clearance": float(c.clearance),
            "boundary": boundary,
            "solid": bool(c.solid),
            "soft_penalty": component_soft_penalty(Path(c.file), settings),
            "mesh": c.mesh,   # 关键：把已经生成好的 mesh 传给子进程
        })

    settings_values = {
        "MESH_LINEAR_DEFLECTION": float(settings.MESH_LINEAR_DEFLECTION),
        "MESH_ANGULAR_DEFLECTION": float(settings.MESH_ANGULAR_DEFLECTION),
        "PIPE_RADIUS": float(settings.PIPE_RADIUS),
        "MIN_FREE_REGION_AREA": float(settings.MIN_FREE_REGION_AREA),
        "POLYGON_SIMPLIFY_TOLERANCE": float(settings.POLYGON_SIMPLIFY_TOLERANCE),
        "SOLID_LOOP_CLOSE_TOLERANCE": float(getattr(settings, "SOLID_LOOP_CLOSE_TOLERANCE", 8.0)),
        "SOLID_MIN_FILL_AREA": float(getattr(settings, "SOLID_MIN_FILL_AREA", settings.MIN_FREE_REGION_AREA)),
        "SOLID_FILL_FROM_BUFFER": bool(getattr(settings, "SOLID_FILL_FROM_BUFFER", True)),
    }

    print(f"[SPACE] components={len(components)}, obstacles={len(obstacles)}, hard={hard_count}, soft={soft_count}, ground_z={ground_z:.3f}")
    print("[COMPONENTS]")
    for c in components:
        mesh_triangles = int(c.mesh.get("triangle_count", 0)) if c.mesh else 0
        boundary = component_boundary_mode(Path(c.file), c.role, settings)
        print(
            f"  - {Path(c.file).name}: role={c.role}, boundary={boundary}, "
            f"clearance={float(c.clearance):.1f}, solid={bool(c.solid)}, "
            f"mesh_triangles={mesh_triangles}"
        )
    if ignored:
        print("[IGNORED]")
        for c in ignored:
            print(
                f"  - {Path(c.file).name}: role={c.role}, "
                f"clearance={float(c.clearance):.1f}, solid={bool(c.solid)}"
            )

    print(
        f"[SPACE] x={ranges['x']} y={ranges['y']} z={ranges['z']} "
        f"x_sections={len(xs)} y_sections={len(ys)}"
    )
    print("[SPACE] hard boundary cuts free space; soft boundary is only recorded.")

    completed_by_x: dict[float, dict] = {}
    completed_by_y: dict[float, dict] = {}
    max_workers = int(settings.SECTION_PARALLEL_WORKERS)
    print(f"[PARALLEL] workers={max_workers}")
    print(f"[PARALLEL] sections={len(xs) + len(ys)} (x={len(xs)}, y={len(ys)})")
    print("[PARALLEL] submitting section jobs...")

    with ProcessPoolExecutor(max_workers=max_workers) as executor:
        futures = {}
        for x in xs:
            future = executor.submit(
                compute_axis_section, "x", float(x), obstacle_components, y_range, z_range, settings_values
            )
            futures[future] = ("x", float(x))
        for y in ys:
            future = executor.submit(
                compute_axis_section, "y", float(y), obstacle_components, x_range, z_range, settings_values
            )
            futures[future] = ("y", float(y))
        print(f"[PARALLEL] submitted {len(futures)} jobs, waiting for first section result...")
        completed = 0
        for fut in as_completed(futures):
            axis, value = futures[fut]
            section_result = fut.result()
            if axis == "x":
                completed_by_x[value] = section_result
            else:
                completed_by_y[value] = section_result
            completed += 1
            ordered_x = [completed_by_x[float(v)] for v in xs if float(v) in completed_by_x]
            ordered_y = [completed_by_y[float(v)] for v in ys if float(v) in completed_by_y]

            sec = section_result
            free_area = sum(float(r.get("area", 0.0)) for r in sec.get("free_regions", []))
            hard_lines = sum(len(c.get("lines", [])) for c in sec.get("hard_obstacle_section_lines", []))
            soft_lines = sum(len(c.get("lines", [])) for c in sec.get("soft_obstacle_section_lines", []))

            # Dual-axis JSON is larger, so checkpoint periodically instead of
            # rewriting it after every completed worker.
            if completed % 5 == 0 or completed == len(futures):
                checkpoint(
                    ordered_x, components, ignored, settings, ranges, axes,
                    ground_z, final=False, y_sections=ordered_y,
                )

            print(
                f"\r[SECTION] {completed:>4}/{len(futures)} "
                f"{completed / len(futures) * 100:6.2f}% | "
                f"{axis}={value:.1f} | free_regions={len(sec.get('free_regions', []))} "
                f"area={free_area:.1f} | hard_lines={hard_lines} soft_lines={soft_lines} | "
                f"checkpoint=ok",
                end="",
                flush=True,
            )

    print()
    sections = [completed_by_x[float(x)] for x in xs]
    y_sections = [completed_by_y[float(y)] for y in ys]
    result = serialize_space(
        sections, components, ignored, settings, ranges, axes, ground_z,
        is_checkpoint=False, y_sections=y_sections,
    )
    print("\n[SAVE] Writing final planning space JSON...")
    save_json(result, settings.SPACE_JSON)

    print("[SAVE] Writing final checkpoint JSON...")
    checkpoint(
        sections, components, ignored, settings, ranges, axes, ground_z,
        final=True, y_sections=y_sections,
    )

    elapsed = time.time() - t0
    print("[DONE] Planning space completed.")
    print(f"[DONE] SPACE_JSON = {settings.SPACE_JSON}")
    print(f"[DONE] SPACE_CHECKPOINT_JSON = {settings.SPACE_CHECKPOINT_JSON}")
    print(f"[DONE] x_sections = {len(sections)}")
    print(f"[DONE] y_sections = {len(y_sections)}")
    print(f"[DONE] elapsed = {elapsed:.1f}s")

    return result



def polygon_display_mesh(region: dict, max_points: int = 240) -> dict | None:
    ensure_shapely()
    poly = Polygon(region["polygon"], region.get("holes", []))
    if not poly.is_valid:
        poly = poly.buffer(0)
    if poly.is_empty:
        return None

    exterior = list(poly.exterior.coords)
    if len(exterior) > max_points:
        step = max(1, len(exterior) // max_points)
        exterior = exterior[::step]
        if exterior[0] != exterior[-1]:
            exterior.append(exterior[0])
        poly = Polygon(exterior, [list(r.coords) for r in poly.interiors])

    vertices: list[list[float]] = []
    index: dict[tuple[float, float], int] = {}
    ii: list[int] = []
    jj: list[int] = []
    kk: list[int] = []

    def add_vertex(y, z):
        key = (round(float(y), 6), round(float(z), 6))
        if key not in index:
            index[key] = len(vertices)
            vertices.append([float(y), float(z)])
        return index[key]

    for tri in triangulate(poly):
        probe = tri.representative_point()
        if not poly.contains(probe) and not poly.touches(probe):
            continue
        coords = list(tri.exterior.coords)[:3]
        if len(coords) != 3:
            continue
        a = add_vertex(*coords[0])
        b = add_vertex(*coords[1])
        c = add_vertex(*coords[2])
        ii.append(a)
        jj.append(b)
        kk.append(c)

    if not vertices or not ii:
        return None

    return {
        "y": [p[0] for p in vertices],
        "z": [p[1] for p in vertices],
        "i": ii,
        "j": jj,
        "k": kk,
        "outline": [[float(y), float(z)] for y, z in list(poly.exterior.coords)],
        "holes": [
            [[float(y), float(z)] for y, z in list(r.coords)]
            for r in poly.interiors
        ],
    }


def html_x_range(settings) -> tuple[float, float]:
    margin = float(getattr(settings, "HTML_X_MARGIN", 1000.0))
    x0 = min(float(settings.START[0]), float(settings.GOAL[0])) - margin
    x1 = max(float(settings.START[0]), float(settings.GOAL[0])) + margin
    return x0, x1


def clip_mesh_to_x_range(mesh: dict, x_range: tuple[float, float] | None) -> dict:
    if x_range is None:
        return mesh

    x_min, x_max = x_range
    xs = mesh["x"]
    ys = mesh["y"]
    zs = mesh["z"]
    out_x = []
    out_y = []
    out_z = []
    out_i = []
    out_j = []
    out_k = []
    index_map: dict[int, int] = {}

    def add_vertex(old_idx: int) -> int:
        if old_idx not in index_map:
            index_map[old_idx] = len(out_x)
            out_x.append(xs[old_idx])
            out_y.append(ys[old_idx])
            out_z.append(zs[old_idx])
        return index_map[old_idx]

    for a, b, c in zip(mesh["i"], mesh["j"], mesh["k"]):
        tri_x = (float(xs[a]), float(xs[b]), float(xs[c]))
        if min(tri_x) < x_min or max(tri_x) > x_max:
            continue
        out_i.append(add_vertex(a))
        out_j.append(add_vertex(b))
        out_k.append(add_vertex(c))

    clipped = dict(mesh)
    clipped.update({
        "x": out_x,
        "y": out_y,
        "z": out_z,
        "i": out_i,
        "j": out_j,
        "k": out_k,
        "triangle_count": len(out_i),
    })
    return clipped


def html_aspect_ratio(
    x_range: tuple[float, float],
    meshes: list[dict],
    points: list[list[float]] | None = None,
    sections: list[dict] | None = None,
) -> dict:
    ys = []
    zs = []
    for mesh in meshes:
        ys.extend(float(v) for v in mesh.get("y", []))
        zs.extend(float(v) for v in mesh.get("z", []))

    for point in points or []:
        if len(point) >= 3 and float(x_range[0]) <= float(point[0]) <= float(x_range[1]):
            ys.append(float(point[1]))
            zs.append(float(point[2]))

    for sec in sections or []:
        for reg in sec.get("free_meshes", []):
            ys.extend(float(v) for v in reg.get("y", []))
            zs.extend(float(v) for v in reg.get("z", []))
        for comp_key in ("hard_lines", "soft_lines"):
            for comp in sec.get(comp_key, []):
                for line in comp.get("lines", []):
                    for y, z in line:
                        ys.append(float(y))
                        zs.append(float(z))

    dx = max(float(x_range[1]) - float(x_range[0]), 1.0)
    dy = max(max(ys) - min(ys), 1.0) if ys else 1.0
    dz = max(max(zs) - min(zs), 1.0) if zs else 1.0
    scale = max(dx, dy, dz, 1.0)
    return {"x": dx / scale, "y": dy / scale, "z": dz / scale}


def load_step_meshes_for_html(settings, x_range: tuple[float, float] | None = None) -> list[dict]:
    meshes = []
    for path in scan_step_files(settings.BASE_STEP_DIR):
        role, clearance, solid = component_rule(path, settings)
        if role == "ignore":
            continue
        try:
            shape = read_cad_shape(path)
            mesh = triangulate_shape_to_mesh(
                shape,
                settings.MESH_LINEAR_DEFLECTION,
                settings.MESH_ANGULAR_DEFLECTION,
                None,
            )
            mesh = clip_mesh_to_x_range(mesh, x_range)
            if not mesh["i"]:
                continue
            meshes.append({
                "name": path.name,
                "role": role,
                "boundary": component_boundary_mode(path, role, settings),
                "clearance": float(clearance),
                "solid": bool(solid),
                "x": mesh["x"],
                "y": mesh["y"],
                "z": mesh["z"],
                "i": mesh["i"],
                "j": mesh["j"],
                "k": mesh["k"],
            })
        except Exception as exc:
            print(f"[WARN] HTML mesh failed {path.name}: {type(exc).__name__}: {exc}")
    return meshes


def export_planning_space_html(space: dict, settings, output: Path) -> None:
    print("[HTML] building planning space preview...")
    stride = max(1, int(getattr(settings, "SPACE_HTML_SECTION_STRIDE", 4)))
    x_min, x_max = html_x_range(settings)
    sections = space.get("sections", [])
    sections = [sec for sec in sections if x_min - 1e-6 <= float(sec["x"]) <= x_max + 1e-6]
    sampled_sections = sections[::stride]
    if sections and sampled_sections and sampled_sections[-1] is not sections[-1]:
        sampled_sections.append(sections[-1])

    section_records = []
    for sec in sampled_sections:
        free_meshes = []
        for region in sec.get("free_regions", []):
            mesh = polygon_display_mesh(region)
            if mesh is not None:
                free_meshes.append(mesh)
        section_records.append({
            "x": float(sec["x"]),
            "free_meshes": free_meshes,
            "hard_lines": sec.get("hard_obstacle_section_lines", sec.get("obstacle_section_lines", [])),
            "soft_lines": sec.get("soft_obstacle_section_lines", []),
        })

    meshes = load_step_meshes_for_html(settings, (x_min, x_max))
    payload = {
        "meshes": meshes,
        "sections": section_records,
        "start": space.get("start", settings.START),
        "goal": space.get("goal", settings.GOAL),
        "x_range": [x_min, x_max],
        "aspect_ratio": html_aspect_ratio(
            (x_min, x_max),
            meshes,
            [space.get("start", settings.START), space.get("goal", settings.GOAL)],
            section_records,
        ),
        "meta": space.get("meta", {}),
    }
    data_json = json.dumps(payload, ensure_ascii=False)

    html = """
<!doctype html>
<html lang="zh-CN">
<head>
  <meta charset="utf-8" />
  <title>Planning Space Preview</title>
  <script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
  <style>
    html, body { margin: 0; height: 100%; font-family: Arial, sans-serif; }
    #plot { width: 100vw; height: 100vh; }
    #panel {
      position: fixed; left: 12px; top: 12px; z-index: 10;
      background: rgba(255,255,255,.86); border: 1px solid #d0d0d0;
      padding: 8px 10px; font-size: 13px; line-height: 1.45;
    }
  </style>
</head>
<body>
  <div id="panel">
    <strong>Planning Space</strong><br>
    Drag: rotate / Shift+drag: pan / Wheel: zoom<br>
    Green: free section area, red: hard boundary, blue: soft boundary
  </div>
  <div id="plot"></div>
  <script>
const DATA = __DATA__;
const traces = [];

for (const mesh of DATA.meshes) {
  traces.push({
    type: "mesh3d",
    name: mesh.name,
    x: mesh.x, y: mesh.y, z: mesh.z,
    i: mesh.i, j: mesh.j, k: mesh.k,
    color: mesh.boundary === "soft" ? "#60a5fa" : mesh.role === "ground" ? "#9ca3af" : "#9b9b9b",
    opacity: mesh.boundary === "soft" ? 0.18 : 0.23,
    flatshading: true,
    hoverinfo: "name",
  });
}

for (const sec of DATA.sections) {
  for (const reg of sec.free_meshes) {
    traces.push({
      type: "mesh3d",
      name: "free x=" + sec.x.toFixed(1),
      x: reg.y.map(() => sec.x),
      y: reg.y,
      z: reg.z,
      i: reg.i, j: reg.j, k: reg.k,
      color: "#22c55e",
      opacity: 0.16,
      hoverinfo: "skip",
      showscale: false,
    });
  }
  for (const comp of sec.hard_lines) {
    for (const line of comp.lines || []) {
      traces.push({
        type: "scatter3d",
        mode: "lines",
        name: "hard " + comp.name,
        x: line.map(() => sec.x),
        y: line.map(p => p[0]),
        z: line.map(p => p[1]),
        line: { color: "#dc2626", width: 2 },
        hoverinfo: "name",
        showlegend: false,
      });
    }
  }
  for (const comp of sec.soft_lines) {
    for (const line of comp.lines || []) {
      traces.push({
        type: "scatter3d",
        mode: "lines",
        name: "soft " + comp.name,
        x: line.map(() => sec.x),
        y: line.map(p => p[0]),
        z: line.map(p => p[1]),
        line: { color: "#2563eb", width: 2 },
        hoverinfo: "name",
        showlegend: false,
      });
    }
  }
}

traces.push({
  type: "scatter3d",
  mode: "markers+text",
  name: "start / goal",
  x: [DATA.start[0], DATA.goal[0]],
  y: [DATA.start[1], DATA.goal[1]],
  z: [DATA.start[2], DATA.goal[2]],
  text: ["START", "GOAL"],
  textposition: "top center",
  marker: { size: 6, color: ["#16a34a", "#dc2626"] },
});

Plotly.newPlot("plot", traces, {
  margin: { l: 0, r: 0, t: 0, b: 0 },
  scene: {
    aspectmode: "manual",
    aspectratio: DATA.aspect_ratio,
    xaxis: { title: "X mm", range: DATA.x_range },
    yaxis: { title: "Y mm" },
    zaxis: { title: "Z mm" },
  },
  legend: { x: 1, xanchor: "right", y: 1 },
}, { responsive: true, scrollZoom: true });
  </script>
</body>
</html>
""".replace("__DATA__", data_json)

    output = Path(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(html, encoding="utf-8")
    print(f"[HTML] saved: {output}")
