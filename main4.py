# -*- coding: utf-8 -*-
"""
Create a CATIA CATPart from a main3 engineered pipe route.

The centerline is built from main3's line/arc segments, not a fitted spline.

Example:
    & c:\\envs\\occ_env\\python.exe c:/Users/user/Desktop/PY3/main4.py --case1
"""

from __future__ import annotations

import argparse
import json
import math
import re
import sys
import time
from pathlib import Path

import settings


def load_json(path: Path) -> dict:
    return json.loads(Path(path).read_text(encoding="utf-8"))


def safe_name(value: str) -> str:
    value = re.sub(r"[^0-9A-Za-z_]+", "_", str(value)).strip("_")
    return value or "route"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a CATIA CATPart from a main3 engineered pipe route."
    )
    cases = parser.add_mutually_exclusive_group()
    for idx in range(1, 6):
        cases.add_argument(
            f"--case{idx}",
            action="store_const",
            const=idx,
            dest="case_index",
            help=f"Use engineered route {idx}.",
        )
    cases.add_argument("--route-index", type=int, help="1-based engineered route index.")
    cases.add_argument("--route-name", help="Exact engineered route name.")

    parser.add_argument(
        "--input",
        type=Path,
        default=Path(getattr(settings, "ENGINEERING_RESULT_JSON")),
        help="main3 route_engineered.json path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output CATPart path. Defaults to outputs/catia/<route_name>.CATPart.",
    )
    parser.add_argument(
        "--radius",
        type=float,
        default=None,
        help="Override the core pipe outer radius in mm.",
    )
    parser.add_argument(
        "--visible",
        action="store_true",
        help="Show CATIA window while generating.",
    )
    parser.add_argument(
        "--centerline-only",
        action="store_true",
        help="Only create nodes and line/arc center curve, skip the circular swept pipe.",
    )
    return parser.parse_args()


def select_route(result: dict, args: argparse.Namespace) -> tuple[int, dict]:
    routes = result.get("routes") or []
    if not routes:
        raise ValueError("No routes found in route_engineered.json. Run main3.py first.")

    if args.route_name:
        for idx, route in enumerate(routes, start=1):
            if route.get("name") == args.route_name:
                return idx, route
        names = ", ".join(str(route.get("name")) for route in routes)
        raise ValueError(f"Route name not found: {args.route_name}. Available: {names}")

    index = args.route_index or args.case_index or 1
    if index < 1 or index > len(routes):
        raise ValueError(f"Route index out of range: {index}. Available: 1..{len(routes)}")
    return index, routes[index - 1]


def import_catia_client():
    try:
        import win32com.client
        return win32com.client
    except Exception as exc:
        raise RuntimeError(
            "main4.py requires pywin32 and a local CATIA installation. "
            "Install pywin32 in the Python environment used to run this script."
        ) from exc


def append_shape(part, hybrid_body, shape):
    hybrid_body.AppendHybridShape(shape)
    try:
        part.UpdateObject(shape)
    except Exception:
        pass
    return shape


def try_set_name(obj, name: str) -> None:
    try:
        obj.Name = str(name)
    except Exception:
        pass


def try_set_part_identity(part_doc, part, name: str) -> None:
    try_set_name(part_doc, name)
    try_set_name(part, name)
    try:
        part.PartNumber = str(name)
    except Exception:
        pass


def create_point(factory, part, hybrid_body, point):
    p = factory.AddNewPointCoord(float(point[0]), float(point[1]), float(point[2]))
    return append_shape(part, hybrid_body, p)


def create_line(factory, part, hybrid_body, p1, p2):
    line = factory.AddNewLinePtPt(part.CreateReferenceFromObject(p1), part.CreateReferenceFromObject(p2))
    return append_shape(part, hybrid_body, line)


def vector(a, b):
    return [float(b[i]) - float(a[i]) for i in range(3)]


def point_add(a, b):
    return [float(a[i]) + float(b[i]) for i in range(3)]


def point_scale(a, scale: float):
    return [float(a[i]) * float(scale) for i in range(3)]


def vec_norm(v) -> float:
    return math.sqrt(sum(float(x) * float(x) for x in v))


def vec_unit(v):
    n = vec_norm(v)
    if n < 1e-9:
        raise ValueError("Zero-length vector.")
    return [float(x) / n for x in v]


def arc_midpoint(segment: dict) -> list[float]:
    center = segment["center"]
    start = segment["start"]
    end = segment["end"]
    radius = float(segment.get("radius", settings.ENGINEERING_BEND_RADIUS))
    u = vec_unit(vector(center, start))
    v = vec_unit(vector(center, end))
    mid_dir = [u[i] + v[i] for i in range(3)]
    if vec_norm(mid_dir) < 1e-9:
        mid_dir = u
    mid_dir = vec_unit(mid_dir)
    return point_add(center, point_scale(mid_dir, radius))


def set_arc_limitation(shape) -> None:
    for value in (2, 1):
        try:
            shape.SetLimitation(value)
            return
        except Exception:
            pass


def create_arc_3pt(factory, part, hybrid_body, p_start, p_mid, p_end):
    start_ref = part.CreateReferenceFromObject(p_start)
    mid_ref = part.CreateReferenceFromObject(p_mid)
    end_ref = part.CreateReferenceFromObject(p_end)
    arc = factory.AddNewCircle3Points(start_ref, mid_ref, end_ref)
    set_arc_limitation(arc)
    return append_shape(part, hybrid_body, arc)


def create_segment_curves(factory, part, nodes_body, curve_body, route_name: str, segments: list[dict]):
    curves = []
    point_cache: dict[tuple[float, float, float], object] = {}

    def point_key(point):
        return tuple(round(float(x), 6) for x in point)

    def get_point(point, prefix: str):
        key = point_key(point)
        if key in point_cache:
            return point_cache[key]
        catia_point = create_point(factory, part, nodes_body, point)
        try_set_name(catia_point, f"{prefix}_{len(point_cache) + 1:03d}")
        point_cache[key] = catia_point
        return catia_point

    for idx, segment in enumerate(segments, start=1):
        seg_type = segment.get("type")
        if seg_type == "line":
            p1 = get_point(segment["start"], "P")
            p2 = get_point(segment["end"], "P")
            line = create_line(factory, part, curve_body, p1, p2)
            try_set_name(line, f"{route_name}_L{idx:03d}")
            curves.append(line)
        elif seg_type == "arc":
            p_start = get_point(segment["start"], "P")
            p_end = get_point(segment["end"], "P")
            p_mid = get_point(arc_midpoint(segment), "A_mid")
            arc = create_arc_3pt(factory, part, curve_body, p_start, p_mid, p_end)
            try_set_name(arc, f"{route_name}_A{idx:03d}")
            curves.append(arc)
        else:
            raise ValueError(f"Unsupported segment type: {seg_type}")

    return curves, point_cache


def create_join_centerline(factory, part, hybrid_body, curves):
    if not curves:
        raise ValueError("Cannot create centerline join without segment curves.")
    if len(curves) == 1:
        return curves[0]

    join = factory.AddNewJoin(
        part.CreateReferenceFromObject(curves[0]),
        part.CreateReferenceFromObject(curves[1]),
    )
    for curve in curves[2:]:
        join.AddElement(part.CreateReferenceFromObject(curve))
    try:
        join.SetConnex(1)
    except Exception:
        pass
    try:
        join.SetManifold(1)
    except Exception:
        pass
    return append_shape(part, hybrid_body, join)


def try_call(obj, method_name: str, *args) -> bool:
    try:
        getattr(obj, method_name)(*args)
        return True
    except Exception:
        return False


def try_set_attr(obj, attr_name: str, value) -> bool:
    try:
        setattr(obj, attr_name, value)
        return True
    except Exception:
        return False


def set_visual_color(obj, color: str | None) -> None:
    """Best-effort CATIA display color; geometry remains the source of truth."""
    if not color:
        return
    value = str(color).lstrip("#")
    if len(value) != 6:
        return
    try:
        rgb = tuple(int(value[index:index + 2], 16) for index in (0, 2, 4))
        obj.VisProperties.SetRealColor(*rgb, 1)
    except Exception:
        pass


def create_circular_sweep(
    part,
    hybrid_body,
    center_curve,
    radius: float,
    route_name: str,
    color: str | None = None,
):
    factory = part.HybridShapeFactory
    center_ref = part.CreateReferenceFromObject(center_curve)
    sweep = factory.AddNewSweepCircle(center_ref)

    # CATGSMCircularSweep_CenterAndRadius: circular profile from center curve + radius.
    try_set_attr(sweep, "Mode", 6)
    try_set_attr(sweep, "Context", 1)
    try_set_attr(sweep, "TrimOption", 1)
    try_set_attr(sweep, "SmoothActivity", False)
    try_set_attr(sweep, "GuideDeviationActivity", False)
    try_call(sweep, "SetRadius", 1, float(radius))
    try_call(sweep, "SetRadius", 2, float(radius))

    try_set_name(sweep, f"{route_name}_circular_sweep_r{radius:g}")
    set_visual_color(sweep, color)
    set_visual_color(hybrid_body, color)
    return append_shape(part, hybrid_body, sweep)


def normalize_path(path: Path) -> str:
    return str(Path(path).resolve()).lower().replace("/", "\\")


def close_existing_document_for_path(catia, output: Path, current_doc=None) -> None:
    target = normalize_path(output)
    documents = catia.Documents
    to_close = []
    try:
        count = int(documents.Count)
    except Exception:
        return

    for idx in range(1, count + 1):
        try:
            doc = documents.Item(idx)
        except Exception:
            continue
        if current_doc is not None:
            try:
                if doc is current_doc or doc._oleobj_ == current_doc._oleobj_:
                    continue
            except Exception:
                pass
        try:
            full_name = normalize_path(Path(doc.FullName))
        except Exception:
            full_name = ""
        try:
            name = str(doc.Name).lower()
        except Exception:
            name = ""
        if full_name == target or name == Path(output).name.lower():
            to_close.append(doc)

    for doc in to_close:
        try:
            print(f"[SAVE] closing existing CATIA document: {doc.Name}", flush=True)
            doc.Close()
        except Exception as exc:
            print(f"[WARN] failed to close existing document: {exc}", flush=True)


def save_part_document(catia, part_doc, output: Path) -> Path:
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    close_existing_document_for_path(catia, output, part_doc)

    candidates = [output]
    stamp = time.strftime("%Y%m%d_%H%M%S")
    candidates.append(output.with_name(f"{output.stem}_{stamp}{output.suffix}"))

    last_error = None
    for candidate in candidates:
        try:
            if candidate.exists():
                print(f"[SAVE] removing existing file: {candidate}", flush=True)
                candidate.unlink()
            print(f"[SAVE] writing CATPart: {candidate}", flush=True)
            part_doc.SaveAs(str(candidate))
            print(f"[SAVE] CATPart: {candidate}", flush=True)
            return candidate
        except Exception as exc:
            last_error = exc
            print(f"[WARN] SaveAs failed for {candidate}: {exc}", flush=True)

    raise RuntimeError(f"CATIA SaveAs failed for all output paths: {last_error}")


def create_catia_part(route: dict, output: Path, radius: float, visible: bool, centerline_only: bool) -> None:
    # CATIA is the final export boundary. A route explicitly marked as a
    # diagnostic or carrying collision evidence must never be turned into a
    # formal CATPart, even when this function is called directly.
    if route.get("diagnostic_only") or route.get("formal_valid") is False or route.get("exportable") is False:
        raise ValueError("diagnostic or non-exportable routes cannot generate CATPart")
    if route.get("hard_conflicts") or route.get("inter_pipe_conflicts"):
        raise ValueError("routes with hard or inter-pipe conflicts cannot generate CATPart")
    if any(not item.get('valid', False) for item in route.get('double_clamp_reports', [])):
        raise ValueError('routes with invalid double clamps cannot generate CATPart')
    if route.get('protected_waypoint_geometry', {}).get('valid') is False:
        raise ValueError('invalid guardian geometry cannot generate CATPart')
    for key in ("geometry_report", "constraint_report"):
        report = route.get(key)
        if isinstance(report, dict) and report.get("valid") is False:
            raise ValueError(f"route {key} is invalid; CATPart export is blocked")

    win32 = import_catia_client()
    catia = win32.Dispatch("CATIA.Application")
    catia.Visible = bool(visible)
    try_set_attr(catia, "DisplayFileAlerts", False)

    documents = catia.Documents
    part_doc = documents.Add("Part")
    part = part_doc.Part

    hybrid_bodies = part.HybridBodies
    route_name = safe_name(route.get("name", "route"))
    try_set_part_identity(part_doc, part, route_name)
    nodes_body = hybrid_bodies.Add()
    try_set_name(nodes_body, f"{route_name}_nodes")
    curve_body = hybrid_bodies.Add()
    try_set_name(curve_body, f"{route_name}_line_arc_center_curve")

    factory = part.HybridShapeFactory
    segments = route.get("segments") or []
    if not segments:
        raise ValueError(f"Route {route_name} has no line/arc segments. Run main3.py first.")

    print(
        f"[CATIA] route={route.get('name')} "
        f"segments={len(segments)} radius={radius:.2f}"
    )
    components = route.get("component_segments") or []
    if components:
        # main3 clips the original line/arc geometry at exact cumulative
        # distances, so the aluminium core is replaced by three separate
        # sweeps instead of being left as an overlapping full-length tube.
        total_nodes = 0
        for component_index, component in enumerate(components, start=1):
            component_segments = component.get("segments") or []
            if not component_segments:
                continue
            component_kind = safe_name(component.get("kind", "component"))
            component_body = hybrid_bodies.Add()
            body_name = f"{route_name}_{component_kind}_{component_index:02d}"
            try_set_name(component_body, body_name)
            segment_curves, point_cache = create_segment_curves(
                factory,
                part,
                nodes_body,
                component_body,
                body_name,
                component_segments,
            )
            total_nodes += len(point_cache)
            center_curve = create_join_centerline(factory, part, component_body, segment_curves)
            try_set_name(center_curve, f"{body_name}_centerline")
            if not centerline_only:
                create_circular_sweep(
                    part,
                    component_body,
                    center_curve,
                    float(component.get("outer_radius", radius)),
                    body_name,
                    component.get("color"),
                )
            print(
                f"[CATIA] component={component_kind} index={component_index} "
                f"length={float(component.get('length_mm', 0.0)):.2f} mm "
                f"radius={float(component.get('outer_radius', radius)):.2f}"
            )
        print(f"[CATIA] segmented nodes={total_nodes}, components={len(components)}")
        part.Update()
    else:
        segment_curves, point_cache = create_segment_curves(
            factory,
            part,
            nodes_body,
            curve_body,
            route_name,
            segments,
        )
        print(f"[CATIA] nodes={len(point_cache)}, curves={len(segment_curves)}")

        center_curve = create_join_centerline(factory, part, curve_body, segment_curves)
        try_set_name(center_curve, f"{route_name}_line_arc_join")
        print("[CATIA] center curve: joined line + arc segments")

        part.Update()

        if not centerline_only:
            try:
                create_circular_sweep(part, curve_body, center_curve, radius, route_name)
                part.Update()
                print("[CATIA] circular sweep pipe created")
            except Exception as exc:
                raise RuntimeError(
                    f"CATIA circular sweep failed for formally valid route {route_name}; "
                    "the CATPart was not saved"
                ) from exc

    save_part_document(catia, part_doc, output)


def main() -> None:
    args = parse_args()
    result = load_json(args.input)
    index, route = select_route(result, args)

    output = args.output
    if output is None:
        route_name = safe_name(route.get("name", f"case{index}"))
        output = Path(getattr(settings, "OUTPUT_DIR")) / "catia" / f"{index:02d}_{route_name}.CATPart"

    print("=== Main4: CATIA pipe part export ===")
    print(f"ENGINEERING_RESULT_JSON: {args.input}")
    print(f"selected route: case{index} {route.get('name')}")
    print(f"output: {output}")
    radius = args.radius
    if radius is None:
        radius = float(route.get("core_outer_radius", getattr(settings, "PIPE_RADIUS", 25.0)))
    create_catia_part(route, output, radius, args.visible, args.centerline_only)
    print("=== Done ===")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
