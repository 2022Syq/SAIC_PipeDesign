"""工作台的工程数据边界：沿用现有规范、精确曲线及真实校核报告。"""
from __future__ import annotations

import copy
from dataclasses import asdict
import math

import numpy as np

import settings
from common_space import ensure_shapely
from hose_segments import build_hose_layout, arc_radius_intervals, _hose_radius_report
from pipe_config import RunConfig
from .geometry import RouteCurve, rebuild_local

ensure_shapely()

COVERS = {"metal_9": "hose_8DE3", "metal_12": "hose_11DE3",
          "metal_16": "hose_15DE3", "metal_19_05": "hose_15DE3"}
# Source: doc/SAAA工艺要求-20260706.xlsx, Sheet1. The workbook is a local input;
# retaining the transcribed cells makes a delivered workbench independent of it.
TEETH = {"metal_9": 4.5, "metal_12": 7.2, "metal_16": 11.2, "metal_19_05": 11.2}
SADDLES = [
    ["SKC-9AA", 9, 9, 15.4, 16.75, 21.25],
    ["SKC-9AC", 9.53, 12.7, 17.2, 23, 30],
    ["SKC-12AA", 12, 12, 17.4, 21.6, 27.6],
    ["SKC-16AA", 16, 12, 23, 21.6, 27.6],
    ["SKC-16AC", 16, 16, 25, 27.6, 31.6],
    ["SKC-19AF", 19.05, 16, 26, 27.6, 31.6],
    ["SKC-19AB", 19.05, 19.05, 28, 33.025, 38.025],
]

# Keys exposed by the inspector.  Values not used by the current geometry
# engine are still persisted so an engineer can keep the complete process card
# with the project and review it on the next visit.
EDITABLE_PARAMETER_KEYS = {
    "metal_material", "metal_outer_diameter", "metal_wall_thickness", "metal_inner_diameter",
    "metal_bend_radius_recommended", "metal_bend_radius_minimum",
    "straight_recommended", "straight_minimum", "teeth_min_inner_diameter",
    "hose_material", "hose_outer_diameter", "hose_wall_thickness", "hose_bend_radius_recommended",
    "hose_bend_radius_minimum", "bend_angle_recommended", "bend_angle_minimum",
    "weld_spacing_recommended", "weld_spacing_minimum", "valve_straight_recommended",
    "valve_straight_minimum", "teeth_insert_length_recommended", "teeth_insert_length_minimum",
    "sleeve_back_recommended", "sleeve_back_minimum", "sleeve_axis_clearance_recommended",
    "sleeve_axis_clearance_minimum", "required_clearance", "hose_length_minimum",
}


def default_hose(spec):
    return {"core_spec": spec, "cover_spec": COVERS[spec], "min_length_mm": 30.,
            "hose_length_mm": 30., "sleeve_length_mm": 30.}


def config_payload(config):
    return {"pipes": [asdict(p) for p in config.pipes],
            "double_clamps": [asdict(c) for c in config.double_clamps]}


def with_specs(config, selections, parameter_edits=None):
    """Only configured specifications change; measured constraints remain intact."""
    payload = config_payload(config)
    if set(selections) != {p.pipe_id for p in config.pipes}:
        raise ValueError("规格列表与已识别管路不一致")
    parameter_edits = parameter_edits or {}
    if set(parameter_edits) - set(selections):
        raise ValueError("参数列表与已识别管路不一致")
    for pipe in payload["pipes"]:
        spec = selections[pipe["pipe_id"]]
        if spec not in COVERS:
            raise ValueError("请选择工艺表中的金属管规格")
        pipe["pipe_spec"] = spec
        entries = pipe["pipe_segments"]
        if len(entries) > 1:
            raise ValueError("首版每根管仅支持一段胶管，当前配置包含多段")
        entry = dict(entries[0]) if entries else default_hose(spec)
        entry.update(core_spec=spec, cover_spec=COVERS[spec],
                     minimum_bend_radius_mm=settings.PIPE_SPECS[COVERS[spec]]["minimum_bend_radius"])
        pipe["pipe_segments"] = [entry]
        edits = dict(pipe.get("engineering_parameters") or {})
        for key, value in dict(parameter_edits.get(pipe["pipe_id"], {})).items():
            if key not in EDITABLE_PARAMETER_KEYS:
                raise ValueError(f"不支持的工程参数：{key}")
            if value is not None and not isinstance(value, (str, int, float, bool)):
                raise ValueError(f"工程参数格式无效：{key}")
            if value is not None and key not in {"metal_material", "hose_material"}:
                try:
                    numeric = float(value)
                except (TypeError, ValueError) as exc:
                    raise ValueError(f"工程参数必须为数字：{key}") from exc
                if not math.isfinite(numeric) or numeric < 0:
                    raise ValueError(f"工程参数必须为非负有限数：{key}")
            if value is None:
                edits.pop(key, None)
            else:
                edits[key] = value
        if "required_clearance" in edits:
            edits["required_clearance"] = float(edits["required_clearance"] or 0.0)
            pipe["required_clearance"] = edits["required_clearance"]
        if "hose_bend_radius_minimum" in edits:
            pipe["pipe_segments"][0]["minimum_bend_radius_mm"] = float(edits["hose_bend_radius_minimum"])
        if "hose_length_minimum" in edits:
            minimum = float(edits["hose_length_minimum"])
            entry = pipe["pipe_segments"][0]
            entry["min_length_mm"] = minimum
            entry["length_options_mm"] = sorted(set([30.0, 75.0, minimum]))
            entry["hose_length_mm"] = max(float(entry.get("hose_length_mm", minimum)), minimum)
        pipe["engineering_parameters"] = edits
    return RunConfig.from_dict(payload)


def fixed_intervals(route, pipe):
    """Freeze the 15 mm straight bridge on each side of every measured GUADIAN."""
    curve = RouteCurve(route["segments"])
    result = []
    for guardian in pipe.required_hangers:
        point = np.asarray(guardian.point)
        stations = [float(curve.marks[i]) for i, p in enumerate(curve.pieces)
                    if np.linalg.norm(p.start-point) < 1e-5]
        if not stations:
            raise ValueError(f"{pipe.pipe_id} 缺少挂点 {guardian.name} 的精确曲线节点")
        s = stations[0]
        result.append([max(0., s-15.), min(curve.length, s+15.)])
    return result


def hose_entry(route, pipe):
    entries = route.get("pipe_segments") or list(pipe.pipe_segments) or [default_hose(pipe.pipe_spec)]
    if len(entries) != 1:
        raise ValueError("首版每根管需要且仅支持一段胶管")
    entry = copy.deepcopy(entries[0])
    components = route.get("hose_layout", {}).get("components", [])
    hose = next((c for c in components if c["kind"] == "hose"), None)
    sleeve = next((c for c in components if c["kind"] == "sleeve"), None)
    if hose:
        entry.update(position_s_mm=(hose["start_s"]+hose["end_s"])/2,
                     hose_length_mm=hose["end_s"]-hose["start_s"])
    if sleeve:
        entry["sleeve_length_mm"] = sleeve["end_s"]-sleeve["start_s"]
    entry.pop("position_ratio", None)
    return entry


def refresh_route(route, pipe, entry, model=None):
    """Refresh all geometry-dependent fields, without replacing failures by success."""
    from main3 import (active_constraint_profile, validate_engineered_geometry,
                       validate_hard_waypoint_geometry, validate_route_constraints,
                       make_engineering_params, physical_bends, _hose_local_clearance_report)
    curve = RouteCurve(route["segments"])
    sampled, marks = curve.sample()
    effective_specs = {key: dict(value) for key, value in settings.PIPE_SPECS.items()}
    overrides = getattr(pipe, "engineering_parameters", {})
    if pipe.pipe_spec in effective_specs and "metal_outer_diameter" in overrides:
        effective_specs[pipe.pipe_spec]["outer_diameter"] = float(overrides["metal_outer_diameter"])
    cover_id = str(entry.get("cover_spec", COVERS[pipe.pipe_spec]))
    if cover_id in effective_specs:
        for key, value_key in (("material", "hose_material"), ("outer_diameter", "hose_outer_diameter"),
                               ("wall_thickness", "hose_wall_thickness"), ("minimum_bend_radius", "hose_bend_radius_minimum")):
            if value_key in overrides:
                effective_specs[cover_id][key] = overrides[value_key]
    layout = build_hose_layout(curve.length, [entry], effective_specs,
                               route["segments"], allow_constraint_warnings=True)
    local = _hose_local_clearance_report(sampled, layout, model, pipe.planning_radius)
    layout["local_clearance"] = local
    if model is not None and not local["valid"]:
        layout["hard_failures"].append({"rule": "hose_local_clearance", "actual_mm": local["minimum_margin_mm"]})
        layout["valid"] = False
    profile = active_constraint_profile(pipe.pipe_spec, pipe.planning_radius, [entry],
                                        getattr(pipe, "engineering_parameters", None))
    params = make_engineering_params(pipe)
    geometry = validate_engineered_geometry(route["segments"], pipe.start, pipe.goal,
        np.asarray(pipe.start_dir)-pipe.start, np.asarray(pipe.goal_dir)-pipe.goal)
    protected = validate_hard_waypoint_geometry(route["segments"],
        [g.point for g in pipe.required_hangers], 15.,
        tangent_directions=params["hard_waypoint_tangent_directions"],
        tangent_tolerance_deg=params["hard_waypoint_tangent_tolerance_deg"])
    # Exact component curves are authoritative for display and CATIA. Use exact
    # curve stations rather than sampled chord lengths to avoid sleeve drift.
    components = []
    from hose_segments import COMPONENT_COLORS
    for item in layout["components"]:
        pieces = curve.clip(item["start_s"], item["end_s"])
        if pieces:
            components.append({**item, "color": COMPONENT_COLORS[item["kind"]],
                               "segments": pieces,
                               "length_mm": item["end_s"]-item["start_s"]})
    report = validate_route_constraints(sampled, route, profile, model, layout,
        metal_segment_groups=[c["segments"] for c in components if c["kind"] == "aluminum"],
        sleeve_specs=[{"spec": pipe.pipe_spec if pipe.pipe_spec == "metal_19_05" else entry["cover_spec"],
                       "start_s": c["start_s"], "end_s": c["end_s"]}
                      for c in components if c["kind"] == "sleeve"])
    bends = physical_bends(route["segments"])
    result = copy.deepcopy(route)
    # These old meshes/reports must not leak into saved results after an edit.
    for key in ("tube_mesh", "component_meshes", "hard_conflicts", "inter_pipe_conflicts",
                "hard_conflict_components", "collision_report", "double_clamp_reports"):
        result.pop(key, None)
    result.update(pipe_id=pipe.pipe_id, pipe_segments=[entry],
        core_spec=pipe.pipe_spec, active_pipe_spec=pipe.pipe_spec,
        cover_spec=entry["cover_spec"], core_outer_radius=pipe.planning_radius,
        engineered_path=sampled.tolist(), engineered_length=curve.length,
        component_segments=components, hose_layout=layout, constraint_report=report,
        geometry_report=geometry, protected_waypoint_geometry=protected,
        arc_count=sum(p.kind == "arc" for p in curve.pieces), turn_count=len(bends),
        main_turn_count=len(bends), max_turn_angle_deg=max((b["angle_deg"] for b in bends), default=0.),
        bend_radius=min((b["radius"] for b in bends), default=None),
        ui_environment_checked=model is not None)
    if model is not None:
        from main2 import joint_validate_path, hard_conflict_report
        result["collision_report"] = joint_validate_path(model, sampled)
        conflicts = hard_conflict_report(model, sampled)
        result["hard_conflicts"] = conflicts["segments"]
        result["hard_conflict_components"] = conflicts["components"]
    result["formal_valid"] = bool(geometry["valid"] and protected["valid"] and report["valid"]
                                  and model is not None and not result.get("hard_conflicts"))
    result["exportable"] = bool(geometry["valid"] and protected["valid"] and not result.get("diagnostic_only"))
    result["ui_export_policy"] = "constraint_warnings_allowed"
    return result


def edit_route(route, pipe, edit, model=None):
    entry = hose_entry(route, pipe)
    candidate = copy.deepcopy(route)
    if edit["kind"] == "hose":
        # Sleeve length is intentionally not accepted from the browser.
        entry.update(position_s_mm=float(edit["center"]), hose_length_mm=float(edit["length"]))
    elif edit["kind"] == "route":
        segments, report = rebuild_local(route["segments"], float(edit["station"]),
            edit["delta"], float(edit.get("influence", 200)), fixed_intervals=fixed_intervals(route, pipe))
        candidate["segments"] = segments
        candidate["editor_edit"] = report
    else:
        raise ValueError("未知的编辑操作")
    candidate = refresh_route(candidate, pipe, entry, model)
    if not candidate["geometry_report"]["valid"] or not candidate["protected_waypoint_geometry"]["valid"]:
        raise ValueError("此次修改破坏了固定端点、挂点或曲线连续性，原路线已保留")
    return candidate


def check_pairs(routes, config):
    from route_clearance import check_pipe_clearance
    from multi_pipeline import _double_clamp_reports
    pipes = {p.pipe_id: p for p in config.pipes}
    for route in routes:
        route["inter_pipe_conflicts"] = []
        route["ui_pair_reports"] = []
        route["formal_valid"] = bool(route.get("geometry_report", {}).get("valid")
            and route.get("protected_waypoint_geometry", {}).get("valid")
            and route.get("constraint_report", {}).get("valid")
            and route.get("ui_environment_checked") and not route.get("hard_conflicts"))
    for i, left in enumerate(routes):
        for right in routes[i+1:]:
            report = check_pipe_clearance(pipes[left["pipe_id"]], left,
                                          pipes[right["pipe_id"]], right)
            for route in (left, right):
                route["ui_pair_reports"].append(report)
                if not report["valid"]:
                    route["inter_pipe_conflicts"].append(report)
                    route["formal_valid"] = False
    reports = _double_clamp_reports(config, pipes, {r["pipe_id"]: r for r in routes})
    for route in routes:
        route["double_clamp_reports"] = [r for r in reports if route["pipe_id"] in (r["pipe_id_a"], r["pipe_id_b"])]
        if any(not r["valid"] for r in route["double_clamp_reports"]):
            route["formal_valid"] = False


RULE_NAMES = {
    "bend_radius": "金属弯曲半径不足", "uniform_bend_radius": "同一管路存在不同弯曲半径",
    "bend_angle": "弯管角度不足", "bend_spacing": "弯间直线段不足", "hose_bend_radius": "胶管弯曲半径不足",
    "hose_length": "胶管长度小于配置下限", "hose_local_clearance": "胶管或铝套与环境干涉",
    "weld_spacing": "相邻焊点距离不足", "valve_straight": "阀座至弯管距离不足",
    "aluminum_sleeve_axis_clearance": "铝套轴线操作空间不足",
    "aluminum_sleeve_back_length": "铝套背部直线不足",
    "aluminum_sleeve_back_length_missing": "铝套背部直线尺寸未提供，无法判定",
    "aluminum_sleeve_clearance_unchecked": "铝套轴线操作空间未校核",
    "hose_position_auto_shifted": "自动规划已调整胶管初始位置",
}


def warnings_for(route):
    result = []
    report = route.get("constraint_report", {})
    for level in ("hard_failures", "warnings"):
        for item in report.get(level, []):
            name = RULE_NAMES.get(item.get("rule"), item.get("rule", "工艺检查"))
            detail = []
            for key, label in (("actual_mm", "实际"), ("actual_deg", "实际角度"),
                               ("minimum_mm", "下限"), ("minimum_deg", "角度下限"),
                               ("minimum_actual_radius_mm", "实际半径"),
                               ("recommended_mm", "推荐"), ("recommended_deg", "推荐角度")):
                if item.get(key) is not None:
                    detail.append(f"{label} {float(item[key]):.2f}")
            result.append(name+("："+" / ".join(detail) if detail else ""))
    if not route.get("ui_environment_checked"):
        result.append("环境碰撞未重新校核：缺少当前模型的规划空间数据")
    if route.get("hard_conflicts"):
        result.append(f"与环境部件存在干涉（{len(route['hard_conflicts'])} 处）")
    for item in route.get("inter_pipe_conflicts", []):
        result.append(f"{item['pipe_id_a']} / {item['pipe_id_b']} 管间净距 {item['minimum_clearance']:.2f} mm，小于要求 {item['required_clearance']:.2f} mm")
    for item in route.get("double_clamp_reports", []):
        if not item["valid"]:
            result.append(f"双管夹 {item['name']} 不满足约束")
    if route.get("diagnostic_only"):
        result.append("未生成可建模的工程路线，不能导出")
    return list(dict.fromkeys(result))


def parameter_rows(pipe, route=None):
    """Return the full engineering card with explicit editable targets."""
    from main3 import physical_bends, engineered_segment_length
    spec = settings.PIPE_SPECS[pipe.pipe_spec]
    cover_id = COVERS[pipe.pipe_spec]
    cover = settings.PIPE_SPECS[cover_id]
    overrides = getattr(pipe, "engineering_parameters", {})
    rowno = list(COVERS).index(pipe.pipe_spec)+2
    rows = []

    def value(key, default):
        return overrides.get(key, default)

    def row(label, actual=None, recommended=None, minimum=None, cell="", unit="mm", status=None,
            actual_key=None, recommended_key=None, minimum_key=None, input_type="number"):
        if status is None:
            status = "unknown" if actual is None else "normal"
            if isinstance(actual, (int, float)):
                if minimum is not None and actual + 1e-6 < minimum:
                    status = "warning"
                elif recommended is not None and actual + 1e-6 < recommended:
                    status = "advisory"
        rows.append(dict(label=label, actual=actual, recommended=recommended, minimum=minimum,
                         source=f"Sheet1!{cell}" if cell else "当前路线 / 配置", unit=unit, status=status,
                         actual_key=actual_key, recommended_key=recommended_key,
                         minimum_key=minimum_key, input_type=input_type))

    for key, name, col, unit, edit_key in (("material", "金属牌号", "A", "", "metal_material"),
        ("outer_diameter", "金属外径", "B", "mm", "metal_outer_diameter"),
        ("wall_thickness", "金属壁厚", "C", "mm", "metal_wall_thickness"),
        ("inner_diameter", "金属内径", "D", "mm", "metal_inner_diameter")):
        row(name, value(edit_key, spec[key]), cell=f"{col}{rowno}", unit=unit,
            actual_key=None if route else edit_key, input_type="text" if not unit else "number")
    segments = route.get("segments", []) if route else []
    groups = [c["segments"] for c in (route or {}).get("component_segments", []) if c["kind"] == "aluminum"]
    grouped = [(g, physical_bends(g)) for g in groups]
    bends = [b for _,items in grouped for b in items]
    radius = min((b["radius"] for b in bends), default=None)
    straight = [sum(engineered_segment_length(s) for s in group[a["last_segment"]+1:b["first_segment"]]
                    if s["type"] == "line") for group,items in grouped for a,b in zip(items, items[1:])]
    row("金属最小弯曲半径", "直线段" if route and not bends else radius,
        recommended=value("metal_bend_radius_recommended", spec["recommended_bend_radius"]),
        minimum=value("metal_bend_radius_minimum", spec["minimum_bend_radius"]), cell=f"E{rowno}",
        recommended_key="metal_bend_radius_recommended", minimum_key="metal_bend_radius_minimum")
    if spec["minimum_bend_radius"] != spec["recommended_bend_radius"]:
        row("规划执行半径下限（含 1.5D）", value("metal_bend_radius_minimum", spec["minimum_bend_radius"]), unit="mm")
    row("最短弯间直线段", min(straight, default=None), value("straight_recommended", spec["straight_recommended"]),
        value("straight_minimum", spec["straight_minimum"]), f"F{rowno}",
        recommended_key="straight_recommended", minimum_key="straight_minimum")
    row("齿形最小内径", minimum=value("teeth_min_inner_diameter", TEETH[pipe.pipe_spec]), cell=f"G{min(rowno,4)}",
        minimum_key="teeth_min_inner_diameter")
    for key, name, col, unit, edit_key in (("material", "胶管型号", "H", "", "hose_material"),
        ("outer_diameter", "胶管外径", "I", "mm", "hose_outer_diameter"),
        ("wall_thickness", "胶管壁厚", "J", "mm", "hose_wall_thickness")):
        row(name, value(edit_key, cover[key]), cell=f"{col}{min(rowno,4)}", unit=unit,
            actual_key=None if route else edit_key, input_type="text" if not unit else "number")
    hose = next((c for c in (route or {}).get("hose_layout", {}).get("components", []) if c["kind"] == "hose"), None)
    hose_radius = _hose_radius_report(hose["start_s"], hose["end_s"], arc_radius_intervals(segments), cover["minimum_bend_radius"]) if hose else None
    actual = hose_radius["minimum_actual_radius_mm"] if hose_radius else None
    row("胶管最小弯曲半径", "直线段" if hose and actual is None else actual,
        recommended=value("hose_bend_radius_recommended", cover["recommended_bend_radius"]),
        minimum=value("hose_bend_radius_minimum", cover["minimum_bend_radius"]), cell=f"K{min(rowno,4)}",
        recommended_key="hose_bend_radius_recommended", minimum_key="hose_bend_radius_minimum")
    row("最小弯管角度", min((b["angle_deg"] for b in bends), default=None),
        value("bend_angle_recommended", 10.), value("bend_angle_minimum", 5.), "L2", "°",
        recommended_key="bend_angle_recommended", minimum_key="bend_angle_minimum")
    row("相邻焊点距离", recommended=value("weld_spacing_recommended", 30.), minimum=value("weld_spacing_minimum", 25.),
        cell="M2", recommended_key="weld_spacing_recommended", minimum_key="weld_spacing_minimum")
    row("阀座到弯管距离", recommended=value("valve_straight_recommended", 25.), minimum=value("valve_straight_minimum", 5.),
        cell="O2", recommended_key="valve_straight_recommended", minimum_key="valve_straight_minimum")
    row("齿形插入胶管长度", recommended=value("teeth_insert_length_recommended", 35.), minimum=value("teeth_insert_length_minimum", 35.),
        cell="Q2", recommended_key="teeth_insert_length_recommended", minimum_key="teeth_insert_length_minimum")
    back = settings.ALUMINUM_SLEEVE_BACK_LENGTHS[pipe.pipe_spec if rowno == 5 else cover_id]
    row("铝套背部直线段", recommended=value("sleeve_back_recommended", back[0]), minimum=value("sleeve_back_minimum", back[1]),
        cell=f"R{rowno}", recommended_key="sleeve_back_recommended", minimum_key="sleeve_back_minimum")
    row("铝套轴线操作空间", (route or {}).get("constraint_report", {}).get("aluminum_sleeve_axis_clearance_actual"),
        recommended=value("sleeve_axis_clearance_recommended", 80.), minimum=value("sleeve_axis_clearance_minimum", 65.), cell="T2",
        recommended_key="sleeve_axis_clearance_recommended", minimum_key="sleeve_axis_clearance_minimum")
    row("马鞍型号 / 实际尺寸", cell="C8:J13", unit="")
    if route:
        row("总长度", RouteCurve(segments).length)
        row("弯曲数量", len(bends), unit="处")
        row("起点直线段", route.get("geometry_report", {}).get("start_straight_length_mm"))
        row("终点直线段", route.get("geometry_report", {}).get("goal_straight_length_mm"))
    if hose:
        row("胶管长度", hose["end_s"]-hose["start_s"], minimum=value("hose_length_minimum", hose_entry(route, pipe).get("min_length_mm", 30)),
            minimum_key="hose_length_minimum")
        row("胶管中心距起点（沿线）", (hose["start_s"]+hose["end_s"])/2)
        row("每端铝套长度", hose_entry(route, pipe)["sleeve_length_mm"])
    row("实体外缘净距要求", value("required_clearance", pipe.required_clearance), actual_key="required_clearance")
    row("固定挂点", "、".join(g.name for g in pipe.required_hangers) or "无", unit="")
    return rows


def route_view(route, pipe):
    curve = RouteCurve(route["segments"])
    entry = hose_entry(route, pipe)
    components = []
    for component in route.get("component_segments", []):
        local = RouteCurve(component["segments"])
        components.append({"kind": component["kind"], "color": component["color"],
                           "radius": component["outer_radius"], "path": local.sample()[0].tolist(),
                           "start_s": component["start_s"], "end_s": component["end_s"]})
    return {"pipe_id": pipe.pipe_id, "color": "#c7b79d", "radius": pipe.planning_radius,
            **curve.display(), "fixedIntervals": fixed_intervals(route, pipe),
            "components": components, "hose": entry, "warnings": warnings_for(route),
            "parameters": parameter_rows(pipe, route), "exportable": route.get("exportable", False)}
