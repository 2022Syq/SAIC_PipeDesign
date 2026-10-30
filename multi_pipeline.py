# -*- coding: utf-8 -*-
"""Minimal orchestration for running several pipe configurations.

The existing ``main1``-``main4`` scripts use global ``settings.py`` values and
fixed output names.  This module therefore owns only the run bookkeeping and
stage ordering.  Callers that have parameterized stage implementations inject
them into :func:`run_pipeline`; the default adapters import the existing
modules lazily and fail with an explicit explanation of the missing adapter.

Stage callable contract
-----------------------
Each stage receives ``(pipe, context)`` and returns a JSON-serializable value
(or a value accepted by ``json.dump(..., default=...)``). ``context`` is a
mutable :class:`StageContext`; its ``results`` mapping contains the outputs of
earlier stages for that pipe.  A stage may write additional files below its
``pipe_dir`` and return their paths in its result.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import re
import time
import uuid
from dataclasses import asdict, dataclass, is_dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Protocol, TYPE_CHECKING

if TYPE_CHECKING:  # pragma: no cover - import is intentionally lazy at runtime.
    from pipe_config import RunConfig


class StageCallable(Protocol):
    """Callable shape accepted by the three injectable stages."""

    def __call__(self, pipe: Any, context: "StageContext") -> Any: ...


@dataclass
class StageContext:
    """State shared by stages for one pipe in one isolated run directory."""

    config: Any
    pipe: Any
    run_id: str
    run_dir: Path
    pipe_dir: Path
    results: dict[str, Any]


class StageAdapterRequired(RuntimeError):
    """Raised when an existing script has no per-pipe callable interface."""


def _utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


def _safe_component(value: str) -> str:
    """Return a stable directory component without path separators."""

    component = re.sub(r"[^A-Za-z0-9_.-]+", "_", str(value)).strip("._")
    return component or "pipe"


def make_run_id() -> str:
    """Create a sortable, collision-resistant default run identifier."""

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    return f"{stamp}-{uuid.uuid4().hex[:8]}"


def _json_default(value: Any) -> Any:
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return asdict(value)
    if hasattr(value, "tolist") and callable(value.tolist):
        return value.tolist()
    if hasattr(value, "__dict__"):
        return vars(value)
    return repr(value)


def _write_json(path: Path, payload: Any) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2, default=_json_default) + "\n",
        encoding="utf-8",
    )


def _config_pipes(config: Any) -> list[Any]:
    """Read ``RunConfig`` in its small public form and preserve priority order."""

    ordered = getattr(config, "ordered_pipes", None)
    if callable(ordered):
        return list(ordered())
    pipes = getattr(config, "pipes", None)
    if pipes is None:
        raise TypeError("RunConfig must provide pipes or ordered_pipes()")
    indexed = list(enumerate(pipes))
    indexed.sort(key=lambda item: (getattr(item[1], "priority", 0), item[0]))
    return [item[1] for item in indexed]


def _pipe_id(pipe: Any, index: int) -> str:
    return _safe_component(getattr(pipe, "pipe_id", None) or getattr(pipe, "id", None) or f"pipe-{index + 1:02d}")


def _default_stage(stage: str, modules: tuple[str, ...]) -> StageCallable:
    """Build a lazy adapter that reports the current non-parameterized boundary."""

    def adapter(pipe: Any, context: StageContext) -> Any:
        imported = []
        for module_name in modules:
            try:
                importlib.import_module(module_name)
            except ImportError as exc:
                raise StageAdapterRequired(
                    f"{stage} stage requires an injected per-pipe callable; "
                    f"lazy import of existing module {module_name!r} failed ({exc}). "
                    f"Pass {stage}=callable(pipe, context) to run_pipeline()."
                ) from exc
            imported.append(module_name)
        module_list = ", ".join(imported)
        raise StageAdapterRequired(
            f"{stage} stage requires an injected per-pipe callable; "
            f"existing module(s) {module_list} expose global/CLI workflow only. "
            f"Pass {stage}=callable(pipe, context) to run_pipeline()."
        )

    return adapter


def default_planning(pipe: Any, context: StageContext) -> Any:
    """Default planning adapter; see :class:`StageAdapterRequired`."""

    return _default_stage("planning", ("main1", "main2"))(pipe, context)


def default_engineering(pipe: Any, context: StageContext) -> Any:
    """Default engineering adapter; see :class:`StageAdapterRequired`."""

    return _default_stage("engineering", ("main3",))(pipe, context)


def default_export(pipe: Any, context: StageContext) -> Any:
    """Default CATIA export adapter; see :class:`StageAdapterRequired`."""

    return _default_stage("export", ("main4",))(pipe, context)


def _config_snapshot(config: Any) -> Any:
    if is_dataclass(config):
        return asdict(config)
    if isinstance(config, Mapping):
        return dict(config)
    return config


def run_pipeline(
    config: "RunConfig",
    *,
    planning: StageCallable | None = None,
    engineering: StageCallable | None = None,
    export: StageCallable | None = None,
    output_root: str | Path = "outputs",
    run_id: str | None = None,
) -> dict[str, Any]:
    """Run all configured pipes in priority order and write run manifests.

    ``planning``, ``engineering`` and ``export`` are intentionally injectable;
    this keeps tests independent of CATIA and lets parameterized implementations
    choose their own input/output files.  A stage failure is recorded in the
    manifest before being re-raised, so the partial run remains inspectable.
    """

    pipes = _config_pipes(config)
    resolved_run_id = _safe_component(run_id or make_run_id())
    run_dir = Path(output_root).resolve() / resolved_run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    stage_functions: dict[str, StageCallable] = {
        "planning": planning or default_planning,
        "engineering": engineering or default_engineering,
        "export": export or default_export,
    }
    started_at = _utc_now()
    manifest: dict[str, Any] = {
        "run_id": resolved_run_id,
        "started_at": started_at,
        "status": "running",
        "pipe_count": len(pipes),
        "config": _config_snapshot(config),
        "pipes": [],
    }
    aggregate: dict[str, Any] = {
        "run_id": resolved_run_id,
        "status": "running",
        "pipes": [],
    }
    _write_json(run_dir / "manifest.json", manifest)

    try:
        for index, pipe in enumerate(pipes):
            pipe_id = _pipe_id(pipe, index)
            pipe_dir = run_dir / pipe_id
            pipe_dir.mkdir(parents=True, exist_ok=False)
            context = StageContext(config, pipe, resolved_run_id, run_dir, pipe_dir, {})
            pipe_record: dict[str, Any] = {
                "pipe_id": pipe_id,
                "priority": getattr(pipe, "priority", None),
                "status": "running",
                "directory": str(pipe_dir),
                "stages": {},
            }
            manifest["pipes"].append(pipe_record)
            aggregate["pipes"].append(pipe_record)
            for stage_name in ("planning", "engineering", "export"):
                result = stage_functions[stage_name](pipe, context)
                context.results[stage_name] = result
                result_path = pipe_dir / f"{stage_name}.json"
                _write_json(result_path, result)
                pipe_record["stages"][stage_name] = {
                    "status": "completed",
                    "result": result,
                    "artifact": str(result_path),
                }
                _write_json(run_dir / "manifest.json", manifest)
            pipe_record["status"] = "completed"
            _write_json(run_dir / "manifest.json", manifest)
        manifest["status"] = aggregate["status"] = "completed"
    except Exception as exc:
        manifest["status"] = aggregate["status"] = "failed"
        manifest["error"] = {"type": type(exc).__name__, "message": str(exc)}
        aggregate["error"] = manifest["error"]
        _write_json(run_dir / "manifest.json", manifest)
        _write_json(run_dir / "aggregate.json", aggregate)
        raise

    manifest["completed_at"] = aggregate["completed_at"] = _utc_now()
    _write_json(run_dir / "manifest.json", manifest)
    _write_json(run_dir / "aggregate.json", aggregate)
    return {"run_id": resolved_run_id, "run_dir": run_dir, "manifest": manifest, "aggregate": aggregate}


def run_from_config(
    config_or_path: "RunConfig | str | Path",
    **kwargs: Any,
) -> dict[str, Any]:
    """Load a ``RunConfig`` from JSON when given a path, then run it."""

    if isinstance(config_or_path, (str, Path)):
        from pipe_config import load_run_config

        config = load_run_config(config_or_path)
    else:
        config = config_or_path
    return run_pipeline(config, **kwargs)


__all__ = [
    "StageAdapterRequired",
    "StageContext",
    "StageCallable",
    "default_engineering",
    "default_export",
    "default_planning",
    "make_run_id",
    "planning_space_cache_key",
    "run_from_config",
    "run_pipeline",
    "run_multi_pipe",
    "cli_main",
]


def _pipe_value(pipe, name, default=None):
    if isinstance(pipe, Mapping):
        return pipe.get(name, default)
    return getattr(pipe, name, default)


def _planning_radius(pipe) -> float:
    """Return the continuous metal-core radius for centerline planning."""
    value = _pipe_value(pipe, "planning_radius", None)
    if value is None:
        value = _pipe_value(pipe, "outer_radius", _pipe_value(pipe, "radius", 0.0))
    return float(value)


def _shared_space_bounds(pipes: list[Any]) -> tuple[list[float], list[float], float]:
    points = [
        list(_pipe_value(pipe, name))
        for pipe in pipes
        for name in ("start", "goal")
    ]
    if not points:
        raise ValueError("at least one pipe is required to build a shared space")
    start = [min(point[index] for point in points) for index in range(3)]
    goal = [max(point[index] for point in points) for index in range(3)]
    radius = max(_planning_radius(pipe) for pipe in pipes)
    return start, goal, radius


def planning_space_cache_key(model_dir: str | Path, pipes: list[Any]) -> str:
    """Return a key for the CAD and geometric inputs that define a space."""
    import settings

    model_dir = Path(model_dir).resolve()
    suffixes = {".igs", ".iges", ".step", ".stp", ".stl"}
    files = []
    if model_dir.is_dir():
        for path in sorted(model_dir.iterdir(), key=lambda item: item.name.lower()):
            if path.is_file() and path.suffix.lower() in suffixes:
                stat = path.stat()
                files.append({
                    "name": path.name,
                    "size": int(stat.st_size),
                    "mtime_ns": int(stat.st_mtime_ns),
                })
    start, goal, radius = _shared_space_bounds(pipes)
    geometry_settings = {
        name: getattr(settings, name)
        for name in (
            "SECTION_WIDTH_Y", "SECTION_CENTER_Y", "SECTION_HEIGHT_ABOVE_GROUND",
            "SECTION_DX", "SECTION_DY", "X_PADDING_BEFORE_START", "X_PADDING_AFTER_GOAL",
            "MESH_LINEAR_DEFLECTION", "MESH_ANGULAR_DEFLECTION", "MIN_FREE_REGION_AREA",
            "POLYGON_SIMPLIFY_TOLERANCE", "SOLID_LOOP_CLOSE_TOLERANCE", "SOLID_MIN_FILL_AREA",
            "SOLID_FILL_FROM_BUFFER", "DEFAULT_OBSTACLE_CLEARANCE", "DEFAULT_GROUND_CLEARANCE",
            "DEFAULT_COMPONENT_SOLID", "SPECIAL_COMPONENT_RULES",
        )
        if hasattr(settings, name)
    }
    payload = {
        "model_dir": str(model_dir),
        "files": files,
        "space_start": start,
        "space_goal": goal,
        "planning_radius": radius,
        "geometry_settings": geometry_settings,
    }
    encoded = json.dumps(payload, ensure_ascii=False, sort_keys=True, default=str).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def _point_entries(pipe: Any) -> list[tuple[str, list[float]]]:
    """Return endpoints and named hard points used to detect impossible joins."""
    entries = []
    pipe_id = str(_pipe_value(pipe, "pipe_id", "pipe"))
    for field in ("start", "goal"):
        point = _pipe_value(pipe, field)
        if point is not None:
            entries.append((f"{pipe_id}.{field}", [float(v) for v in point]))
    guardians = _pipe_value(pipe, "required_hangers", ()) or ()
    for index, guardian in enumerate(guardians, start=1):
        point = guardian.get("point") if isinstance(guardian, Mapping) else getattr(guardian, "point", None)
        name = guardian.get("name") if isinstance(guardian, Mapping) else getattr(guardian, "name", None)
        if point is not None:
            entries.append((f"{pipe_id}.{name or f'guardian_{index}'}", [float(v) for v in point]))
    return entries


def _point_conflict_reports(pipes: list[Any]) -> list[dict[str, Any]]:
    """Report input points whose solid envelopes already overlap.

    Such a report is an input-measurement warning and a formal-route failure;
    it tells the user to inspect the two CATIA endpoint or hanger points.
    """
    reports = []
    for left_index, left_pipe in enumerate(pipes):
        left_radius = _planning_radius(left_pipe)
        for right_pipe in pipes[left_index + 1:]:
            right_radius = _planning_radius(right_pipe)
            for left_name, left_point in _point_entries(left_pipe):
                for right_name, right_point in _point_entries(right_pipe):
                    distance = float(math.dist(left_point, right_point))
                    threshold = left_radius + right_radius
                    if distance + 1e-9 >= threshold:
                        continue
                    reports.append({
                        "type": "point_conflict",
                        "pipe_id_a": str(_pipe_value(left_pipe, "pipe_id", "pipe")),
                        "pipe_id_b": str(_pipe_value(right_pipe, "pipe_id", "pipe")),
                        "point_a": left_name,
                        "point_b": right_name,
                        "point_a_xyz": left_point,
                        "point_b_xyz": right_point,
                        "center_distance": distance,
                        "required_center_distance": threshold,
                        "message": (
                            "连接点中心距离小于两管外半径之和；请检查两管的起点、终点或挂点测量值"
                        ),
                    })
    return reports


def _channel_width_report(model: Any, path: Any, pipe: Any) -> dict[str, Any]:
    """Estimate the narrowest local free-space width along a centerline.

    The section model already owns the free polygons used by planning. For a
    sampled center point, twice the distance to its containing polygon boundary
    is the local diameter available to the pipe envelope. This gives a useful
    diagnostic width without changing the planner's acceptance rule.
    """
    import numpy as np
    from shapely.geometry import Point

    points = np.asarray(path or [], dtype=float)
    radius = _planning_radius(pipe)
    clearance = float(_pipe_value(pipe, "required_clearance", 0.0) or 0.0)
    required = 2.0 * radius + clearance
    available_values = []
    for point in points:
        widths = []
        for values, sections, axis_index, planar_index in (
            (model.x_values, model.x_sections, 0, (1, 2)),
            (model.y_values, model.y_sections, 1, (0, 2)),
        ):
            section = sections[model._nearest_index(values, point[axis_index])]
            planar = Point(float(point[planar_index[0]]), float(point[planar_index[1]]))
            for polygon in section.get("polys", []):
                if polygon.contains(planar) or polygon.touches(planar):
                    widths.append(2.0 * float(planar.distance(polygon.boundary)))
        available_values.append(min(widths) if widths else 0.0)
    available = min(available_values) if available_values else 0.0
    return {
        "available_width_mm": float(available),
        "required_width_mm": float(required),
        "gap_mm": float(available - required),
        "pipe_outer_diameter_mm": float(2.0 * radius),
        "required_clearance_mm": float(clearance),
        "sample_count": int(len(points)),
        "valid": bool(available + 1e-6 >= required),
    }


def _temporary_run_settings(run_dir: Path, space_start, space_goal, radius):
    import contextlib
    import settings

    names = {
        "OUTPUT_DIR": run_dir,
        "SPACE_JSON": run_dir / "planning_space.json",
        "SPACE_CHECKPOINT_JSON": run_dir / "planning_space_checkpoint.json",
        "SPACE_HTML": run_dir / "planning_space.html",
        # 规划空间包含上亿个数值；多管流程默认只保留 JSON，避免 VS Code
        # 解析超大内嵌 HTML 时耗尽内存。
        "WRITE_SPACE_HTML": False,
        "BIDIR_RESULT_JSON": run_dir / "routes_guide.json",
        "BIDIR_HTML": run_dir / "routes_guide.html",
        "BIDIR_DISTANCE_PNG": run_dir / "routes_guide_distance.png",
        "ENGINEERING_RESULT_JSON": run_dir / "routes_engineered.json",
        "ENGINEERING_HTML": run_dir / "multi_route_engineered.html",
        "START": list(space_start),
        "GOAL": list(space_goal),
        "PIPE_RADIUS": float(radius),
        "ROUTE_MAX_ENVELOPE_RADIUS": float(radius),
        "X_RANGE": None,
    }
    previous = {name: getattr(settings, name, None) for name in names}

    @contextlib.contextmanager
    def manager():
        try:
            for name, value in names.items():
                setattr(settings, name, value)
            yield settings
        finally:
            for name, value in previous.items():
                setattr(settings, name, value)

    return manager()


def _write_pipe_result(pipe_dir: Path, stage: str, result: Any) -> Path:
    path = pipe_dir / f"{stage}.json"
    _write_json(path, result)
    return path


def _clearance_against_reserved(pipe, route, reserved, sample_step: float = 20.0) -> list[dict[str, Any]]:
    from route_clearance import check_pipe_clearance

    reports = []
    for other_pipe, other_route in reserved:
        reports.append(check_pipe_clearance(pipe, route, other_pipe, other_route, sample_step))
    return reports


def _double_clamp_reports(config, pipes_by_id, routes_by_id) -> list[dict[str, Any]]:
    """Evaluate configured two-pipe clamps when both routes are available."""
    clamps = getattr(config, "double_clamps", ()) or ()
    if not clamps:
        return []
    from route_clearance import double_clamp_report, local_radius_at_point

    reports = []
    for clamp in clamps:
        route_a = routes_by_id.get(clamp.pipe_a)
        route_b = routes_by_id.get(clamp.pipe_b)
        pipe_a = pipes_by_id.get(clamp.pipe_a)
        pipe_b = pipes_by_id.get(clamp.pipe_b)
        if route_a is None or route_b is None or pipe_a is None or pipe_b is None:
            continue
        guardian_a = next(
            (item for item in (_pipe_value(pipe_a, "required_hangers", ()) or ())
             if (item.get("name") if isinstance(item, Mapping) else getattr(item, "name", None)) == clamp.guardian_a),
            None,
        )
        guardian_b = next(
            (item for item in (_pipe_value(pipe_b, "required_hangers", ()) or ())
             if (item.get("name") if isinstance(item, Mapping) else getattr(item, "name", None)) == clamp.guardian_b),
            None,
        )
        if guardian_a is None or guardian_b is None:
            raise ValueError(f"double clamp {clamp.name} guardian references are not resolved")
        point_a = guardian_a.get("point") if isinstance(guardian_a, Mapping) else guardian_a.point
        point_b = guardian_b.get("point") if isinstance(guardian_b, Mapping) else guardian_b.point
        report = double_clamp_report(
            route_a,
            point_a,
            local_radius_at_point(route_a, point_a, _planning_radius(pipe_a)),
            route_b,
            point_b,
            local_radius_at_point(route_b, point_b, _planning_radius(pipe_b)),
            tangent_tolerance_deg=float(clamp.tangent_tolerance_deg),
            required_clearance=max(
                float(_pipe_value(pipe_a, "required_clearance", 0.0) or 0.0),
                float(_pipe_value(pipe_b, "required_clearance", 0.0) or 0.0),
            ),
        )
        report.update({
            "name": clamp.name,
            "pipe_id_a": clamp.pipe_a,
            "guardian_a": clamp.guardian_a,
            "pipe_id_b": clamp.pipe_b,
            "guardian_b": clamp.guardian_b,
        })
        reports.append(report)
    return reports


def _invalid_clearance_reports(reports: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Keep failed pipe-to-pipe checks for explicit green-light diagnostics."""
    return [report for report in reports if not bool(report.get("valid", False))]


def _route_has_hard_conflict(route: Any) -> bool:
    """Return whether a route hit an original hard obstacle.

    ``DualSectionModel(allow_hard=True)`` deliberately changes the effective
    planner count to zero, so acceptance must inspect the original conflict
    evidence carried by the route/collision report as well.
    """
    if not isinstance(route, Mapping):
        return False
    if route.get("hard_conflicts") or route.get("hard_conflict_components"):
        return True
    if bool(route.get("hard_conflicts_allowed", False)):
        return True
    for key in (
        "hard_bad", "hard_bad_segments", "hard_bad_samples",
        "hard_bad_original", "hard_bad_original_segments", "hard_bad_original_samples",
    ):
        try:
            if int(route.get(key, 0) or 0) > 0:
                return True
        except (TypeError, ValueError):
            continue
    for report_key in ("collision_report", "constraint_report"):
        report = route.get(report_key)
        if not isinstance(report, Mapping):
            continue
        if report.get("hard_conflicts") or report.get("hard_conflict_components"):
            return True
        for key in (
            "hard_bad", "hard_bad_segments", "hard_bad_samples",
            "hard_bad_original", "hard_bad_original_segments", "hard_bad_original_samples",
        ):
            try:
                if int(report.get(key, 0) or 0) > 0:
                    return True
            except (TypeError, ValueError):
                continue
    return False


def _engineering_is_formally_valid(route: Any) -> bool:
    """Apply the multi-pipe export gate to an engineered route."""
    if not isinstance(route, Mapping):
        return False
    if (
        bool(route.get("diagnostic_only", False))
        or route.get("formal_valid") is False
        or route.get("engineering_fallback")
    ):
        return False
    for path_key in ("engineered_path", "guide_path"):
        path = route.get(path_key)
        if not isinstance(path, (list, tuple)) or len(path) < 2:
            return False
    for key in ("geometry_report", "constraint_report"):
        report = route.get(key)
        if isinstance(report, Mapping) and report.get("valid") is False:
            return False
    collision = route.get("collision_report")
    if isinstance(collision, Mapping):
        for key in (
            "hard_bad", "hard_bad_segments", "hard_bad_samples",
            "hard_bad_original", "hard_bad_original_segments", "hard_bad_original_samples",
            "inter_pipe_bad", "inter_pipe_bad_segments", "inter_pipe_bad_samples",
        ):
            try:
                if int(collision.get(key, 0) or 0) > 0:
                    return False
            except (TypeError, ValueError):
                continue
    return not _route_has_hard_conflict(route)


def _direct_guide_engineering(candidate: dict[str, Any], pipe: Any, reason: str) -> dict[str, Any]:
    """Represent a guide route as a display-only diagnostic polyline.

    This path intentionally carries invalid engineering reports and is never
    eligible for ``reserved`` or CATIA export.
    """
    from main3 import path_length, tube_mesh
    import numpy as np
    import settings

    path = [list(point) for point in candidate.get("path", [])]
    if len(path) < 2:
        start = _pipe_value(pipe, "start")
        goal = _pipe_value(pipe, "goal")
        if start is not None and goal is not None:
            path = [list(start), list(goal)]
    if len(path) < 2:
        raise RuntimeError("direct guide modeling requires at least two guide points")
    segments = [
        {
            "type": "line",
            "start": path[index],
            "end": path[index + 1],
            "length": float(math.dist(path[index], path[index + 1])),
        }
        for index in range(len(path) - 1)
    ]
    array_path = np.asarray(path, dtype=float)
    result = dict(candidate)
    result.update({
        "color": candidate.get("color", "#2563eb"),
        "engineering_fallback": "direct_guide_modeling",
        "diagnostic_only": True,
        "formal_valid": False,
        "exportable": False,
        "engineering_fallback_reason": str(reason),
        "engineered_path": path,
        "guide_path": path,
        "z_smoothed_guide_path": path,
        "control_path_before_s_bend": path,
        "control_path": path,
        "segments": segments,
        "tube_mesh": tube_mesh(
            array_path,
            _planning_radius(pipe),
            int(getattr(settings, "ENGINEERING_TUBE_SEGMENTS", 16)),
        ),
        "guide_length": float(path_length(array_path)),
        "engineered_length": float(path_length(array_path)),
        "arc_count": 0,
        "turn_count": max(0, len(path) - 2),
        "main_turn_count": max(0, len(path) - 2),
        "radius_reduced_count": 0,
        "geometry_report": {
            "valid": False,
            "mode": "direct_guide_modeling",
            "reason": str(reason),
        },
        "constraint_report": {
            "valid": False,
            "mode": "direct_guide_modeling",
            "reason": str(reason),
        },
    })
    return result


def _diagnostic_candidate(pipe: Any, pipe_id: str, reason: str) -> dict[str, Any]:
    """Create an endpoint-only route when planning produced no candidate."""
    start = _pipe_value(pipe, "start")
    goal = _pipe_value(pipe, "goal")
    if start is None or goal is None:
        path = []
    else:
        path = [list(start), list(goal)]
    if len(path) < 2:
        # Valid RunConfig objects always have endpoints. Keep the HTML
        # contract intact even for a malformed injected object by displaying a
        # zero-length placeholder and the original reason in diagnostics.
        path = [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    half = max(1, len(path) // 2)
    return {
        "name": f"{pipe_id}_diagnostic",
        "description": "诊断路线：严格规划未找到可验收路线",
        "pipe_id": pipe_id,
        "start": list(start) if start is not None else None,
        "goal": list(goal) if goal is not None else None,
        "path": path,
        "front_path": path[: half + 1],
        "rear_path": path[half:],
        "meeting_point": path[half],
        "front_branch_x": None,
        "rear_branch_x": None,
        "length": float(sum(math.dist(a, b) for a, b in zip(path[:-1], path[1:]))),
        "valid": False,
        "engineering_valid": False,
        "hard_bad_segments": 0,
        "soft_bad_segments": 0,
        "hard_conflicts": [],
        "hard_conflict_components": [],
        "inter_pipe_conflicts": [],
        "inter_pipe_conflicts_allowed": False,
        "allow_hard_conflicts": bool(_pipe_value(pipe, "allow_hard_conflicts", False)),
        "hard_conflicts_allowed": False,
        "allow_inter_pipe_conflicts": False,
        "hanger_distance": {"count": 0, "satisfied_count": 0, "max_distance": 0.0, "range": 0.0},
        "hanger_valid": False,
        "logs": [],
        "cost": float("inf"),
        "reason": str(reason),
        "diagnostic_only": True,
    }


def _accepts_keyword(callable_obj: Any, name: str) -> bool:
    """Check an optional staged API without swallowing call-time failures."""
    import inspect

    try:
        return name in inspect.signature(callable_obj).parameters
    except (TypeError, ValueError):
        return False


def run_multi_pipe(
    config_or_path: "RunConfig | str | Path",
    *,
    points_path: str | Path | None = None,
    output_root: str | Path = "outputs",
    run_id: str | None = None,
    centerline_only: bool = False,
    catia_visible: bool = False,
    skip_catia: bool = False,
    reuse_space_json: str | Path | None = None,
    write_html: bool = True,
) -> dict[str, Any]:
    """Run the real sequential multi-pipe flow using the existing stages."""
    if isinstance(config_or_path, (str, Path)):
        from pipe_config import load_run_config

        config = load_run_config(config_or_path, measured=points_path)
    else:
        config = config_or_path
    if points_path is not None and not isinstance(config_or_path, (str, Path)):
        from pipe_config import merge_measured_points

        config = merge_measured_points(config, points_path)
    pipes = _config_pipes(config)
    pipes_by_id = {_pipe_id(pipe, index): pipe for index, pipe in enumerate(pipes)}
    from main1 import build_shared_space
    from main2 import DualSectionModel, export_html as export_guide_html, plan_pipe_routes, section_polygons
    from main3 import engineer_pipe_route, export_engineering_html
    from main4 import create_catia_part
    import settings

    space_start, space_goal, shared_radius = _shared_space_bounds(pipes)
    space_cache_key = planning_space_cache_key(settings.BASE_STEP_DIR, pipes)
    resolved_run_id = _safe_component(run_id or make_run_id())
    run_dir = Path(output_root).resolve() / resolved_run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    point_conflicts = _point_conflict_reports(pipes)
    manifest = {
        "run_id": resolved_run_id,
        "status": "running",
        "pipe_count": len(pipes),
        "planning_order": [_pipe_id(pipe, index) for index, pipe in enumerate(pipes)],
        "points_source": str(Path(points_path).resolve()) if points_path is not None else None,
        "double_clamps": [asdict(clamp) for clamp in getattr(config, 'double_clamps', ())],
        "shared_space": {
            "start_bound": space_start,
            "goal_bound": space_goal,
            "max_outer_radius": shared_radius,
        },
        "space_cache_key": space_cache_key,
        "point_conflicts": point_conflicts,
        "pipes": [],
    }
    _write_json(run_dir / "manifest.json", manifest)
    reserved: list[tuple[Any, dict[str, Any]]] = []
    guide_routes: list[dict[str, Any]] = []
    engineered_routes: list[dict[str, Any]] = []
    clearance_reports: list[dict[str, Any]] = []

    try:
        with _temporary_run_settings(run_dir, space_start, space_goal, shared_radius):
            space_started = time.perf_counter()
            reused_space_path = None
            if reuse_space_json is None:
                space = build_shared_space(space_start, space_goal, shared_radius, run_dir)
                _write_json(run_dir / "planning_space.cache.json", {
                    "cache_key": space_cache_key,
                    "space_json": str(run_dir / "planning_space.json"),
                })
            else:
                space_path = Path(reuse_space_json).resolve()
                if not space_path.is_file():
                    raise FileNotFoundError(f"reused planning space JSON not found: {space_path}")
                cache_path = space_path.with_name("planning_space.cache.json")
                if not cache_path.is_file():
                    raise ValueError(
                        "reused planning space has no cache manifest; rebuild it once before reuse"
                    )
                cache_record = json.loads(cache_path.read_text(encoding="utf-8"))
                if cache_record.get("cache_key") != space_cache_key:
                    raise ValueError(
                        "reused planning space does not match the current CAD model, endpoints, or space settings"
                    )
                print(f"[SPACE] reusing completed planning space: {space_path}", flush=True)
                space = json.loads(space_path.read_text(encoding="utf-8"))
                space["source_json"] = str(space_path)
                reused_space_path = space_path
            if reused_space_path is not None:
                # Engineering distance profiles read settings.SPACE_JSON;
                # point it at the completed source when no copy is created.
                settings.SPACE_JSON = reused_space_path

            planning_extra = float(getattr(settings, "MAIN2_EXTRA_CLEARANCE", 0.0))
            section_cache = (
                section_polygons(space, "x", extra_clearance=planning_extra),
                section_polygons(space, "y", extra_clearance=planning_extra),
                section_polygons(space, "x"),
                section_polygons(space, "y"),
            )
            space_elapsed = time.perf_counter() - space_started
            manifest["timing"] = {
                "shared_space_seconds": float(space_elapsed),
                "shared_space_reused": bool(reused_space_path is not None),
            }
            print(
                f"[TIMING] shared_space_seconds={space_elapsed:.1f} "
                f"reused={bool(reused_space_path is not None)}",
                flush=True,
            )

            validation_extra = float(getattr(settings, "ENGINEERING_VALIDATION_EXTRA_CLEARANCE", 2.0))
            if abs(validation_extra) <= 1e-12:
                collision_sections = section_cache[2:]
            else:
                collision_sections = (
                    section_polygons(space, "x", extra_clearance=validation_extra),
                    section_polygons(space, "y", extra_clearance=validation_extra),
                )
            for index, pipe in enumerate(pipes):
                pipe_id = _pipe_id(pipe, index)
                pipe_dir = run_dir / pipe_id
                pipe_dir.mkdir(parents=True, exist_ok=False)
                allow_hard_conflicts = bool(_pipe_value(pipe, "allow_hard_conflicts", False))
                allow_inter_pipe_conflicts = bool(
                    _pipe_value(pipe, "allow_inter_pipe_conflicts", False)
                )
                allow_direct_guide_modeling = bool(
                    _pipe_value(pipe, "allow_direct_guide_modeling", False)
                )
                # Always search the strict model first.  An allow-listed hard
                # conflict is an exploration diagnostic, not a shortcut that
                # can hide a strict solution.
                collision_kwargs = {
                    "allow_hard": False,
                    "pipe_radius": _planning_radius(pipe),
                    "required_clearance": float(_pipe_value(pipe, "required_clearance", 0.0) or 0.0),
                }
                if _accepts_keyword(DualSectionModel, "reserved_routes"):
                    collision_kwargs["reserved_routes"] = reserved
                collision_model = DualSectionModel(
                    collision_sections[0], collision_sections[1], **collision_kwargs
                )
                record = {
                    "pipe_id": pipe_id,
                    "priority": _pipe_value(pipe, "priority"),
                    "status": "running",
                    "allow_hard_conflicts": allow_hard_conflicts,
                    "allow_inter_pipe_conflicts": allow_inter_pipe_conflicts,
                    "allow_direct_guide_modeling": allow_direct_guide_modeling,
                    "input_point_conflicts": [
                        item for item in point_conflicts
                        if item["pipe_id_a"] == pipe_id or item["pipe_id_b"] == pipe_id
                    ],
                }
                manifest["pipes"].append(record)

                # Publish the current pipe before a potentially long route search.
                _write_json(run_dir / "manifest.json", manifest)

                pipe_started = time.perf_counter()
                planning_started = pipe_started
                planning_error = None
                try:
                    planning_kwargs = {}
                    if _accepts_keyword(plan_pipe_routes, "reserved_routes"):
                        planning_kwargs["reserved_routes"] = reserved
                    if _accepts_keyword(plan_pipe_routes, "allow_hard_conflicts"):
                        planning_kwargs["allow_hard_conflicts"] = False
                    if _accepts_keyword(plan_pipe_routes, "section_cache"):
                        planning_kwargs["section_cache"] = section_cache
                    planning_result = plan_pipe_routes(space, pipe, **planning_kwargs)
                except Exception as exc:
                    planning_error = f"planning failed: {exc}"
                    planning_result = {"routes": [], "error": planning_error}
                planning_elapsed = time.perf_counter() - planning_started
                _write_pipe_result(pipe_dir, "planning", planning_result)
                selected_guide = None
                selected_engineered = None
                selected_reports = None
                selected_formal = False
                failed_candidates = []
                raw_candidates = planning_result.get("routes", []) if isinstance(planning_result, Mapping) else []
                candidates = list(raw_candidates or [])
                diagnostic_candidate = candidates[0] if candidates else _diagnostic_candidate(
                    pipe, pipe_id, planning_error or "no planning candidate route"
                )
                diagnostic_engineered = None
                diagnostic_reports: list[dict[str, Any]] = []

                for candidate in candidates:
                    guide_reports = _clearance_against_reserved(pipe, candidate, reserved)
                    guide_conflicts = _invalid_clearance_reports(guide_reports)
                    candidate_channel = _channel_width_report(
                        collision_model, candidate.get("path", []), pipe
                    )
                    candidate["channel_report"] = candidate_channel
                    try:
                        engineered = engineer_pipe_route(
                            candidate,
                            pipe,
                            color=candidate.get("color", "#2563eb"),
                            collision_model=collision_model,
                        )
                    except Exception as exc:
                        failed_candidates.append({
                            "name": candidate.get("name"),
                            "stage": "engineering",
                            "reason": str(exc),
                        })
                        try:
                            engineered = _direct_guide_engineering(candidate, pipe, str(exc))
                        except Exception as diagnostic_exc:
                            engineered = None
                            failed_candidates[-1]["diagnostic_reason"] = str(diagnostic_exc)
                    if engineered is None:
                        continue
                    final_reports = _clearance_against_reserved(pipe, engineered, reserved)
                    final_conflicts = _invalid_clearance_reports(final_reports)
                    engineered["channel_report"] = _channel_width_report(
                        collision_model,
                        engineered.get("engineered_path", engineered.get("path", [])),
                        pipe,
                    )
                    engineered["inter_pipe_conflicts"] = final_conflicts
                    engineered["inter_pipe_conflicts_allowed"] = False
                    route_candidates = {
                        _pipe_id(other_pipe, 0): other_route
                        for other_pipe, other_route in reserved
                    }
                    route_candidates[pipe_id] = engineered
                    double_reports = _double_clamp_reports(
                        config, pipes_by_id, route_candidates
                    )
                    double_failures = [
                        item for item in double_reports if not item.get("valid", False)
                    ]
                    engineered["double_clamp_reports"] = double_reports
                    candidate["double_clamp_reports"] = double_reports
                    candidate["inter_pipe_conflicts"] = guide_conflicts
                    candidate["inter_pipe_conflicts_allowed"] = False
                    if diagnostic_engineered is None:
                        diagnostic_candidate = candidate
                        diagnostic_engineered = engineered
                        diagnostic_reports = guide_reports + final_reports

                    hard_conflict = _route_has_hard_conflict(candidate) or _route_has_hard_conflict(engineered)
                    input_point_conflicts = record["input_point_conflicts"]
                    candidate["point_conflicts"] = input_point_conflicts
                    engineered["point_conflicts"] = input_point_conflicts
                    formal = (
                        not guide_conflicts
                        and not final_conflicts
                        and not input_point_conflicts
                        and not hard_conflict
                        and not double_failures
                        and _engineering_is_formally_valid(engineered)
                    )
                    if not formal:
                        failed_candidates.append({
                            "name": candidate.get("name"),
                            "stage": "acceptance",
                            "hard_conflict": bool(hard_conflict),
                            "point_conflicts": input_point_conflicts,
                            "channel_report": engineered.get("channel_report", candidate_channel),
                            "guide_clearance": guide_reports,
                            "engineered_clearance": final_reports,
                            "geometry_valid": engineered.get("geometry_report", {}).get("valid")
                            if isinstance(engineered.get("geometry_report"), Mapping) else None,
                            "constraint_valid": engineered.get("constraint_report", {}).get("valid")
                            if isinstance(engineered.get("constraint_report"), Mapping) else None,
                            "double_clamp_reports": double_reports,
                        })
                        continue
                    selected_guide = candidate
                    selected_engineered = engineered
                    selected_reports = guide_reports + final_reports
                    selected_formal = True
                    selected_engineered["diagnostic_only"] = False
                    selected_engineered["formal_valid"] = True
                    selected_engineered["exportable"] = True
                    break

                if not selected_formal:
                    # Only after strict planning produced no candidate may an
                    # explicitly authorized hard-conflict search provide the
                    # display-only route required by the HTML report.
                    if not candidates and allow_hard_conflicts:
                        diagnostic_model = DualSectionModel(
                            collision_sections[0],
                            collision_sections[1],
                            allow_hard=True,
                            reserved_routes=reserved,
                            pipe_radius=_planning_radius(pipe),
                            required_clearance=float(_pipe_value(pipe, "required_clearance", 0.0) or 0.0),
                        )
                        diagnostic_kwargs = {}
                        if _accepts_keyword(plan_pipe_routes, "reserved_routes"):
                            diagnostic_kwargs["reserved_routes"] = reserved
                        if _accepts_keyword(plan_pipe_routes, "allow_hard_conflicts"):
                            diagnostic_kwargs["allow_hard_conflicts"] = True
                        if _accepts_keyword(plan_pipe_routes, "section_cache"):
                            diagnostic_kwargs["section_cache"] = section_cache
                        try:
                            diagnostic_planning = plan_pipe_routes(
                                space, pipe, **diagnostic_kwargs
                            )
                        except Exception as exc:
                            diagnostic_planning = {"routes": [], "error": str(exc)}
                        diagnostic_candidates = list(
                            diagnostic_planning.get("routes", [])
                            if isinstance(diagnostic_planning, Mapping)
                            else []
                        )
                        if diagnostic_candidates:
                            diagnostic_candidate = diagnostic_candidates[0]
                            diagnostic_candidate["channel_report"] = _channel_width_report(
                                diagnostic_model, diagnostic_candidate.get("path", []), pipe
                            )
                            diagnostic_guide_reports = _clearance_against_reserved(
                                pipe, diagnostic_candidate, reserved
                            )
                            diagnostic_candidate["inter_pipe_conflicts"] = _invalid_clearance_reports(
                                diagnostic_guide_reports
                            )
                            try:
                                diagnostic_engineered = engineer_pipe_route(
                                    diagnostic_candidate,
                                    pipe,
                                    color=diagnostic_candidate.get("color", "#2563eb"),
                                    collision_model=diagnostic_model,
                                )
                            except Exception as exc:
                                diagnostic_engineered = _direct_guide_engineering(
                                    diagnostic_candidate, pipe, str(exc)
                                )
                            diagnostic_engineered["diagnostic_only"] = True
                            diagnostic_engineered["formal_valid"] = False
                            diagnostic_engineered["exportable"] = False
                            diagnostic_final_reports = _clearance_against_reserved(
                                pipe, diagnostic_engineered, reserved
                            )
                            diagnostic_engineered["inter_pipe_conflicts"] = _invalid_clearance_reports(
                                diagnostic_final_reports
                            )
                            diagnostic_engineered["inter_pipe_conflicts_allowed"] = False
                            diagnostic_reports = diagnostic_guide_reports + diagnostic_final_reports
                            diagnostic_engineered["channel_report"] = _channel_width_report(
                                diagnostic_model,
                                diagnostic_engineered.get("engineered_path", diagnostic_engineered.get("path", [])),
                                pipe,
                            )

                    if diagnostic_engineered is None:
                        diagnostic_candidate = diagnostic_candidate or _diagnostic_candidate(
                            pipe, pipe_id, planning_error or "no candidate route could be engineered"
                        )
                        try:
                            diagnostic_engineered = _direct_guide_engineering(
                                diagnostic_candidate,
                                pipe,
                                planning_error or "no candidate route could be engineered",
                            )
                        except Exception as exc:
                            diagnostic_engineered = _diagnostic_candidate(
                                pipe,
                                pipe_id,
                                f"{planning_error or 'no candidate route could be engineered'}; {exc}",
                            )
                            diagnostic_engineered.update({
                                "geometry_report": {"valid": False, "reason": str(exc)},
                                "constraint_report": {"valid": False, "reason": str(exc)},
                                "engineering_fallback": "diagnostic_placeholder",
                                "engineered_path": diagnostic_engineered["path"],
                                "guide_path": diagnostic_engineered["path"],
                                "z_smoothed_guide_path": diagnostic_engineered["path"],
                                "control_path_before_s_bend": diagnostic_engineered["path"],
                                "control_path": diagnostic_engineered["path"],
                                "segments": [],
                                "tube_mesh": {"x": [], "y": [], "z": [], "i": [], "j": [], "k": []},
                                "guide_length": diagnostic_engineered["length"],
                                "engineered_length": diagnostic_engineered["length"],
                                "arc_count": 0,
                                "turn_count": 0,
                                "main_turn_count": 0,
                                "max_turn_count": 0,
                                "z_simplify_points": len(diagnostic_engineered["path"]),
                                "s_bend_removed_points": 0,
                                "wiggle_removed_points": 0,
                                "z_total_variation_before": 0.0,
                                "z_total_variation_after": 0.0,
                            })
                    selected_guide = diagnostic_candidate
                    selected_engineered = diagnostic_engineered
                    selected_reports = diagnostic_reports
                    selected_engineered["diagnostic_only"] = True
                    selected_engineered["formal_valid"] = False
                    selected_engineered["exportable"] = False
                    selected_guide["point_conflicts"] = record["input_point_conflicts"]
                    selected_engineered["point_conflicts"] = record["input_point_conflicts"]
                    if "channel_report" not in selected_guide:
                        selected_guide["channel_report"] = _channel_width_report(
                            collision_model, selected_guide.get("path", []), pipe
                        )
                    if "channel_report" not in selected_engineered:
                        selected_engineered["channel_report"] = selected_guide["channel_report"]

                _write_pipe_result(pipe_dir, "engineering", selected_engineered)
                _write_pipe_result(pipe_dir, "clearance", selected_reports or [])
                conflict_count = len(selected_engineered.get("inter_pipe_conflicts", []))
                blocking_reasons = []
                if record["input_point_conflicts"]:
                    blocking_reasons.append("point_conflict")
                if selected_engineered.get("hard_conflicts") or _route_has_hard_conflict(selected_engineered):
                    blocking_reasons.append("hard_obstacle_conflict")
                if selected_engineered.get("inter_pipe_conflicts"):
                    blocking_reasons.append("inter_pipe_conflict")
                channel_report = selected_engineered.get("channel_report", {})
                if isinstance(channel_report, Mapping) and float(channel_report.get("gap_mm", 0.0)) < -1e-6:
                    blocking_reasons.append("channel_too_narrow")
                if any(
                    not item.get("valid", False)
                    for item in selected_engineered.get("double_clamp_reports", [])
                ):
                    blocking_reasons.append("double_clamp_constraint")
                if not selected_formal and not blocking_reasons:
                    blocking_reasons.append("engineering_or_constraint_failure")
                record.update({
                    "status": "completed" if selected_formal else "diagnostic_failed",
                    "formal_valid": bool(selected_formal),
                    "directory": str(pipe_dir),
                    "failed_candidates": failed_candidates,
                    "inter_pipe_conflicts_allowed": False,
                    "inter_pipe_conflict_count": conflict_count,
                    "channel_report": channel_report,
                    "blocking_reasons": blocking_reasons,
                    "reorder_recommendation": (
                        "当前固定顺序已穷尽本管路搜索；请示后再考虑调整管路顺序"
                        if not selected_formal else None
                    ),
                    "timing_seconds": {
                        "planning": float(planning_elapsed),
                        "total": float(time.perf_counter() - pipe_started),
                    },
                })
                print(
                    f"[TIMING] {pipe_id} planning_seconds={planning_elapsed:.1f} "
                    f"total_seconds={record['timing_seconds']['total']:.1f}",
                    flush=True,
                )
                guide_routes.append(selected_guide)
                engineered_routes.append(selected_engineered)
                clearance_reports.extend(selected_reports or [])
                if selected_formal:
                    reserved.append((pipe, selected_engineered))
                _write_json(run_dir / "manifest.json", manifest)

            # A clamp is a shared constraint: neither member is formally accepted
            # if the other member failed or their final pair fails the clamp check.
            double_reports = _double_clamp_reports(
                config, pipes_by_id,
                {route['pipe_id']: route for route in engineered_routes},
            )
            for report in double_reports:
                pair_ids = {report['pipe_id_a'], report['pipe_id_b']}
                pair = [route for route in engineered_routes if route['pipe_id'] in pair_ids]
                report['members_formal_valid'] = all(route.get('formal_valid', False) for route in pair)
                report['valid'] = report['valid'] and report['members_formal_valid']
                for route in pair:
                    route['double_clamp_reports'] = [
                        item for item in double_reports
                        if route['pipe_id'] in {item['pipe_id_a'], item['pipe_id_b']}
                    ]
                    if not report['valid']:
                        route.update(formal_valid=False, exportable=False, diagnostic_only=True)
                        record = next(item for item in manifest['pipes'] if item['pipe_id'] == route['pipe_id'])
                        record.update(status='diagnostic_failed', formal_valid=False)
                        if 'double_clamp_constraint' not in record['blocking_reasons']:
                            record['blocking_reasons'].append('double_clamp_constraint')
                    _write_pipe_result(run_dir / route['pipe_id'], 'engineering', route)
            manifest['double_clamp_reports'] = double_reports
            reserved = [(pipe, route) for pipe, route in reserved if route.get('formal_valid')]
            guide_result = {"run_id": resolved_run_id, "routes": guide_routes}
            engineered_result = {"run_id": resolved_run_id, "routes": engineered_routes}
            _write_json(run_dir / "routes_guide.json", guide_result)
            _write_json(run_dir / "routes_engineered.json", engineered_result)
            _write_json(run_dir / "validation.json", {"reports": clearance_reports,
                                                       "double_clamp_reports": double_reports})
            # The workbench renders the final engineered data itself. Avoid
            # loading the full CAD model twice to create unused static viewers.
            if write_html:
                export_guide_html(guide_routes, run_dir / "routes_guide.html")
                export_engineering_html(engineered_result, run_dir / "multi_route_engineered.html")

            blocked = [
                record for record in manifest["pipes"]
                if record.get("status") != "completed"
            ]
            manifest["blocked_pipes"] = [record.get("pipe_id") for record in blocked]
            manifest["reorder_recommendation"] = {
                "required": bool(blocked),
                "pipes": [
                    {
                        "pipe_id": record.get("pipe_id"),
                        "blocking_reasons": record.get("blocking_reasons", []),
                        "message": record.get("reorder_recommendation"),
                    }
                    for record in blocked
                ],
            }

            if not skip_catia:
                catia_dir = run_dir / "catia"
                catia_dir.mkdir(parents=True, exist_ok=True)
                for pipe, route in reserved:
                    output = catia_dir / f"{_pipe_id(pipe, 0)}.CATPart"
                    create_catia_part(
                        route,
                        output,
                        _planning_radius(pipe),
                        bool(catia_visible),
                        bool(centerline_only),
                    )
        manifest["status"] = (
            "completed"
            if all(record.get("status") == "completed" for record in manifest["pipes"])
            else "diagnostic_failed"
        )
    except Exception as exc:
        manifest["status"] = "failed"
        manifest["error"] = {"type": type(exc).__name__, "message": str(exc)}
        _write_json(run_dir / "manifest.json", manifest)
        raise

    _write_json(run_dir / "manifest.json", manifest)
    return {"run_id": resolved_run_id, "run_dir": run_dir, "manifest": manifest, "routes": engineered_routes}


def cli_main(argv=None) -> int:
    import argparse

    parser = argparse.ArgumentParser(description="Run sequential multi-pipe planning and CATIA export.")
    parser.add_argument("config", type=Path, help="JSON file containing the pipe instances.")
    parser.add_argument(
        "--points",
        dest="points_path",
        type=Path,
        help="JSON written by main_point.py --multi-output; replaces endpoint coordinates in the config.",
    )
    parser.add_argument("--output-root", type=Path, default=Path("outputs"))
    parser.add_argument("--run-id")
    parser.add_argument("--centerline-only", action="store_true")
    parser.add_argument("--catia-visible", action="store_true")
    parser.add_argument("--skip-catia", action="store_true")
    parser.add_argument(
        "--reuse-space-json",
        type=Path,
        help="Reuse an existing completed planning_space.json and skip rebuilding the shared space.",
    )
    args = parser.parse_args(argv)
    result = run_multi_pipe(
        args.config,
        points_path=args.points_path,
        output_root=args.output_root,
        run_id=args.run_id,
        centerline_only=args.centerline_only,
        catia_visible=args.catia_visible,
        skip_catia=args.skip_catia,
        reuse_space_json=args.reuse_space_json,
    )
    print(f"MULTI_PIPE_RUN: {result['run_dir']}")
    return 0


if __name__ == "__main__":  # pragma: no cover - CATIA/CAD integration entry.
    raise SystemExit(cli_main())
