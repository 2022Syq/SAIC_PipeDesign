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

import importlib
import json
import re
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
    "run_from_config",
    "run_pipeline",
    "run_multi_pipe",
    "cli_main",
]


def _pipe_value(pipe, name, default=None):
    if isinstance(pipe, Mapping):
        return pipe.get(name, default)
    return getattr(pipe, name, default)


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
    radius = max(float(_pipe_value(pipe, "outer_radius")) for pipe in pipes)
    return start, goal, radius


def _temporary_run_settings(run_dir: Path, space_start, space_goal, radius):
    import contextlib
    import settings

    names = {
        "OUTPUT_DIR": run_dir,
        "SPACE_JSON": run_dir / "planning_space.json",
        "SPACE_CHECKPOINT_JSON": run_dir / "planning_space_checkpoint.json",
        "SPACE_HTML": run_dir / "planning_space.html",
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

    path = route.get("engineered_path") or route.get("path") or route
    reports = []
    for other_pipe, other_route in reserved:
        reports.append(check_pipe_clearance(pipe, path, other_pipe, other_route, sample_step))
    return reports


def run_multi_pipe(
    config_or_path: "RunConfig | str | Path",
    *,
    points_path: str | Path | None = None,
    output_root: str | Path = "outputs",
    run_id: str | None = None,
    centerline_only: bool = False,
    catia_visible: bool = False,
    skip_catia: bool = False,
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
    from main1 import build_shared_space
    from main2 import DualSectionModel, export_html as export_guide_html, plan_pipe_routes, section_polygons
    from main3 import engineer_pipe_route, export_engineering_html
    from main4 import create_catia_part

    space_start, space_goal, shared_radius = _shared_space_bounds(pipes)
    resolved_run_id = _safe_component(run_id or make_run_id())
    run_dir = Path(output_root).resolve() / resolved_run_id
    run_dir.mkdir(parents=True, exist_ok=False)
    manifest = {
        "run_id": resolved_run_id,
        "status": "running",
        "pipe_count": len(pipes),
        "planning_order": [_pipe_id(pipe, index) for index, pipe in enumerate(pipes)],
        "points_source": str(Path(points_path).resolve()) if points_path is not None else None,
        "shared_space": {
            "start_bound": space_start,
            "goal_bound": space_goal,
            "max_outer_radius": shared_radius,
        },
        "pipes": [],
    }
    _write_json(run_dir / "manifest.json", manifest)
    reserved: list[tuple[Any, dict[str, Any]]] = []
    guide_routes: list[dict[str, Any]] = []
    engineered_routes: list[dict[str, Any]] = []
    clearance_reports: list[dict[str, Any]] = []

    try:
        with _temporary_run_settings(run_dir, space_start, space_goal, shared_radius):
            space = build_shared_space(space_start, space_goal, shared_radius, run_dir)
            import settings

            validation_extra = float(getattr(settings, "ENGINEERING_VALIDATION_EXTRA_CLEARANCE", 2.0))
            collision_model = DualSectionModel(
                section_polygons(space, "x", extra_clearance=validation_extra),
                section_polygons(space, "y", extra_clearance=validation_extra),
            )
            for index, pipe in enumerate(pipes):
                pipe_id = _pipe_id(pipe, index)
                pipe_dir = run_dir / pipe_id
                pipe_dir.mkdir(parents=True, exist_ok=False)
                record = {"pipe_id": pipe_id, "priority": _pipe_value(pipe, "priority"), "status": "running"}
                manifest["pipes"].append(record)

                planning_result = plan_pipe_routes(space, pipe)
                _write_pipe_result(pipe_dir, "planning", planning_result)
                selected_guide = None
                selected_engineered = None
                selected_reports = None
                failed_candidates = []
                for candidate in planning_result["routes"]:
                    guide_reports = _clearance_against_reserved(pipe, candidate, reserved)
                    if any(not report["valid"] for report in guide_reports):
                        failed_candidates.append({"name": candidate.get("name"), "stage": "planning", "clearance": guide_reports})
                        continue
                    try:
                        engineered = engineer_pipe_route(
                            candidate,
                            pipe,
                            color=candidate.get("color", "#2563eb"),
                            collision_model=collision_model,
                        )
                    except Exception as exc:
                        failed_candidates.append({"name": candidate.get("name"), "stage": "engineering", "reason": str(exc)})
                        continue
                    final_reports = _clearance_against_reserved(pipe, engineered, reserved)
                    if any(not report["valid"] for report in final_reports):
                        failed_candidates.append({"name": candidate.get("name"), "stage": "engineering", "clearance": final_reports})
                        continue
                    selected_guide = candidate
                    selected_engineered = engineered
                    selected_reports = guide_reports + final_reports
                    break
                if selected_guide is None or selected_engineered is None:
                    record.update({"status": "failed", "failed_candidates": failed_candidates})
                    _write_json(run_dir / "manifest.json", manifest)
                    raise RuntimeError(f"{pipe_id}: no candidate route satisfies inter-pipe clearance")

                _write_pipe_result(pipe_dir, "engineering", selected_engineered)
                _write_pipe_result(pipe_dir, "clearance", selected_reports or [])
                record.update({"status": "completed", "directory": str(pipe_dir), "failed_candidates": failed_candidates})
                reserved.append((pipe, selected_engineered))
                guide_routes.append(selected_guide)
                engineered_routes.append(selected_engineered)
                clearance_reports.extend(selected_reports or [])
                _write_json(run_dir / "manifest.json", manifest)

            guide_result = {"run_id": resolved_run_id, "routes": guide_routes}
            engineered_result = {"run_id": resolved_run_id, "routes": engineered_routes}
            _write_json(run_dir / "routes_guide.json", guide_result)
            _write_json(run_dir / "routes_engineered.json", engineered_result)
            _write_json(run_dir / "validation.json", {"reports": clearance_reports})
            export_guide_html(guide_routes, run_dir / "routes_guide.html")
            export_engineering_html(engineered_result, run_dir / "multi_route_engineered.html")

            if not skip_catia:
                catia_dir = run_dir / "catia"
                catia_dir.mkdir(parents=True, exist_ok=True)
                for pipe, route in reserved:
                    output = catia_dir / f"{_pipe_id(pipe, 0)}.CATPart"
                    create_catia_part(
                        route,
                        output,
                        float(_pipe_value(pipe, "outer_radius")),
                        bool(catia_visible),
                        bool(centerline_only),
                    )
        manifest["status"] = "completed"
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
    args = parser.parse_args(argv)
    result = run_multi_pipe(
        args.config,
        points_path=args.points_path,
        output_root=args.output_root,
        run_id=args.run_id,
        centerline_only=args.centerline_only,
        catia_visible=args.catia_visible,
        skip_catia=args.skip_catia,
    )
    print(f"MULTI_PIPE_RUN: {result['run_dir']}")
    return 0


if __name__ == "__main__":  # pragma: no cover - CATIA/CAD integration entry.
    raise SystemExit(cli_main())
