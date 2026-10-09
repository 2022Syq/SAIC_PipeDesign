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
]
