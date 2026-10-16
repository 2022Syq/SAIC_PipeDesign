"""Configuration models for a multi-pipe routing run.

The single-pipe pipeline keeps its values in :mod:`settings`.  Multi-pipe
runs instead carry endpoint data in a JSON file so that reading one pipe does
not overwrite the coordinates of another.  This module owns the small amount
of validation needed at that boundary and resolves an outside radius from the
existing ``settings.PIPE_SPECS`` table.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any, Mapping, Sequence

import settings


Vector3 = tuple[float, float, float]


@dataclass(frozen=True)
class GuardianPoint:
    """One named hard route point measured in the common CATIA frame."""

    name: str
    point: Vector3

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip():
            raise ValueError("guardian name must be a non-empty string")
        object.__setattr__(self, "name", self.name.strip())
        object.__setattr__(self, "point", _vector(self.point, f"guardians[{self.name}].point"))


def _vector(value: Any, field_name: str) -> Vector3:
    """Validate and normalize one three-dimensional point."""
    if isinstance(value, (str, bytes, Mapping)):
        raise ValueError(f"{field_name} must be a sequence of three finite numbers")
    try:
        size = len(value)
    except TypeError as exc:
        raise ValueError(f"{field_name} must be a sequence of three finite numbers") from exc
    if size != 3:
        raise ValueError(f"{field_name} must contain exactly three values")
    result: list[float] = []
    for index, component in enumerate(value):
        if isinstance(component, bool):
            raise ValueError(f"{field_name}[{index}] must be a finite number")
        try:
            number = float(component)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field_name}[{index}] must be a finite number") from exc
        if not math.isfinite(number):
            raise ValueError(f"{field_name}[{index}] must be a finite number")
        result.append(number)
    return tuple(result)  # type: ignore[return-value]


def _number(value: Any, field_name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{field_name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{field_name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{field_name} must be a finite number")
    if minimum is not None and number < minimum:
        raise ValueError(f"{field_name} must be >= {minimum}")
    return number


def _priority(value: Any) -> int:
    if isinstance(value, bool):
        raise ValueError("priority must be an integer")
    if isinstance(value, float) and not value.is_integer():
        raise ValueError("priority must be an integer")
    try:
        result = int(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("priority must be an integer") from exc
    if isinstance(value, str) and str(result) != value.strip():
        raise ValueError("priority must be an integer")
    return result


def _guardians(value: Any, field_name: str = "guardians") -> tuple[GuardianPoint, ...]:
    """Normalize named hard route points from JSON."""
    if value is None:
        return ()
    if isinstance(value, Mapping):
        entries = [{"name": name, "point": point} for name, point in value.items()]
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes)):
        entries = list(value)
    else:
        raise ValueError(f"{field_name} must be an array or object")

    result: list[GuardianPoint] = []
    names: set[str] = set()
    for index, entry in enumerate(entries):
        if isinstance(entry, GuardianPoint):
            if entry.name in names:
                raise ValueError(f"duplicate guardian name: {entry.name}")
            names.add(entry.name)
            result.append(entry)
            continue
        if not isinstance(entry, Mapping):
            raise ValueError(f"{field_name}[{index}] must contain name and point")
        name = entry.get("name")
        point = entry.get("point")
        if not isinstance(name, str) or not name.strip():
            raise ValueError(f"{field_name}[{index}].name must be a non-empty string")
        name = name.strip()
        if name in names:
            raise ValueError(f"duplicate guardian name: {name}")
        names.add(name)
        if point is None:
            raise ValueError(f"{field_name}[{index}].point is required; merge measured CATIA points first")
        result.append(GuardianPoint(name, _vector(point, f"{field_name}[{index}].point")))
    return tuple(result)


def spec_outer_radius(spec_id: str, specs: Mapping[str, Mapping[str, Any]] | None = None) -> float:
    """Return the outside radius (mm) for a row in ``settings.PIPE_SPECS``."""
    table = settings.PIPE_SPECS if specs is None else specs
    if not isinstance(spec_id, str) or not spec_id.strip():
        raise ValueError("pipe_spec must be a non-empty string")
    if spec_id not in table:
        raise ValueError(f"Unknown pipe spec: {spec_id}")
    try:
        diameter = _number(table[spec_id]["outer_diameter"], f"PIPE_SPECS[{spec_id}].outer_diameter", minimum=0.0)
    except KeyError as exc:
        raise ValueError(f"PIPE_SPECS[{spec_id}] has no outer_diameter") from exc
    if diameter <= 0.0:
        raise ValueError(f"PIPE_SPECS[{spec_id}].outer_diameter must be > 0")
    return diameter / 2.0


# Names used by callers that prefer the word ``radius`` to ``outer``.
pipe_spec_radius = spec_outer_radius
spec_radius = spec_outer_radius


@dataclass(frozen=True)
class PipeInstance:
    """One configured route and its endpoint tangent reference points."""

    pipe_id: str
    start: Vector3
    goal: Vector3
    start_dir: Vector3
    goal_dir: Vector3
    pipe_spec: str
    required_clearance: float = 0.0
    priority: int = 0
    required_hangers: tuple[GuardianPoint, ...] = ()
    allow_hard_conflicts: bool = False
    allow_inter_pipe_conflicts: bool = False
    allow_direct_guide_modeling: bool = False

    def __post_init__(self) -> None:
        if not isinstance(self.pipe_id, str) or not self.pipe_id.strip():
            raise ValueError("pipe_id must be a non-empty string")
        object.__setattr__(self, "pipe_id", self.pipe_id.strip())
        for name in ("start", "goal", "start_dir", "goal_dir"):
            object.__setattr__(self, name, _vector(getattr(self, name), name))
        if not isinstance(self.pipe_spec, str) or not self.pipe_spec.strip():
            raise ValueError("pipe_spec must be a non-empty string")
        object.__setattr__(self, "pipe_spec", self.pipe_spec.strip())
        # Resolve here so a typo is reported while loading the run, not during
        # a later clearance check.
        spec_outer_radius(self.pipe_spec)
        object.__setattr__(
            self,
            "required_clearance",
            _number(self.required_clearance, "required_clearance", minimum=0.0),
        )
        object.__setattr__(self, "priority", _priority(self.priority))
        object.__setattr__(self, "required_hangers", _guardians(self.required_hangers, "required_hangers"))
        if not isinstance(self.allow_hard_conflicts, bool):
            raise ValueError("allow_hard_conflicts must be a boolean")
        if not isinstance(self.allow_inter_pipe_conflicts, bool):
            raise ValueError("allow_inter_pipe_conflicts must be a boolean")
        if not isinstance(self.allow_direct_guide_modeling, bool):
            raise ValueError("allow_direct_guide_modeling must be a boolean")
        if self.start == self.start_dir:
            raise ValueError("start_dir must differ from start")
        if self.goal == self.goal_dir:
            raise ValueError("goal_dir must differ from goal")

    @property
    def outer_radius(self) -> float:
        return spec_outer_radius(self.pipe_spec)

    @property
    def radius(self) -> float:
        return self.outer_radius

    @property
    def spec_id(self) -> str:
        return self.pipe_spec

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "PipeInstance":
        if not isinstance(value, Mapping):
            raise ValueError("each pipe entry must be a JSON object")
        required = ("pipe_id", "start", "goal", "start_dir", "goal_dir", "pipe_spec")
        missing = [name for name in required if name not in value]
        if missing:
            raise ValueError(f"pipe entry is missing required fields: {', '.join(missing)}")
        return cls(
            pipe_id=value["pipe_id"],
            start=value["start"],
            goal=value["goal"],
            start_dir=value["start_dir"],
            goal_dir=value["goal_dir"],
            pipe_spec=value["pipe_spec"],
            required_clearance=value.get("required_clearance", 0.0),
            priority=value.get("priority", 0),
            required_hangers=value.get("required_hangers", ()),
            allow_hard_conflicts=value.get("allow_hard_conflicts", False),
            allow_inter_pipe_conflicts=value.get("allow_inter_pipe_conflicts", False),
            allow_direct_guide_modeling=value.get("allow_direct_guide_modeling", False),
        )


@dataclass(frozen=True)
class RunConfig:
    """Validated configuration for one multi-pipe run."""

    pipes: tuple[PipeInstance, ...]
    metadata: Mapping[str, Any] = field(default_factory=dict)

    def __post_init__(self) -> None:
        pipes = tuple(self.pipes)
        if not pipes:
            raise ValueError("run config must contain at least one pipe")
        if any(not isinstance(pipe, PipeInstance) for pipe in pipes):
            raise ValueError("run config pipes must be PipeInstance objects")
        ids = [pipe.pipe_id for pipe in pipes]
        if len(set(ids)) != len(ids):
            duplicates = sorted({pipe_id for pipe_id in ids if ids.count(pipe_id) > 1})
            raise ValueError(f"pipe_id values must be unique: {duplicates}")
        object.__setattr__(self, "pipes", pipes)
        if not isinstance(self.metadata, Mapping):
            raise ValueError("metadata must be a JSON object")

    def ordered_pipes(self) -> tuple[PipeInstance, ...]:
        """Return pipes in deterministic priority, then input order."""
        ordered = sorted(enumerate(self.pipes), key=lambda item: (item[1].priority, item[0]))
        return tuple(pipe for _, pipe in ordered)

    @property
    def pipe_instances(self) -> tuple[PipeInstance, ...]:
        return self.pipes

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "RunConfig":
        if not isinstance(value, Mapping):
            raise ValueError("run config must be a JSON object")
        raw_pipes = value.get("pipes")
        if raw_pipes is None:
            raise ValueError("run config must contain a 'pipes' array")
        if isinstance(raw_pipes, (str, bytes)) or not isinstance(raw_pipes, Sequence):
            raise ValueError("run config 'pipes' must be an array")
        pipes = tuple(PipeInstance.from_dict(item) for item in raw_pipes)
        metadata = {str(k): v for k, v in value.items() if k != "pipes"}
        return cls(pipes=pipes, metadata=metadata)


def _read_json_payload(path: str | Path, description: str) -> Any:
    config_path = Path(path)
    try:
        return json.loads(config_path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        raise
    except (OSError, UnicodeError) as exc:
        raise ValueError(f"unable to read {description} {config_path}: {exc}") from exc
    except json.JSONDecodeError as exc:
        raise ValueError(f"invalid JSON in {description} {config_path}: {exc.msg}") from exc


def _measured_by_id(payload: Any) -> dict[str, Mapping[str, Any]]:
    measured_items = payload.get("pipes") if isinstance(payload, Mapping) else None
    if (
        not isinstance(payload, Mapping)
        or isinstance(measured_items, (str, bytes))
        or not isinstance(measured_items, Sequence)
    ):
        raise ValueError("measured points must contain a 'pipes' array")
    measured_by_id: dict[str, Mapping[str, Any]] = {}
    for item in measured_items:
        if not isinstance(item, Mapping) or not item.get("pipe_id"):
            raise ValueError("each measured point entry must contain pipe_id")
        pipe_id = str(item["pipe_id"])
        if pipe_id in measured_by_id:
            raise ValueError(f"duplicate measured pipe_id: {pipe_id}")
        measured_by_id[pipe_id] = item
    return measured_by_id


def _measured_guardians(payload: Any) -> dict[str, Vector3]:
    raw = payload.get("guardians", []) if isinstance(payload, Mapping) else []
    if raw is None:
        return {}
    if isinstance(raw, Mapping):
        entries = [{"name": name, "point": point} for name, point in raw.items()]
    elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
        entries = list(raw)
    else:
        raise ValueError("measured guardians must be an array or object")
    result: dict[str, Vector3] = {}
    for index, entry in enumerate(entries):
        if not isinstance(entry, Mapping) or not isinstance(entry.get("name"), str):
            raise ValueError(f"measured guardians[{index}] must contain name and point")
        name = entry["name"].strip()
        if name in result:
            raise ValueError(f"duplicate measured guardian name: {name}")
        result[name] = _vector(entry.get("point"), f"measured guardians[{index}].point")
    return result


def _resolve_required_hangers(raw: Any, measured: Mapping[str, Vector3]) -> list[dict[str, Any]]:
    if raw is None:
        return []
    if isinstance(raw, Mapping):
        entries = [{"name": name, "point": point} for name, point in raw.items()]
    elif isinstance(raw, Sequence) and not isinstance(raw, (str, bytes)):
        entries = list(raw)
    else:
        raise ValueError("required_hangers must be an array or object")
    resolved = []
    for index, entry in enumerate(entries):
        if isinstance(entry, str):
            entry = {"name": entry}
        if not isinstance(entry, Mapping) or not isinstance(entry.get("name"), str):
            raise ValueError(f"required_hangers[{index}] must contain a name")
        name = entry["name"].strip()
        point = entry.get("point", measured.get(name))
        if point is None:
            raise ValueError(f"no measured point found for required guardian: {name}")
        resolved.append({"name": name, "point": _vector(point, f"required_hangers[{index}].point")})
    return resolved


def _merge_measured_into_raw_config(config_payload: Any, measured_payload: Any) -> Mapping[str, Any]:
    if not isinstance(config_payload, Mapping):
        raise ValueError("run config must be a JSON object")
    raw_pipes = config_payload.get("pipes")
    if isinstance(raw_pipes, (str, bytes)) or not isinstance(raw_pipes, Sequence):
        raise ValueError("run config 'pipes' must be an array")
    measured_by_id = _measured_by_id(measured_payload)
    measured_guardians = _measured_guardians(measured_payload)
    merged_pipes = []
    for item in raw_pipes:
        if not isinstance(item, Mapping) or not item.get("pipe_id"):
            raise ValueError("each run config pipe entry must contain pipe_id")
        pipe_id = str(item["pipe_id"])
        measured = measured_by_id.get(pipe_id)
        if measured is None:
            raise ValueError(f"measured points do not contain run config pipe_id: {pipe_id}")
        merged = dict(item)
        for name in ("start", "goal", "start_dir", "goal_dir"):
            if name in measured:
                merged[name] = measured[name]
        if "required_hangers" in merged:
            merged["required_hangers"] = _resolve_required_hangers(
                merged["required_hangers"], measured_guardians
            )
        merged_pipes.append(merged)

    config_ids = {str(item["pipe_id"]) for item in raw_pipes if isinstance(item, Mapping) and item.get("pipe_id")}
    unknown = sorted(set(measured_by_id).difference(config_ids))
    if unknown:
        raise ValueError(f"measured points do not match run config: unknown measured pipe_id={unknown}")
    merged_payload = dict(config_payload)
    merged_payload["pipes"] = merged_pipes
    return merged_payload


def load_run_config(path: str | Path, measured: Mapping[str, Any] | str | Path | None = None) -> RunConfig:
    """Read, optionally merge measured coordinates, and validate a run config."""
    payload = _read_json_payload(path, "run config")
    if measured is not None:
        measured_payload = (
            _read_json_payload(measured, "measured points")
            if isinstance(measured, (str, Path))
            else measured
        )
        payload = _merge_measured_into_raw_config(payload, measured_payload)
    return RunConfig.from_dict(payload)


def merge_measured_points(config: RunConfig, measured: Mapping[str, Any] | str | Path) -> RunConfig:
    """Merge ``main_point.py --multi-output`` coordinates into a run config.

    The measured file supplies only endpoint and direction coordinates. Pipe
    specifications, priorities, and clearances remain owned by the run config.
    """
    if not isinstance(config, RunConfig):
        raise TypeError("config must be a RunConfig")
    payload = (
        _read_json_payload(measured, "measured points")
        if isinstance(measured, (str, Path))
        else measured
    )
    measured_by_id = _measured_by_id(payload)
    measured_guardians = _measured_guardians(payload)

    config_ids = {pipe.pipe_id for pipe in config.pipes}
    unknown = sorted(set(measured_by_id).difference(config_ids))
    missing = sorted(config_ids.difference(measured_by_id))
    if unknown or missing:
        details = []
        if unknown:
            details.append(f"unknown measured pipe_id={unknown}")
        if missing:
            details.append(f"missing measured pipe_id={missing}")
        raise ValueError("measured points do not match run config: " + "; ".join(details))

    required = ("start", "goal", "start_dir", "goal_dir")
    merged = []
    for pipe in config.pipes:
        item = measured_by_id[pipe.pipe_id]
        values = {name: item.get(name) for name in required}
        missing_fields = [name for name, value in values.items() if value is None]
        if missing_fields:
            raise ValueError(f"measured points for {pipe.pipe_id} missing: {', '.join(missing_fields)}")
        guardians = pipe.required_hangers
        if guardians:
            resolved = _resolve_required_hangers(
                [{"name": guardian.name, "point": guardian.point} for guardian in guardians], measured_guardians
            )
            guardians = tuple(GuardianPoint(entry["name"], entry["point"]) for entry in resolved)
        merged.append(replace(pipe, required_hangers=guardians, **values))
    return RunConfig(pipes=tuple(merged), metadata=dict(config.metadata))


read_run_config = load_run_config
from_json = load_run_config


__all__ = [
    "GuardianPoint",
    "PipeInstance",
    "RunConfig",
    "load_run_config",
    "merge_measured_points",
    "read_run_config",
    "from_json",
    "spec_outer_radius",
    "pipe_spec_radius",
    "spec_radius",
]
