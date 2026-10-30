"""Local hose and aluminium sleeve rules for an engineered route.

The routing solver still plans one conservative centreline.  This module
turns an optional ``PIPE_SEGMENTS`` entry into material intervals along that
centreline.  It deliberately has no CATIA or Plotly dependency so the length,
position and bend-radius rules can be checked without those applications.
"""

from __future__ import annotations

import math
from typing import Any, Mapping, Sequence


COMPONENT_COLORS = {
    "aluminum": "#c7b79d",
    "hose": "#b24f45",
    "sleeve": "#c5a12a",
}

DEFAULT_HOSE_LENGTH_OPTIONS_MM = (30.0, 75.0)
DEFAULT_HOSE_LENGTH_MM = 30.0
DEFAULT_SLEEVE_LENGTH_MM = 30.0


def _number(value: Any, name: str, *, minimum: float | None = None) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{name} must be a finite number")
    if minimum is not None and number < minimum:
        raise ValueError(f"{name} must be >= {minimum:g} mm")
    return number


def _spec(specs: Mapping[str, Mapping[str, Any]], spec_id: str, kind: str) -> Mapping[str, Any]:
    if spec_id not in specs:
        raise ValueError(f"Unknown {kind} spec: {spec_id}")
    value = specs[spec_id]
    if value.get("kind") != kind:
        raise ValueError(f"{spec_id} must be a {kind} spec")
    return value


def _segment_length(segment: Mapping[str, Any]) -> float:
    if segment.get("type") == "line":
        a, b = segment["start"], segment["end"]
        return math.sqrt(sum((float(x) - float(y)) ** 2 for x, y in zip(a, b)))
    if segment.get("type") == "arc":
        return abs(math.radians(float(segment["angle_deg"]))) * float(segment["radius"])
    raise ValueError(f"unsupported engineered segment type: {segment.get('type')}")


def arc_radius_intervals(engineered_segments: Sequence[Mapping[str, Any]]) -> list[tuple[float, float, float]]:
    """Return ``(start_s, end_s, radius)`` for every physical arc."""
    result: list[tuple[float, float, float]] = []
    current = 0.0
    for segment in engineered_segments:
        length = _segment_length(segment)
        if segment.get("type") == "arc":
            result.append((current, current + length, float(segment["radius"])))
        current += length
    return result


def _hose_radius_report(
    start_s: float,
    end_s: float,
    arcs: Sequence[tuple[float, float, float]],
    minimum: float,
) -> dict[str, Any]:
    """Report the existing route curvature under one hose interval."""
    radii = [
        radius
        for arc_start, arc_end, radius in arcs
        if min(end_s, arc_end) - max(start_s, arc_start) > 1e-7
    ]
    actual = min(radii) if radii else None
    return {
        "valid": actual is None or actual + 1e-6 >= minimum,
        "actual_radii_mm": radii,
        "minimum_actual_radius_mm": actual,
        "minimum_mm": minimum,
    }


def build_hose_layout(
    path_length: float,
    pipe_segments: Sequence[Mapping[str, Any]] | None,
    specs: Mapping[str, Mapping[str, Any]],
    engineered_segments: Sequence[Mapping[str, Any]] = (),
    *,
    default_hose_length_options_mm: Sequence[float] = DEFAULT_HOSE_LENGTH_OPTIONS_MM,
    default_hose_length_mm: float = DEFAULT_HOSE_LENGTH_MM,
    default_sleeve_length_mm: float = DEFAULT_SLEEVE_LENGTH_MM,
    default_sleeve_radial_clearance_mm: float = 0.0,
    allow_constraint_warnings: bool = False,
) -> dict[str, Any]:
    """Resolve optional hose/sleeve intervals and their constraint report.

    ``position_s_mm`` is the centre of the hose itself.  When omitted, the
    centre of the complete ``sleeve + hose + sleeve`` span is used.  The hose
    follows the existing route samples; its bend radius is a warning only.
    """
    path_length = _number(path_length, "path_length", minimum=0.0)
    entries = list(pipe_segments or ())
    if not entries:
        return {
            "enabled": False,
            "valid": True,
            "components": [],
            "hard_failures": [],
            "warnings": [],
            "path_length_mm": path_length,
        }

    arcs = arc_radius_intervals(engineered_segments)
    components: list[dict[str, Any]] = []
    hard_failures: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    occupied: list[tuple[float, float]] = []
    selected_position_s: float | None = None
    selected_position_ratio: float | None = None
    selected_hose_start: float | None = None
    selected_hose_end: float | None = None

    for index, raw in enumerate(entries):
        if not isinstance(raw, Mapping):
            raise ValueError(f"PIPE_SEGMENTS[{index}] must be an object")
        core_spec = str(raw.get("core_spec", "metal_9"))
        _spec(specs, core_spec, "metal")
        hose_spec = str(raw.get("cover_spec", ""))
        hose = _spec(specs, hose_spec, "hose")
        if core_spec == "metal_9" and hose_spec != "hose_8DE3":
            raise ValueError("metal_9 must use hose_8DE3 (15 mm hose)")

        requested_min = raw.get("min_length_mm", raw.get("hose_length_requirement_mm", default_hose_length_mm))
        minimum_length = _number(requested_min, f"PIPE_SEGMENTS[{index}].min_length_mm", minimum=0.0)
        allowed_options = tuple(float(value) for value in raw.get("length_options_mm", default_hose_length_options_mm))
        if minimum_length not in allowed_options:
            raise ValueError(
                f"PIPE_SEGMENTS[{index}].min_length_mm must be one of {allowed_options} mm"
            )
        hose_length = _number(raw.get("hose_length_mm", minimum_length), f"PIPE_SEGMENTS[{index}].hose_length_mm",
                              minimum=1e-6 if allow_constraint_warnings else minimum_length)
        if hose_length < minimum_length:
            hard_failures.append({"rule": "hose_length", "actual_mm": hose_length, "minimum_mm": minimum_length})
        sleeve_length = _number(raw.get("sleeve_length_mm", default_sleeve_length_mm), f"PIPE_SEGMENTS[{index}].sleeve_length_mm", minimum=1e-6)
        minimum_radius = _number(
            raw.get("minimum_bend_radius_mm", hose.get("minimum_bend_radius", 0.0)),
            f"PIPE_SEGMENTS[{index}].minimum_bend_radius_mm",
            minimum=0.0,
        )
        total_span = hose_length + 2.0 * sleeve_length
        # Manual edits must retain the requested material layout. Never silently
        # replace an overlong hose with metal or move it away from the user target.
        if allow_constraint_warnings and total_span > path_length + 1e-6:
            raise ValueError("路线长度不足以容纳胶管和两端铝套，未接受此次修改")
        if total_span > path_length + 1e-7:
            warnings.append({
                "rule": "hose_total_span",
                "index": index,
                "required_mm": total_span,
                "available_mm": path_length,
                "action": "fallback_to_full_aluminum",
            })
            return {
                "enabled": False,
                "valid": True,
                "components": [],
                "hard_failures": [],
                "warnings": warnings,
                "path_length_mm": path_length,
                "fallback": "all_aluminum",
                "fallback_reason": "hose_total_span",
                "position_s_mm": None,
                "position_ratio": None,
            }

        if raw.get("position_s_mm") is not None:
            hose_center = _number(raw["position_s_mm"], f"PIPE_SEGMENTS[{index}].position_s_mm")
            position_mode = "manual_s"
        elif raw.get("position_ratio") is not None:
            ratio = _number(raw["position_ratio"], f"PIPE_SEGMENTS[{index}].position_ratio", minimum=0.0)
            if ratio > 1.0:
                raise ValueError(f"PIPE_SEGMENTS[{index}].position_ratio must be <= 1")
            hose_center = path_length * ratio
            position_mode = "manual_ratio"
        else:
            hose_center = path_length / 2.0
            position_mode = "center"

        half_span = total_span / 2.0
        requested_center = hose_center
        preferred_center = min(max(hose_center, half_span), path_length - half_span)
        if allow_constraint_warnings and abs(preferred_center - hose_center) > 1e-6:
            raise ValueError("胶管或铝套超出路线端部，未接受此次修改")
        position_adjusted = abs(preferred_center - requested_center) > 1e-7
        if position_mode != "center" and abs(requested_center - preferred_center) > 1e-7:
            warnings.append({
                "rule": "hose_position_out_of_bounds",
                "index": index,
                "requested_center_s_mm": requested_center,
                "allowed_center_range_mm": [half_span, path_length - half_span],
                "action": "clamped_to_route",
            })
        hose_half = hose_length / 2.0
        chosen_center = preferred_center
        hose_start = chosen_center - hose_half
        hose_end = chosen_center + hose_half
        total_start = chosen_center - half_span
        total_end = chosen_center + half_span
        selected_position_s = chosen_center
        selected_position_ratio = chosen_center / path_length if path_length else None
        selected_hose_start = hose_start
        selected_hose_end = hose_end
        radius_report = _hose_radius_report(hose_start, hose_end, arcs, minimum_radius)
        if not radius_report["valid"]:
            warnings.append({
                "rule": "hose_bend_radius",
                "index": index,
                **radius_report,
                "position_s_mm": chosen_center,
                "position_ratio": chosen_center / path_length if path_length else None,
                "action": "generate_anyway",
            })

        if any(max(total_start, other_start) < min(total_end, other_end) - 1e-6 for other_start, other_end in occupied):
            hard_failures.append({"rule": "hose_segment_overlap", "index": index})
        occupied.append((total_start, total_end))

        sleeve_radius = float(hose.get("outer_diameter", 0.0)) / 2.0 + _number(
            raw.get("sleeve_radial_clearance_mm", default_sleeve_radial_clearance_mm),
            f"PIPE_SEGMENTS[{index}].sleeve_radial_clearance_mm",
            minimum=0.0,
        )
        components.extend([
            {"kind": "sleeve", "spec_id": hose_spec, "core_spec": core_spec, "start_s": total_start, "end_s": total_start + sleeve_length, "outer_radius": sleeve_radius},
            {"kind": "hose", "spec_id": hose_spec, "core_spec": core_spec, "start_s": hose_start, "end_s": hose_end, "outer_radius": float(hose["outer_diameter"]) / 2.0},
            {"kind": "sleeve", "spec_id": hose_spec, "core_spec": core_spec, "start_s": hose_end, "end_s": total_end, "outer_radius": sleeve_radius},
        ])
        components[-3]["position_mode"] = position_mode
        components[-3]["position_adjusted"] = position_adjusted
        components[-3]["position_s_mm"] = chosen_center
        components[-3]["position_ratio"] = chosen_center / path_length if path_length else None
        components[-3]["minimum_bend_radius_mm"] = minimum_radius
        components[-3]["length_mm"] = sleeve_length
        components[-2]["position_mode"] = position_mode
        components[-2]["position_adjusted"] = position_adjusted
        components[-2]["position_s_mm"] = chosen_center
        components[-2]["position_ratio"] = chosen_center / path_length if path_length else None
        components[-2]["minimum_bend_radius_mm"] = minimum_radius
        components[-2]["length_mm"] = hose_length
        components[-1]["position_mode"] = position_mode
        components[-1]["position_adjusted"] = position_adjusted
        components[-1]["position_s_mm"] = chosen_center
        components[-1]["position_ratio"] = chosen_center / path_length if path_length else None
        components[-1]["minimum_bend_radius_mm"] = minimum_radius
        components[-1]["length_mm"] = sleeve_length

    # Fill the metal portions around all local components.  Multiple hose
    # entries are kept ordered and must not overlap.
    if components:
        boundaries = sorted({0.0, path_length, *(value for component in components for value in (component["start_s"], component["end_s"]))})
        for start, end in zip(boundaries[:-1], boundaries[1:]):
            if end - start <= 1e-7:
                continue
            local = next((component for component in components if component["start_s"] <= start + 1e-7 and end <= component["end_s"] + 1e-7), None)
            if local is None:
                core_spec = str(entries[0].get("core_spec", "metal_9"))
                components.append({"kind": "aluminum", "spec_id": core_spec, "start_s": start, "end_s": end, "outer_radius": float(specs[core_spec]["outer_diameter"]) / 2.0, "length_mm": end - start})
    components.sort(key=lambda item: (float(item["start_s"]), float(item["end_s"])))
    return {
        "enabled": True,
        "valid": not hard_failures,
        "components": components,
        "hard_failures": hard_failures,
        "warnings": warnings,
        "path_length_mm": path_length,
        "position_s_mm": selected_position_s,
        "position_ratio": selected_position_ratio,
        "hose_start_s_mm": selected_hose_start,
        "hose_end_s_mm": selected_hose_end,
    }


__all__ = [
    "COMPONENT_COLORS",
    "DEFAULT_HOSE_LENGTH_OPTIONS_MM",
    "DEFAULT_HOSE_LENGTH_MM",
    "DEFAULT_SLEEVE_LENGTH_MM",
    "arc_radius_intervals",
    "build_hose_layout",
]
