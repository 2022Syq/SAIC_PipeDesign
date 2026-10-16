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
DEFAULT_HOSE_POSITION_SEARCH_STEP_MM = 5.0


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


def _hose_radius_is_valid(start_s: float, end_s: float, arcs: Sequence[tuple[float, float, float]], minimum: float) -> bool:
    for arc_start, arc_end, radius in arcs:
        if min(end_s, arc_end) - max(start_s, arc_start) > 1e-7 and radius + 1e-6 < minimum:
            return False
    return True


def _candidate_centres(
    path_length: float,
    total_span: float,
    preferred: float,
    step: float,
) -> list[float]:
    half = total_span / 2.0
    lo, hi = half, path_length - half
    if hi < lo - 1e-7:
        return []
    preferred = min(max(preferred, lo), hi)
    values = [preferred]
    cursor = lo
    while cursor <= hi + 1e-7:
        values.append(cursor)
        cursor += max(step, 0.1)
    # Searching from both ends avoids missing a close feasible point after a
    # floating-point step lands just beyond a narrow straight interval.
    cursor = hi
    while cursor >= lo - 1e-7:
        values.append(cursor)
        cursor -= max(step, 0.1)
    return sorted(set(round(value, 7) for value in values), key=lambda value: abs(value - preferred))


def build_hose_layout(
    path_length: float,
    pipe_segments: Sequence[Mapping[str, Any]] | None,
    specs: Mapping[str, Mapping[str, Any]],
    engineered_segments: Sequence[Mapping[str, Any]] = (),
    *,
    default_search_step_mm: float = DEFAULT_HOSE_POSITION_SEARCH_STEP_MM,
    default_bend_radius_mode: str = "warning",
    default_hose_length_options_mm: Sequence[float] = DEFAULT_HOSE_LENGTH_OPTIONS_MM,
    default_hose_length_mm: float = DEFAULT_HOSE_LENGTH_MM,
    default_sleeve_length_mm: float = DEFAULT_SLEEVE_LENGTH_MM,
) -> dict[str, Any]:
    """Resolve optional hose/sleeve intervals and their constraint report.

    ``position_s_mm`` is the centre of the hose itself.  When omitted, the
    centre of the complete ``sleeve + hose + sleeve`` span is used.  A segment
    with ``bend_radius_mode='warning'`` remains usable when no R50-safe
    position exists, while recording the exact violation for the HTML/report.
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
        hose_length = _number(raw.get("hose_length_mm", minimum_length), f"PIPE_SEGMENTS[{index}].hose_length_mm", minimum=minimum_length)
        sleeve_length = _number(raw.get("sleeve_length_mm", default_sleeve_length_mm), f"PIPE_SEGMENTS[{index}].sleeve_length_mm", minimum=1e-6)
        minimum_radius = _number(
            raw.get("minimum_bend_radius_mm", hose.get("minimum_bend_radius", 0.0)),
            f"PIPE_SEGMENTS[{index}].minimum_bend_radius_mm",
            minimum=0.0,
        )
        mode = str(raw.get("bend_radius_mode", default_bend_radius_mode)).strip().lower()
        if mode not in {"warning", "hard"}:
            raise ValueError(f"PIPE_SEGMENTS[{index}].bend_radius_mode must be 'warning' or 'hard'")

        total_span = hose_length + 2.0 * sleeve_length
        if total_span > path_length + 1e-7:
            hard_failures.append({
                "rule": "hose_total_span",
                "index": index,
                "required_mm": total_span,
                "available_mm": path_length,
            })
            continue

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
        if position_mode != "center" and abs(requested_center - preferred_center) > 1e-7:
            hard_failures.append({
                "rule": "hose_position_out_of_bounds",
                "index": index,
                "requested_center_s_mm": requested_center,
                "allowed_center_range_mm": [half_span, path_length - half_span],
            })
        hose_half = hose_length / 2.0
        chosen_center = preferred_center
        manual_position = position_mode != "center"
        # A future UI position is an explicit engineering choice. Preserve it
        # exactly and report R50 as a warning; only the automatic centre
        # placement may move to a nearby safe interval.
        candidates = [preferred_center] if manual_position else _candidate_centres(
            path_length,
            total_span,
            preferred_center,
            float(raw.get("position_search_step_mm", default_search_step_mm)),
        )
        if manual_position:
            mode = "warning"
        for candidate in candidates:
            hose_start = candidate - hose_half
            hose_end = candidate + hose_half
            if _hose_radius_is_valid(hose_start, hose_end, arcs, minimum_radius):
                chosen_center = candidate
                break
        hose_start = chosen_center - hose_half
        hose_end = chosen_center + hose_half
        total_start = chosen_center - half_span
        total_end = chosen_center + half_span
        radius_valid = _hose_radius_is_valid(hose_start, hose_end, arcs, minimum_radius)
        adjusted = abs(chosen_center - preferred_center) > 1e-5
        if adjusted:
            warnings.append({
                "rule": "hose_position_adjusted",
                "index": index,
                "requested_center_s_mm": preferred_center,
                "selected_center_s_mm": chosen_center,
            })
        if not radius_valid:
            violation = {
                "rule": "hose_bend_radius",
                "index": index,
                "actual_radii_mm": [radius for start, end, radius in arcs if min(hose_end, end) - max(hose_start, start) > 1e-7],
                "minimum_mm": minimum_radius,
                "position_s_mm": chosen_center,
            }
            if mode == "hard":
                hard_failures.append(violation)
            else:
                warnings.append({**violation, "mode": "warning"})

        if any(max(total_start, other_start) < min(total_end, other_end) - 1e-6 for other_start, other_end in occupied):
            hard_failures.append({"rule": "hose_segment_overlap", "index": index})
        occupied.append((total_start, total_end))

        sleeve_radius = float(hose.get("outer_diameter", 0.0)) / 2.0 + _number(
            raw.get("sleeve_radial_clearance_mm", 1.0),
            f"PIPE_SEGMENTS[{index}].sleeve_radial_clearance_mm",
            minimum=0.0,
        )
        components.extend([
            {"kind": "sleeve", "spec_id": hose_spec, "core_spec": core_spec, "start_s": total_start, "end_s": total_start + sleeve_length, "outer_radius": sleeve_radius},
            {"kind": "hose", "spec_id": hose_spec, "core_spec": core_spec, "start_s": hose_start, "end_s": hose_end, "outer_radius": float(hose["outer_diameter"]) / 2.0},
            {"kind": "sleeve", "spec_id": hose_spec, "core_spec": core_spec, "start_s": hose_end, "end_s": total_end, "outer_radius": sleeve_radius},
        ])
        components[-3]["position_mode"] = position_mode
        components[-3]["position_adjusted"] = adjusted
        components[-3]["minimum_bend_radius_mm"] = minimum_radius
        components[-3]["length_mm"] = sleeve_length
        components[-2]["position_mode"] = position_mode
        components[-2]["position_adjusted"] = adjusted
        components[-2]["minimum_bend_radius_mm"] = minimum_radius
        components[-2]["length_mm"] = hose_length
        components[-1]["position_mode"] = position_mode
        components[-1]["position_adjusted"] = adjusted
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
    }


__all__ = [
    "COMPONENT_COLORS",
    "DEFAULT_HOSE_LENGTH_OPTIONS_MM",
    "DEFAULT_HOSE_LENGTH_MM",
    "DEFAULT_SLEEVE_LENGTH_MM",
    "arc_radius_intervals",
    "build_hose_layout",
]
