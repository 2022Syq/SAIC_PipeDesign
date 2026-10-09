# -*- coding: utf-8 -*-
from contextlib import contextmanager
from pathlib import Path

import settings
from common_space import compute_planning_space, export_planning_space_html


@contextmanager
def _temporary_space_settings(start, goal, pipe_radius, output_dir):
    """Apply one shared-space context without writing settings.py."""
    output_dir = Path(output_dir)
    overrides = {
        "START": list(start),
        "GOAL": list(goal),
        "PIPE_RADIUS": float(pipe_radius),
        "ROUTE_MAX_ENVELOPE_RADIUS": float(pipe_radius),
        "SPACE_JSON": output_dir / "planning_space.json",
        "SPACE_CHECKPOINT_JSON": output_dir / "planning_space_checkpoint.json",
        "SPACE_HTML": output_dir / "planning_space.html",
        "X_RANGE": None,
    }
    previous = {name: getattr(settings, name, None) for name in overrides}
    missing = {name for name in overrides if not hasattr(settings, name)}
    try:
        for name, value in overrides.items():
            setattr(settings, name, value)
        yield
    finally:
        for name in missing:
            if hasattr(settings, name):
                delattr(settings, name)
        for name, value in previous.items():
            setattr(settings, name, value)


def build_shared_space(start, goal, pipe_radius, output_dir):
    """Build one planning space for a multi-pipe run.

    The default ``main()`` path below remains the original single-pipe path.
    This helper only changes module values in memory and restores them before
    returning, so multi-pipe coordinate data never overwrites ``settings.py``.
    """
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    with _temporary_space_settings(start, goal, pipe_radius, output_dir):
        space = compute_planning_space(settings)
        export_planning_space_html(space, settings, settings.SPACE_HTML)
        space["source_json"] = str(settings.SPACE_JSON)
        return space


def main() -> None:
    print("=== Main1: Mesh Section Planning Space ===")
    print(f"BASE_STEP_DIR: {settings.BASE_STEP_DIR}")
    print(f"SECTION_WIDTH_Y: {settings.SECTION_WIDTH_Y}")
    print(f"SECTION_HEIGHT_ABOVE_GROUND: {settings.SECTION_HEIGHT_ABOVE_GROUND}")
    print(f"SECTION_DX: {settings.SECTION_DX}")
    print(f"SECTION_DY: {settings.SECTION_DY}")
    print(f"ProcessPoolExecutor max_workers={settings.SECTION_PARALLEL_WORKERS}")

    space = compute_planning_space(settings)
    export_planning_space_html(space, settings, settings.SPACE_HTML)

    print("=== Done ===")
    print(f"SPACE_JSON: {settings.SPACE_JSON}")
    print(f"SPACE_CHECKPOINT_JSON: {settings.SPACE_CHECKPOINT_JSON}")
    print(f"SPACE_HTML: {settings.SPACE_HTML}")
    print(f"x_sections: {len(space.get('x_sections', space['sections']))}")
    print(f"y_sections: {len(space.get('y_sections', []))}")


if __name__ == "__main__":
    main()
