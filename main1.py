# -*- coding: utf-8 -*-
import settings
from common_space import compute_planning_space, export_planning_space_html


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
