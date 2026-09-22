# -*- coding: utf-8 -*-
"""
Read route helper points from the active CATIA Part and print settings.py coordinates.

Default CATIA feature set:
    point

Default point names:
    START
    GOAL
    START_DIR_POINT
    GOAL_DIR_POINT
    GUADIAN / GUADIAN1 ... GUADIAN10
"""

from __future__ import annotations

import argparse
import ast
import math
import re
import sys
from pathlib import Path

import settings


DEFAULT_NAMES = {
    "START": "START",
    "GOAL": "GOAL",
    "START_DIR_POINT": "START_DIR_POINT",
    "GOAL_DIR_POINT": "GOAL_DIR_POINT",
}

HANGER_POINT_NAMES = ["GUADIAN"] + [f"GUADIAN{i}" for i in range(1, 11)]


def coordinate_key(point: list[float], tolerance: float = 1e-5) -> tuple[int, int, int]:
    scale = 1.0 / max(float(tolerance), 1e-12)
    return tuple(int(round(float(value) * scale)) for value in point)


def dedupe_coordinate_points(points: list[list[float]], tolerance: float = 1e-5) -> list[list[float]]:
    seen = set()
    unique = []
    for point in points:
        key = coordinate_key(point, tolerance)
        if key in seen:
            continue
        seen.add(key)
        unique.append(point)
    return unique


def dedupe_named_coordinate_points(
    points: list[list[float]], names: list[str], tolerance: float = 1e-5
) -> tuple[list[list[float]], list[str]]:
    seen = set()
    unique_points = []
    unique_names = []
    for point, name in zip(points, names):
        key = coordinate_key(point, tolerance)
        if key in seen:
            continue
        seen.add(key)
        unique_points.append(point)
        unique_names.append(name)
    return unique_points, unique_names

DEFAULT_ALIASES = {
    "START": ["START", "Start", "start", "S", "A"],
    "GOAL": ["GOAL", "Goal", "goal", "G", "B"],
    "START_DIR_POINT": [
        "START_DIR_POINT",
        "START_DIR",
        "Start_Dir",
        "start_dir",
        "START_DIRECTION",
        "C",
    ],
    "GOAL_DIR_POINT": [
        "GOAL_DIR_POINT",
        "GOAL_DIR",
        "Goal_Dir",
        "goal_dir",
        "GOAL_DIRECTION",
        "D",
    ],
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Measure START/GOAL direction helper points from CATIA."
    )
    parser.add_argument(
        "--body",
        default="point",
        help='HybridBody / geometrical set name or path. Default: "point".',
    )
    parser.add_argument(
        "--documents",
        action="store_true",
        help="List open CATIA documents, then exit.",
    )
    parser.add_argument(
        "--document",
        help="Use an open CATIA document by exact name or name fragment.",
    )
    parser.add_argument("--start", default=DEFAULT_NAMES["START"], help="START point name.")
    parser.add_argument("--goal", default=DEFAULT_NAMES["GOAL"], help="GOAL point name.")
    parser.add_argument(
        "--start-dir",
        default=DEFAULT_NAMES["START_DIR_POINT"],
        help="START_DIR_POINT point name.",
    )
    parser.add_argument(
        "--goal-dir",
        default=DEFAULT_NAMES["GOAL_DIR_POINT"],
        help="GOAL_DIR_POINT point name.",
    )
    parser.add_argument(
        "--no-aliases",
        action="store_true",
        help="Only use the exact names passed on the command line.",
    )
    parser.add_argument(
        "--write-settings",
        dest="write_settings",
        action="store_true",
        help="Update measured coordinates in settings.py (default).",
    )
    parser.add_argument(
        "--no-write-settings",
        dest="write_settings",
        action="store_false",
        help="Only print measured coordinates; do not modify settings.py.",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="List hybrid bodies and shapes in the active Part, then exit.",
    )
    parser.add_argument(
        "--search",
        help="Search active CATIA document by name pattern, for example point or START.",
    )
    parser.add_argument(
        "--selection",
        action="store_true",
        help="Use the first selected CATIA container as the point set.",
    )
    parser.add_argument(
        "--settings-file",
        type=Path,
        default=Path(settings.PROJECT_DIR) / "settings.py",
        help="settings.py path used with --write-settings.",
    )
    parser.set_defaults(write_settings=True)
    return parser.parse_args()


def import_catia():
    try:
        from pycatia import catia
        return catia
    except Exception as exc:
        raise RuntimeError(
            "main_point.py requires pycatia. Install it in the active Python environment."
        ) from exc


def collection_count(collection) -> int:
    for attr in ("count", "Count"):
        try:
            return int(getattr(collection, attr))
        except Exception:
            pass
    return 0


def documents_list(caa) -> list:
    documents = caa.documents
    result = []
    for index in range(1, collection_count(documents) + 1):
        doc = collection_item(documents, index)
        if doc is not None:
            result.append(doc)
    return result


def document_part(document):
    try:
        return document.part
    except Exception:
        return None


def list_documents(caa) -> None:
    active_name = item_name(caa.active_document)
    print("=== CATIA Documents ===")
    print(f"Active: {active_name}")
    for index, doc in enumerate(documents_list(caa), start=1):
        part = document_part(doc)
        part_text = f", part={item_name(part)}" if part is not None else ""
        path = item_full_name(doc)
        path_text = f", path={path}" if path else ""
        marker = "*" if item_name(doc) == active_name else " "
        print(f"{marker} {index:03d}. {item_name(doc)} [{type(doc).__name__}{part_text}{path_text}]")


def select_document(caa, document_name: str):
    if not document_name:
        return caa.active_document

    docs = documents_list(caa)
    wanted = document_name.casefold()
    exact = [doc for doc in docs if item_name(doc).casefold() == wanted]
    if exact:
        return exact[0]

    partial = [doc for doc in docs if wanted in item_name(doc).casefold()]
    if len(partial) == 1:
        return partial[0]
    if len(partial) > 1:
        names = ", ".join(item_name(doc) for doc in partial[:20])
        raise KeyError(f'Document name "{document_name}" is ambiguous. Matches: {names}')

    names = ", ".join(item_name(doc) for doc in docs[:40])
    raise KeyError(f'Document not found: "{document_name}". Open documents include: {names}')


def item_name(item) -> str:
    for attr in ("name", "Name"):
        try:
            return str(getattr(item, attr))
        except Exception:
            pass
    return "<unnamed>"


def item_full_name(item) -> str:
    for attr in ("full_name", "FullName"):
        try:
            value = getattr(item, attr)
            if value:
                return str(value)
        except Exception:
            pass
    return ""


def get_sub_hybrid_bodies(item):
    for attr in ("hybrid_bodies", "HybridBodies"):
        try:
            return getattr(item, attr)
        except Exception:
            pass
    return None


def get_sub_ordered_geometrical_sets(item):
    for attr in ("ordered_geometrical_sets", "OrderedGeometricalSets"):
        try:
            return getattr(item, attr)
        except Exception:
            pass
    return None


def get_hybrid_shapes(item):
    for attr in ("hybrid_shapes", "HybridShapes"):
        try:
            return getattr(item, attr)
        except Exception:
            pass
    return None


def get_collection(item, *names):
    for attr in names:
        try:
            collection = getattr(item, attr)
            if collection is not None:
                return collection
        except Exception:
            pass
    return None


def collection_item(collection, index: int):
    for method_name in ("item", "Item"):
        try:
            item = getattr(collection, method_name)(index)
            if item is not None:
                return item
        except Exception:
            pass
    return None


def collection_names(collection) -> list[str]:
    names = []
    for index in range(1, collection_count(collection) + 1):
        item = collection_item(collection, index)
        if item is None:
            continue
        names.append(item_name(item))
    return names


def get_item_by_name(collection, name: str):
    for method_name in ("get_item_by_name", "item", "Item"):
        try:
            item = getattr(collection, method_name)(name)
            if item is not None:
                return item
        except Exception:
            pass

    wanted = str(name).casefold()
    for index in range(1, collection_count(collection) + 1):
        item = collection_item(collection, index)
        if item is None:
            continue
        if item_name(item).casefold() == wanted:
            return item

    available = ", ".join(collection_names(collection))
    raise KeyError(f'Cannot find "{name}". Available: {available}')


def search_by_name(document, pattern: str):
    selection = document.selection
    results = []
    queries = [f"Name={pattern},all"]
    if "*" not in pattern:
        queries.append(f"Name=*{pattern}*,all")

    for query in queries:
        try:
            selection.clear()
            selection.search(query)
            count = int(selection.count)
            for index in range(1, count + 1):
                try:
                    value = selection.item(index).value
                except Exception:
                    continue
                if value is not None and value not in results:
                    results.append(value)
        except Exception:
            pass
    try:
        selection.clear()
    except Exception:
        pass
    return results


def print_search_results(document, pattern: str) -> None:
    print(f'=== CATIA Search: "{pattern}" ===')
    results = search_by_name(document, pattern)
    if not results:
        print("(no results)")
        return
    for index, value in enumerate(results, start=1):
        print(f"{index}. {item_name(value)}  [{type(value).__name__}]")


def selected_objects(document) -> list:
    selection = document.selection
    objects = []
    try:
        count = int(selection.count)
    except Exception:
        count = 0
    for index in range(1, count + 1):
        try:
            value = selection.item(index).value
            if value is not None:
                objects.append(value)
        except Exception:
            pass
    return objects


def selected_container(document):
    for obj in selected_objects(document):
        if get_hybrid_shapes(obj) is not None:
            return obj
    return None


def list_shapes(container, indent: int = 0) -> None:
    shapes = get_hybrid_shapes(container)
    if shapes is None:
        return
    prefix = "  " * indent
    for shape_index in range(1, collection_count(shapes) + 1):
        shape = collection_item(shapes, shape_index)
        if shape is not None:
            print(f"{prefix}- Shape: {item_name(shape)}")


def list_container_tree(container, label: str, indent: int = 0) -> None:
    prefix = "  " * indent
    print(f"{prefix}- {label}: {item_name(container)}")
    list_shapes(container, indent + 1)

    child_bodies = get_sub_hybrid_bodies(container)
    if child_bodies is not None:
        list_hybrid_bodies(child_bodies, indent + 1)

    child_sets = get_sub_ordered_geometrical_sets(container)
    if child_sets is not None:
        list_ordered_geometrical_sets(child_sets, indent + 1)


def list_hybrid_bodies(collection, indent: int = 0) -> None:
    for index in range(1, collection_count(collection) + 1):
        body = collection_item(collection, index)
        if body is None:
            continue
        list_container_tree(body, "HybridBody", indent)


def list_ordered_geometrical_sets(collection, indent: int = 0) -> None:
    for index in range(1, collection_count(collection) + 1):
        ogs = collection_item(collection, index)
        if ogs is None:
            continue
        list_container_tree(ogs, "OrderedGeometricalSet", indent)


def list_bodies(collection, indent: int = 0) -> None:
    for index in range(1, collection_count(collection) + 1):
        body = collection_item(collection, index)
        if body is None:
            continue
        list_container_tree(body, "Body", indent)


def iter_child_containers(container):
    child_bodies = get_sub_hybrid_bodies(container)
    if child_bodies is not None:
        for index in range(1, collection_count(child_bodies) + 1):
            child = collection_item(child_bodies, index)
            if child is not None:
                yield child

    child_sets = get_sub_ordered_geometrical_sets(container)
    if child_sets is not None:
        for index in range(1, collection_count(child_sets) + 1):
            child = collection_item(child_sets, index)
            if child is not None:
                yield child


def recursive_find_container(collection, name: str):
    wanted = str(name).casefold()
    for index in range(1, collection_count(collection) + 1):
        container = collection_item(collection, index)
        if container is None:
            continue
        if item_name(container).casefold() == wanted:
            return container
        for child in iter_child_containers(container):
            found = recursive_find_container_from_root(child, wanted)
            if found is not None:
                return found
    return None


def recursive_find_container_from_root(container, wanted_casefold: str):
    if item_name(container).casefold() == wanted_casefold:
        return container
    for child in iter_child_containers(container):
        found = recursive_find_container_from_root(child, wanted_casefold)
        if found is not None:
            return found
    return None


def find_container_anywhere(part, name: str):
    collections = [
        get_collection(part, "hybrid_bodies", "HybridBodies"),
        get_collection(part, "ordered_geometrical_sets", "OrderedGeometricalSets"),
        get_collection(part, "bodies", "Bodies"),
    ]
    for collection in collections:
        if collection is None:
            continue
        found = recursive_find_container(collection, name)
        if found is not None:
            return found
    return None


def get_shape_global(document, names: list[str]):
    last_error = None
    for name in names:
        results = search_by_name(document, name)
        for value in results:
            try:
                # Keep only objects that CATIA SPA can measure as a point.
                reference = document.part.create_reference_from_object(value)
                document.spa_workbench().get_measurable(reference).get_point()
                return value, item_name(value)
            except Exception as exc:
                last_error = exc
    raise KeyError(f"Point not found globally. Tried: {', '.join(names)}. {last_error}")


def get_hybrid_body(part, body_path: str):
    tokens = [token for token in re.split(r"[\\/]+", str(body_path)) if token]
    if not tokens:
        raise ValueError("HybridBody path is empty.")

    current = None
    bodies = get_collection(part, "hybrid_bodies", "HybridBodies")
    if bodies is None:
        raise KeyError("Active Part has no HybridBodies collection.")

    if len(tokens) == 1:
        found = find_container_anywhere(part, tokens[0])
        if found is not None:
            return found

    for token_index, token in enumerate(tokens):
        current = get_item_by_name(bodies, token)
        child_bodies = get_sub_hybrid_bodies(current)
        if child_bodies is None and token_index != len(tokens) - 1:
            available = ", ".join(collection_names(bodies))
            raise KeyError(
                f'HybridBody "{item_name(current)}" has no child HybridBodies. '
                f'Remaining path: {" / ".join(tokens[token_index + 1:])}. '
                f"Available here: {available}"
            )
        if child_bodies is not None:
            bodies = child_bodies
    return current


def candidate_names(setting_name: str, requested_name: str, use_aliases: bool) -> list[str]:
    names = [requested_name]
    if use_aliases and requested_name == DEFAULT_NAMES[setting_name]:
        names.extend(DEFAULT_ALIASES[setting_name])

    out = []
    for name in names:
        if name not in out:
            out.append(name)
    return out


def get_shape_with_candidates(hybrid_body, names: list[str]):
    last_error = None
    shapes = get_hybrid_shapes(hybrid_body)
    if shapes is None:
        raise KeyError(f'HybridBody "{item_name(hybrid_body)}" has no HybridShapes.')
    for name in names:
        try:
            return get_item_by_name(shapes, name), name
        except Exception as exc:
            last_error = exc
    raise KeyError(f"Point not found. Tried: {', '.join(names)}. {last_error}")


def measure_point(part, spa_workbench, point_shape) -> list[float]:
    reference = part.create_reference_from_object(point_shape)
    measurable = spa_workbench.get_measurable(reference)
    coords = measurable.get_point()
    return [float(coords[0]), float(coords[1]), float(coords[2])]


def format_float(value: float) -> str:
    value = float(value)
    if abs(value) < 1e-10:
        value = 0.0
    text = f"{value:.12f}".rstrip("0").rstrip(".")
    if "." not in text and "e" not in text.lower():
        text += ".0"
    return text


def format_vector(values: list[float]) -> str:
    return "[" + ", ".join(format_float(value) for value in values) + "]"


def format_settings_value(value) -> str:
    if isinstance(value, (list, tuple)):
        if len(value) == 3 and all(isinstance(v, (int, float)) for v in value):
            return format_vector(value)
        return "[" + ", ".join(format_settings_value(item) for item in value) + "]"
    return repr(value)


def validate_measured_points(measured: dict[str, list[float]]) -> None:
    for name in DEFAULT_NAMES:
        values = measured.get(name)
        if values is None or len(values) != 3:
            raise ValueError(f"{name} must contain exactly three coordinates")
        if not all(math.isfinite(float(value)) for value in values):
            raise ValueError(f"{name} contains a non-finite coordinate: {values}")
    for point in measured.get("HANGER_POINTS", []):
        if len(point) != 3 or not all(math.isfinite(float(value)) for value in point):
            raise ValueError(f"HANGER_POINTS contains an invalid coordinate: {point}")


def updated_settings_text(text: str, measured: dict[str, list[float]]) -> str:
    """Replace active coordinate assignments without touching comments/strings."""
    validate_measured_points(measured)
    tree = ast.parse(text)
    assignments: dict[str, ast.Assign | ast.AnnAssign] = {}
    for node in ast.walk(tree):
        targets = []
        if isinstance(node, ast.Assign):
            targets = node.targets
        elif isinstance(node, ast.AnnAssign):
            targets = [node.target]
        for target in targets:
            if isinstance(target, ast.Name) and target.id in measured:
                assignments[target.id] = node

    lines = text.splitlines(keepends=True)
    replacements = []
    missing = []
    for name, values in measured.items():
        node = assignments.get(name)
        line = f"{name} = {format_settings_value(values)}\n"
        if node is None:
            missing.append(line)
            continue
        start = int(node.lineno) - 1
        end = int(getattr(node, "end_lineno", node.lineno))
        replacements.append((start, end, line))

    for start, end, line in sorted(replacements, reverse=True):
        lines[start:end] = [line]

    result = "".join(lines)
    if missing:
        if result and not result.endswith("\n"):
            result += "\n"
        result += "".join(missing)
    return result


def update_settings_file(path: Path, measured: dict[str, list[float]]) -> None:
    path = Path(path).resolve()
    text = path.read_text(encoding="utf-8")
    updated = updated_settings_text(text, measured)
    if updated == text:
        print(f"[SAVE] settings coordinates unchanged: {path}")
        return

    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(updated, encoding="utf-8")
    temporary.replace(path)


def optional_point_from_candidates(document, hybrid_body, names: list[str]):
    try:
        if hybrid_body is None:
            shape, actual_name = get_shape_global(document, names)
        else:
            shape, actual_name = get_shape_with_candidates(hybrid_body, names)
        return shape, actual_name
    except Exception:
        return None, None


def main() -> None:
    args = parse_args()
    catia = import_catia()

    caa = catia()
    if args.documents:
        list_documents(caa)
        return

    document = select_document(caa, args.document)
    part = document_part(document)
    if part is None:
        raise TypeError(f"Selected document is not a CATPart document: {item_name(document)}")
    spa_workbench = document.spa_workbench()

    doc_name = item_name(document)
    part_name = item_name(part)
    doc_path = item_full_name(document)
    print(f"[CATIA] Active document: {doc_name}")
    print(f"[CATIA] Active part: {part_name}")
    print(f"[CATIA] Active path: {doc_path if doc_path else '(unsaved or unavailable)'}")
    selected = selected_objects(document)
    if selected:
        print("[CATIA] Selection: " + ", ".join(item_name(obj) for obj in selected))

    if args.search:
        print_search_results(document, args.search)
        return

    if args.list:
        print("=== CATIA Containers ===")
        hybrid_bodies = get_collection(part, "hybrid_bodies", "HybridBodies")
        if hybrid_bodies is not None:
            print("[HybridBodies]")
            list_hybrid_bodies(hybrid_bodies)

        ordered_sets = get_collection(part, "ordered_geometrical_sets", "OrderedGeometricalSets")
        if ordered_sets is not None:
            print("[OrderedGeometricalSets]")
            list_ordered_geometrical_sets(ordered_sets)

        bodies = get_collection(part, "bodies", "Bodies")
        if bodies is not None:
            print("[Bodies]")
            list_bodies(bodies)
        return

    hybrid_body = None
    if args.selection:
        hybrid_body = selected_container(document)
        if hybrid_body is None:
            raise KeyError("No selected CATIA container with HybridShapes. Select the point set and retry.")
    else:
        try:
            hybrid_body = get_hybrid_body(part, args.body)
        except Exception as exc:
            print(f'[WARN] cannot find body "{args.body}": {exc}')
            hybrid_body = selected_container(document)
            if hybrid_body is not None:
                print(f"[WARN] fallback: use selected container {item_name(hybrid_body)}.")
            else:
                print("[WARN] fallback: search helper points globally in the active document.")

    use_aliases = not args.no_aliases
    requested = {
        "START": args.start,
        "GOAL": args.goal,
        "START_DIR_POINT": args.start_dir,
        "GOAL_DIR_POINT": args.goal_dir,
    }

    measured = {}
    resolved_names = {}
    for setting_name, requested_name in requested.items():
        names = candidate_names(setting_name, requested_name, use_aliases)
        if hybrid_body is None:
            shape, actual_name = get_shape_global(document, names)
        else:
            shape, actual_name = get_shape_with_candidates(hybrid_body, names)
        measured[setting_name] = measure_point(part, spa_workbench, shape)
        resolved_names[setting_name] = actual_name

    hanger_points = []
    hanger_names = []
    for hanger_name in HANGER_POINT_NAMES:
        shape, actual_name = optional_point_from_candidates(document, hybrid_body, [hanger_name])
        if shape is None:
            continue
        hanger_points.append(measure_point(part, spa_workbench, shape))
        hanger_names.append(actual_name)
    hanger_points, hanger_names = dedupe_named_coordinate_points(hanger_points, hanger_names)
    measured["HANGER_POINTS"] = hanger_points

    print("=== CATIA Helper Points ===")
    print(f"HybridBody: {item_name(hybrid_body) if hybrid_body is not None else '(global search)'}")
    for name in ("START", "GOAL", "START_DIR_POINT", "GOAL_DIR_POINT"):
        print(f"{name}_POINT_NAME = {resolved_names[name]}")
    if hanger_names:
        print("HANGER_POINT_NAMES = " + ", ".join(hanger_names))
    else:
        print("HANGER_POINT_NAMES = (none found)")
    print()
    for name in ("START", "GOAL", "START_DIR_POINT", "GOAL_DIR_POINT"):
        print(f"{name} = {format_vector(measured[name])}")
    print(f"HANGER_POINTS = {format_settings_value(measured['HANGER_POINTS'])}")

    validate_measured_points(measured)

    if args.write_settings:
        update_settings_file(args.settings_file, measured)
        print()
        print(f"[SAVE] updated: {args.settings_file}")


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        print(f"[ERROR] {type(exc).__name__}: {exc}", file=sys.stderr)
        raise
