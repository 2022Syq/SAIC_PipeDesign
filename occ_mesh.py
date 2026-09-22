# -*- coding: utf-8 -*-
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass
class StepComponent:
    name: str
    file: str
    role: str
    clearance: float
    solid: bool
    shape: object | None = None
    mesh: dict | None = None


CAD_INPUT_EXTENSIONS = (".stp", ".step", ".igs", ".iges", ".stl")


def read_step_shape(path: Path):
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.STEPControl import STEPControl_Reader

    path = Path(path)
    reader = STEPControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"STEP read failed: {path}")
    reader.TransferRoots()
    return reader.OneShape()


def read_iges_shape(path: Path):
    from OCC.Core.IFSelect import IFSelect_RetDone
    from OCC.Core.IGESControl import IGESControl_Reader

    path = Path(path)
    reader = IGESControl_Reader()
    status = reader.ReadFile(str(path))
    if status != IFSelect_RetDone:
        raise RuntimeError(f"IGES read failed: {path}")

    reader.TransferRoots()
    shape = reader.OneShape()
    if shape is not None and not shape.IsNull():
        return shape

    for index in range(1, reader.NbRootsForTransfer() + 1):
        try:
            reader.TransferOneRoot(index)
        except Exception:
            continue

    shape = reader.OneShape()
    if shape is None or shape.IsNull():
        raise RuntimeError(f"IGES transfer produced no OCC shape: {path}")

    return shape

def _empty_mesh() -> dict:
    return {"x": [], "y": [], "z": [], "i": [], "j": [], "k": [], "triangle_count": 0}


def _append_stl_triangle(mesh: dict, points: list[tuple[float, float, float]]) -> None:
    base = len(mesh["x"])
    for x, y, z in points:
        mesh["x"].append(float(x))
        mesh["y"].append(float(y))
        mesh["z"].append(float(z))
    mesh["i"].append(base)
    mesh["j"].append(base + 1)
    mesh["k"].append(base + 2)


def _is_binary_stl(path: Path) -> bool:
    import struct

    size = path.stat().st_size
    if size < 84:
        return False
    with path.open("rb") as file:
        file.seek(80)
        triangle_count = struct.unpack("<I", file.read(4))[0]
    return size == 84 + triangle_count * 50


def read_binary_stl_mesh(path: Path) -> dict:
    import struct

    mesh = _empty_mesh()
    record = struct.Struct("<12fH")
    with Path(path).open("rb") as file:
        file.seek(80)
        triangle_count = struct.unpack("<I", file.read(4))[0]
        for _ in range(triangle_count):
            values = record.unpack(file.read(50))
            _append_stl_triangle(
                mesh,
                [
                    (values[3], values[4], values[5]),
                    (values[6], values[7], values[8]),
                    (values[9], values[10], values[11]),
                ],
            )
    mesh["triangle_count"] = len(mesh["i"])
    return mesh


def read_ascii_stl_mesh(path: Path) -> dict:
    mesh = _empty_mesh()
    triangle: list[tuple[float, float, float]] = []

    with Path(path).open("r", encoding="utf-8", errors="ignore") as file:
        for line in file:
            parts = line.strip().split()
            if len(parts) >= 4 and parts[0].lower() == "vertex":
                triangle.append((float(parts[1]), float(parts[2]), float(parts[3])))
                if len(triangle) == 3:
                    _append_stl_triangle(mesh, triangle)
                    triangle = []

    mesh["triangle_count"] = len(mesh["i"])
    return mesh


def read_stl_mesh(path: Path) -> dict:
    path = Path(path)
    mesh = read_binary_stl_mesh(path) if _is_binary_stl(path) else read_ascii_stl_mesh(path)
    if mesh["triangle_count"] <= 0:
        raise RuntimeError(f"STL mesh has no triangles: {path}")
    return mesh


def is_mesh_dict(value) -> bool:
    return isinstance(value, dict) and all(key in value for key in ("x", "y", "z", "i", "j", "k"))

def read_cad_shape(path: Path):
    path = Path(path)
    suffix = path.suffix.lower()
    if suffix in {".stp", ".step"}:
        return read_step_shape(path)
    if suffix in {".igs", ".iges"}:
        return read_iges_shape(path)
    if suffix == ".stl":
        return read_stl_mesh(path)
    raise ValueError(f"Unsupported CAD input format: {path}")


def triangulate_shape_to_mesh(
    shape,
    linear_deflection: float = 8.0,
    angular_deflection: float = 0.5,
    max_triangles: int | None = None,
) -> dict:
    if is_mesh_dict(shape):
        return shape
    if shape is None:
        raise RuntimeError("Cannot triangulate an empty CAD shape.")

    from OCC.Core.BRep import BRep_Tool
    from OCC.Core.BRepMesh import BRepMesh_IncrementalMesh
    from OCC.Core.TopAbs import TopAbs_FACE
    from OCC.Core.TopExp import TopExp_Explorer
    from OCC.Core.TopLoc import TopLoc_Location

    BRepMesh_IncrementalMesh(
        shape,
        float(linear_deflection),
        False,
        float(angular_deflection),
        True,
    )

    xs: list[float] = []
    ys: list[float] = []
    zs: list[float] = []
    ii: list[int] = []
    jj: list[int] = []
    kk: list[int] = []

    exp = TopExp_Explorer(shape, TopAbs_FACE)
    while exp.More():
        face = exp.Current()
        loc = TopLoc_Location()
        tri = BRep_Tool.Triangulation(face, loc)
        if tri is not None:
            trsf = loc.Transformation()
            base = len(xs)

            for idx in range(1, tri.NbNodes() + 1):
                try:
                    p = tri.Node(idx)
                except Exception:
                    p = tri.Nodes().Value(idx)
                p = p.Transformed(trsf)
                xs.append(float(p.X()))
                ys.append(float(p.Y()))
                zs.append(float(p.Z()))

            for idx in range(1, tri.NbTriangles() + 1):
                if max_triangles is not None and len(ii) >= int(max_triangles):
                    break
                try:
                    t = tri.Triangle(idx)
                except Exception:
                    t = tri.Triangles().Value(idx)
                n1, n2, n3 = t.Get()
                ii.append(base + n1 - 1)
                jj.append(base + n2 - 1)
                kk.append(base + n3 - 1)

        if max_triangles is not None and len(ii) >= int(max_triangles):
            break
        exp.Next()

    return {"x": xs, "y": ys, "z": zs, "i": ii, "j": jj, "k": kk, "triangle_count": len(ii)}


def mesh_points(mesh: dict) -> list[list[float]]:
    return [[float(x), float(y), float(z)] for x, y, z in zip(mesh["x"], mesh["y"], mesh["z"])]


def infer_ground_z(mesh: dict, default_z: float = -300.0) -> float:
    zs = [float(z) for z in mesh.get("z", [])]
    if not zs:
        return float(default_z)
    zs.sort()
    return float(zs[len(zs) // 2])


def scan_cad_files(base_step_dir: Path) -> list[Path]:
    base_step_dir = Path(base_step_dir)
    if not base_step_dir.exists():
        raise FileNotFoundError(f"BASE_STEP_DIR does not exist: {base_step_dir}")
    files: list[Path] = []
    for suffix in CAD_INPUT_EXTENSIONS:
        files.extend(base_step_dir.glob(f"*{suffix}"))
        files.extend(base_step_dir.glob(f"*{suffix.upper()}"))
    unique = {str(p).lower(): p for p in files}
    return sorted(unique.values(), key=lambda p: p.name.lower())


def scan_step_files(base_step_dir: Path) -> list[Path]:
    """Backward-compatible name; now scans STEP and IGES files."""
    return scan_cad_files(base_step_dir)


def component_rule(step_path: Path, settings) -> tuple[str, float, bool]:
    key_file = step_path.name.lower().strip()
    key_stem = step_path.stem.lower().strip()
    rules = {str(k).lower().strip(): v for k, v in settings.SPECIAL_COMPONENT_RULES.items()}

    rule = rules.get(key_file) or rules.get(key_stem)
    if rule is not None:
        return (
            str(rule.get("role", "obstacle")),
            float(rule.get("clearance", settings.DEFAULT_OBSTACLE_CLEARANCE)),
            bool(rule.get("solid", settings.DEFAULT_COMPONENT_SOLID)),
        )
    if "ground" in key_stem or "floor" in key_stem:
        return "ground", float(settings.DEFAULT_GROUND_CLEARANCE), True
    return "obstacle", float(settings.DEFAULT_OBSTACLE_CLEARANCE), bool(settings.DEFAULT_COMPONENT_SOLID)


def load_components(settings, with_shape: bool = True, with_mesh: bool = True) -> tuple[list[StepComponent], list[StepComponent]]:
    components: list[StepComponent] = []
    ignored: list[StepComponent] = []
    step_files = scan_cad_files(settings.BASE_STEP_DIR)
    print(f"[CAD] scan: {len(step_files)} files in {settings.BASE_STEP_DIR}")

    for index, path in enumerate(step_files, start=1):
        role, clearance, solid = component_rule(path, settings)
        comp = StepComponent(path.stem, str(path), role, clearance, solid)
        print(
            f"[CAD] {index}/{len(step_files)} {path.name}: "
            f"role={role}, clearance={clearance:.1f}, solid={solid}"
        )
        if role == "ignore":
            print(f"[CAD] {path.name}: ignored")
            ignored.append(comp)
            continue

        if with_shape or with_mesh:
            print(f"[CAD] {path.name}: read start", flush=True)
            comp.shape = read_cad_shape(path)
            print(f"[CAD] {path.name}: read done", flush=True)
        if with_mesh:
            print(f"[MESH] {path.name}: mesh start", flush=True)
            comp.mesh = triangulate_shape_to_mesh(
                comp.shape,
                settings.MESH_LINEAR_DEFLECTION,
                settings.MESH_ANGULAR_DEFLECTION,
                None,
            )
            tri_count = int(comp.mesh.get("triangle_count", 0)) if comp.mesh else 0
            print(f"[MESH] {path.name}: mesh done, triangles={tri_count}", flush=True)
        components.append(comp)

    return components, ignored
