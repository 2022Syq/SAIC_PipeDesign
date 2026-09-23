# -*- coding: utf-8 -*-
"""
三角网络通过occ_section.py得到X/Y截面线
"""
from __future__ import annotations

from collections import defaultdict


def _dedupe_points(points: list[list[float]], eps: float = 1e-6) -> list[list[float]]:
    unique: list[list[float]] = []
    for p in points:
        y, z = float(p[0]), float(p[1])
        if not any(abs(y - q[0]) <= eps and abs(z - q[1]) <= eps for q in unique):
            unique.append([y, z])
    return unique


def mesh_triangle_axis_plane_intersections(
    mesh: dict,
    plane: float,
    axis: str = "x",
    eps: float = 1e-7,
) -> list[list[list[float]]]:
    """Intersect a triangle mesh with an X or Y plane.

    The returned 2-D coordinates are Y/Z for an X plane and X/Z for a Y
    plane.  Keeping Z as the second coordinate lets the polygon code use the
    same implementation for both section families.
    """
    if not mesh:
        return []

    axis = str(axis).lower()
    if axis not in {"x", "y"}:
        raise ValueError(f"Unsupported section axis: {axis}")

    xs = mesh.get("x", [])
    ys = mesh.get("y", [])
    zs = mesh.get("z", [])
    ii = mesh.get("i", [])
    jj = mesh.get("j", [])
    kk = mesh.get("k", [])

    plane = float(plane)
    axis_index = 0 if axis == "x" else 1
    lateral_index = 1 if axis == "x" else 0
    segments: list[list[list[float]]] = []

    for ia, ib, ic in zip(ii, jj, kk):
        tri = [
            (float(xs[ia]), float(ys[ia]), float(zs[ia])),
            (float(xs[ib]), float(ys[ib]), float(zs[ib])),
            (float(xs[ic]), float(ys[ic]), float(zs[ic])),
        ]

        signed = [p[axis_index] - plane for p in tri]

        if all(abs(d) <= eps for d in signed):
            planar = [
                [tri[0][lateral_index], tri[0][2]],
                [tri[1][lateral_index], tri[1][2]],
                [tri[2][lateral_index], tri[2][2]],
            ]
            segments.append([planar[0], planar[1]])
            segments.append([planar[1], planar[2]])
            segments.append([planar[2], planar[0]])
            continue

        pts: list[list[float]] = []
        for p1, p2 in ((tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])):
            a1, a2 = p1[axis_index], p2[axis_index]
            lateral1, lateral2 = p1[lateral_index], p2[lateral_index]
            z1, z2 = p1[2], p2[2]
            d1 = a1 - plane
            d2 = a2 - plane

            if abs(d1) <= eps and abs(d2) <= eps:
                pts.append([float(lateral1), float(z1)])
                pts.append([float(lateral2), float(z2)])
            elif abs(d1) <= eps:
                pts.append([float(lateral1), float(z1)])
            elif abs(d2) <= eps:
                pts.append([float(lateral2), float(z2)])
            elif d1 * d2 < 0.0 and abs(a2 - a1) > eps:
                t = (plane - a1) / (a2 - a1)
                if -eps <= t <= 1.0 + eps:
                    lateral = lateral1 + t * (lateral2 - lateral1)
                    z = z1 + t * (z2 - z1)
                    pts.append([float(lateral), float(z)])

        unique = _dedupe_points(pts)
        if len(unique) >= 2:
            segments.append([unique[0], unique[1]])

    return segments


def mesh_triangle_plane_intersections(mesh: dict, x_plane: float, eps: float = 1e-7) -> list[list[list[float]]]:
    """Backward-compatible X-plane intersection entry point."""
    return mesh_triangle_axis_plane_intersections(mesh, x_plane, "x", eps)


def connect_segments_to_lines(segments: list[list[list[float]]], eps: float = 1e-4) -> list[list[list[float]]]:
    if not segments:
        return []

    scale = 1.0 / eps

    def key(p):
        return (round(float(p[0]) * scale), round(float(p[1]) * scale))

    points: dict[tuple[int, int], list[float]] = {}
    adjacency: dict[tuple[int, int], list[tuple[int, int]]] = defaultdict(list)
    edges: set[tuple[tuple[int, int], tuple[int, int]]] = set()

    for seg in segments:
        if len(seg) < 2:
            continue
        a, b = seg[0], seg[1]
        ka = key(a)
        kb = key(b)
        if ka == kb:
            continue
        points.setdefault(ka, [float(a[0]), float(a[1])])
        points.setdefault(kb, [float(b[0]), float(b[1])])
        edge = tuple(sorted((ka, kb)))
        if edge in edges:
            continue
        edges.add(edge)
        adjacency[ka].append(kb)
        adjacency[kb].append(ka)

    unused = set(edges)
    lines: list[list[list[float]]] = []

    while unused:
        start_edge = None
        for e in unused:
            a, b = e
            if len(adjacency[a]) == 1 or len(adjacency[b]) == 1:
                start_edge = e
                break
        if start_edge is None:
            start_edge = next(iter(unused))

        unused.remove(start_edge)
        a, b = start_edge
        line_keys = [a, b]

        extended = True
        while extended:
            extended = False
            head = line_keys[0]
            for nb in adjacency[head]:
                edge = tuple(sorted((head, nb)))
                if edge in unused:
                    unused.remove(edge)
                    line_keys.insert(0, nb)
                    extended = True
                    break
            tail = line_keys[-1]
            for nb in adjacency[tail]:
                edge = tuple(sorted((tail, nb)))
                if edge in unused:
                    unused.remove(edge)
                    line_keys.append(nb)
                    extended = True
                    break

        line = [points[k] for k in line_keys]
        if len(line) >= 2:
            y0, z0 = line[0]
            y1, z1 = line[-1]
            if ((y0 - y1) ** 2 + (z0 - z1) ** 2) ** 0.5 <= eps * 5:
                line[-1] = line[0]
            lines.append(line)

    return lines


def section_mesh_to_yz_lines(mesh: dict, x_plane: float) -> list[list[list[float]]]:
    segments = mesh_triangle_plane_intersections(mesh, x_plane)
    return connect_segments_to_lines(segments)


def section_mesh_to_xz_lines(mesh: dict, y_plane: float) -> list[list[list[float]]]:
    segments = mesh_triangle_axis_plane_intersections(mesh, y_plane, "y")
    return connect_segments_to_lines(segments)
