# Migrated from the supplied hvwire pipe editor; uses this project's OCC readers.
"""Standalone editor runtime; maintained independently of automatic routing."""
from __future__ import annotations
"""Read-only CAD display conversion, isolated from the engineering process.

Binary layout: two little-endian uint32 counts, interleaved float32 XYZ/normal
vertices, then uint32 triangle indices. No face truncation or X-range clipping.
"""


import argparse


import hashlib


import json


import os


from pathlib import Path


import subprocess


import sys


import threading


import uuid


import numpy as np


def write_display_mesh(mesh, output):
    points = np.column_stack([mesh[k] for k in ("x", "y", "z")]).astype("<f4")
    faces = np.column_stack([mesh[k] for k in ("i", "j", "k")]).astype("<u4")
    if not len(points) or not len(faces) or not np.isfinite(points).all() or int(faces.max()) >= len(points):
        raise ValueError("数模没有有效三角面或索引越界")
    a, b, c = (points[faces[:, i]] for i in range(3))
    face_normals = np.cross(b - a, c - a)
    normals = np.zeros_like(points)
    for i in range(3):
        np.add.at(normals, faces[:, i], face_normals)
    normals /= np.maximum(np.linalg.norm(normals, axis=1, keepdims=True), 1e-20)
    vertices = np.column_stack([points, normals]).astype("<f4")
    output = Path(output)
    # The worker can outlive the service-side cache setup.  Recreate the
    # destination directory here so a transient cache cleanup cannot make
    # every CAD conversion fail with FileNotFoundError.
    output.parent.mkdir(parents=True, exist_ok=True)
    # Keep the temporary basename short: the cache path can already be deep
    # on Windows, where appending a UUID to the hash-based filename can pass
    # the traditional MAX_PATH limit.
    temporary = output.with_name("." + uuid.uuid4().hex + ".tmp")
    try:
        with temporary.open("wb") as stream:
            stream.write(np.asarray([len(points), len(faces)], dtype="<u4").tobytes())
            stream.write(vertices.tobytes())
            stream.write(faces.tobytes())
        temporary.replace(output)
    finally:
        temporary.unlink(missing_ok=True)
    return {"vertices": len(points), "triangles": len(faces), "bytes": output.stat().st_size,
            "bounds": [points.min(axis=0).tolist(), points.max(axis=0).tolist()]}


class ModelStore:
    def __init__(self, base_dir, cache_dir, linear=8., angular=.5):
        self.base_dir, self.cache_dir = Path(base_dir).resolve(), Path(cache_dir).resolve()
        self.linear, self.angular = float(linear), float(angular)
        self.items = []
        self.status, self.current = "idle", ""
        self.lock = threading.Lock()
        self.thread = None
        self.stop_event = threading.Event()
        self.process = None

    def snapshot(self):
        with self.lock:
            return {"status": self.status, "current": self.current, "base_dir": str(self.base_dir),
                    "items": [{k: v for k, v in item.items() if k != "path"} for item in self.items]}

    def mesh_path(self, identifier):
        with self.lock:
            return next((Path(i["path"]) for i in self.items
                         if i["id"] == identifier and i["status"] == "ready"), None)

    def start(self):
        if self.thread is not None:
            return
        self.thread = threading.Thread(target=self._load, daemon=True, name="editor-cad-loader")
        self.thread.start()

    def close(self):
        self.stop_event.set()
        with self.lock:
            process = self.process
        if process and process.poll() is None:
            process.terminate()
        if self.thread:
            self.thread.join(timeout=5)

    def _load(self):
        from occ_mesh import scan_step_files, component_rule
        import settings
        try:
            if not self.base_dir.is_dir():
                raise ValueError(f"数模目录不存在：{self.base_dir}")
            files = scan_step_files(self.base_dir)
            self.cache_dir.mkdir(parents=True, exist_ok=True)
            with self.lock:
                self.status = "loading"
                self.items = [{"id": str(i), "name": p.name, "role": component_rule(p, settings)[0],
                               "status": "pending"} for i, p in enumerate(files)]
            for item, source in zip(self.items, files):
                if self.stop_event.is_set():
                    return
                stat = source.stat()
                key = hashlib.sha256(json.dumps(["editor-mesh-v1", str(source.resolve()), stat.st_size,
                    stat.st_mtime_ns, self.linear, self.angular]).encode()).hexdigest()
                binary, meta = self.cache_dir / (key + ".bin"), self.cache_dir / (key + ".json")
                with self.lock:
                    self.current = source.name
                    item["status"] = "loading"
                try:
                    details = None
                    if meta.is_file() and binary.is_file():
                        try:
                            details = json.loads(meta.read_text(encoding="utf-8"))
                            if details["bytes"] != binary.stat().st_size:
                                details = None
                        except (ValueError, KeyError, OSError):
                            details = None
                    cached = details is not None
                    if not cached:
                        log_path = self.cache_dir / (key + ".log")
                        with log_path.open("w", encoding="utf-8") as log:
                            command = [sys.executable, '-m', 'pipe_ui.models', "--source", str(source),
                                       "--output", str(binary), "--linear", str(self.linear), "--angular", str(self.angular)]
                            with self.lock:
                                if self.stop_event.is_set():
                                    return
                                self.process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                    creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
                            code = self.process.wait()
                        if code != 0:
                            raise RuntimeError(f"CAD 转换失败（退出码 {code}），日志：{log_path}")
                        details = json.loads(meta.read_text(encoding="utf-8"))
                    with self.lock:
                        item.update(details, status="ready", path=str(binary), cached=cached,
                                    url="/api/model/" + item["id"])
                except Exception as exc:
                    with self.lock:
                        item.update(status="error", error=str(exc))
            with self.lock:
                self.status = "done"
                self.current = ""
        except Exception as exc:
            with self.lock:
                self.status, self.current = "error", str(exc)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--linear", type=float, default=8.)
    parser.add_argument("--angular", type=float, default=.5)
    args = parser.parse_args()
    from occ_mesh import read_cad_shape, triangulate_shape_to_mesh
    mesh = triangulate_shape_to_mesh(read_cad_shape(args.source), args.linear, args.angular, None)
    details = write_display_mesh(mesh, args.output)
    meta = args.output.with_suffix(".json")
    temporary = meta.with_name("." + uuid.uuid4().hex + ".tmp")
    temporary.write_text(json.dumps(details, allow_nan=False), encoding="utf-8")
    temporary.replace(meta)


if __name__ == "__main__":
    main()
