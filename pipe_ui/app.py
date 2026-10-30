"""本机工作台服务。后台任务串行执行，浏览器只显示工程师需要的阶段。"""
from __future__ import annotations

import argparse
import copy
from dataclasses import replace
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
import json
import mimetypes
import os
from pathlib import Path
import secrets
import shutil
import subprocess
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import threading
import time
import traceback
from urllib.parse import urlparse
import webbrowser

import settings
from pipe_config import RunConfig, load_run_config, merge_measured_points
from .engineering import (COVERS, SADDLES, config_payload, default_hose, with_specs,
                          parameter_rows, route_view, refresh_route, hose_entry,
                          edit_route, check_pairs)
from .models import ModelStore

ROOT = Path(__file__).resolve().parents[1]
STATIC = Path(__file__).parent / "static"
OUTPUT = ROOT / "outputs" / "ui"


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8-sig"))


def write_json(path, value):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix(".tmp")
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False), encoding="utf-8")
    temp.replace(path)


def stamp():
    return datetime.now().strftime("%Y%m%d_%H%M%S_%f")


def catia_sources():
    """Enumerate point containers without opening, activating or modifying documents."""
    from main_point import (import_catia, documents_list, document_part, item_name,
                            iter_child_containers, get_hybrid_shapes, collection_count,
                            collection_item, MULTI_POINT_PATTERN, discover_multi_route_point_names)
    caa = import_catia()()
    sources = []
    def visit(document, container, path):
        shapes = get_hybrid_shapes(container)
        names = [item_name(collection_item(shapes, i)) for i in range(1, collection_count(shapes)+1)] if shapes else []
        if any(MULTI_POINT_PATTERN.match(name) for name in names):
            groups = discover_multi_route_point_names(container)
            sources.append({"document": item_name(document), "body": path,
                            "pipe_ids": list(groups)})
        for child in iter_child_containers(container):
            visit(document, child, path+"/"+item_name(child) if path else item_name(child))
    for document in documents_list(caa):
        part = document_part(document)
        if part is not None:
            visit(document, part, "")
    if not sources:
        raise ValueError("打开的 CATIA 零件中没有完整的管路命名点组。请打开包含 START、GOAL 及方向点的零件。")
    return sources


def measure_sources(selected):
    from main_point import (import_catia, select_document, document_part, get_hybrid_body,
                            measure_multi_route_points, measure_named_guardian_points)
    if not selected or len({s["document"] for s in selected}) != 1:
        raise ValueError("请选择同一个 CATIA 零件内的管路点组")
    document = select_document(import_catia()(), selected[0]["document"])
    part = document_part(document)
    bodies = [get_hybrid_body(part, s["body"].replace("/", "\\")) for s in selected]
    spa = document.spa_workbench()
    return {"pipes": measure_multi_route_points(part, spa, bodies),
            "guardians": measure_named_guardian_points(part, spa, bodies),
            "body": [s["body"] for s in selected]}


class Workbench:
    def __init__(self, config_path=None, points_path=None, model_dir=None):
        OUTPUT.mkdir(parents=True, exist_ok=True)
        self.config = load_run_config(config_path, measured=points_path) if config_path else None
        if self.config:
            self.config = with_specs(self.config, {p.pipe_id: p.pipe_spec for p in self.config.pipes})
        self.model_dir = Path(model_dir or settings.BASE_STEP_DIR).resolve()
        self.models = ModelStore(self.model_dir, OUTPUT / "mesh_cache")
        self.models.start()
        self.routes = []
        self.space_path = None
        self.models_by_pipe = {}
        self.stage = "configure"
        self.revision = 0
        self.saved = None
        self.result_project = None
        self.point_source = Path(points_path or config_path).name if (points_path or config_path) else "尚未读取管路点"
        self.token = secrets.token_urlsafe(32)
        self.executor = ThreadPoolExecutor(max_workers=1)
        self.lock = threading.RLock()
        self.job = {"status": "idle", "message": "准备导入模型与管路点"}
        self.error = None
        self.planning_process = None

    def snapshot(self):
        with self.lock:
            pipes = list(self.config.pipes) if self.config else []
            lookup = {p.pipe_id: p for p in pipes}
            return {"token": self.token, "revision": self.revision, "stage": self.stage,
                    "model_name": self.model_dir.name,
                    "point_source": self.point_source,
                    "pipes": [{"pipe_id": p.pipe_id, "spec": p.pipe_spec,
                               "start": p.start, "goal": p.goal,
                               "start_dir": p.start_dir, "goal_dir": p.goal_dir,
                               "guardians": [dict(name=g.name, point=g.point) for g in p.required_hangers],
                               "engineering_parameters": dict(p.engineering_parameters),
                               "parameters": parameter_rows(p),
                               "spec_parameters": {key: parameter_rows(replace(p, pipe_spec=key)) for key in COVERS}} for p in pipes],
                    "specs": [{"id": key, "label": f"φ{settings.PIPE_SPECS[key]['outer_diameter']:g} · {settings.PIPE_SPECS[key]['material']}"} for key in COVERS],
                    # Navigation hides finished routes in step 1 without discarding them.
                    "routes": [route_view(r, lookup[r["pipe_id"]]) for r in self.routes] if self.stage == "edit" else [],
                    "has_result": bool(self.routes or self.result_project),
                    "saddles": SADDLES, "saved": self.saved,
                    "job": copy.deepcopy(self.job)}

    def submit(self, name, args):
        with self.lock:
            if self.job["status"] == "running":
                raise ValueError("当前操作尚未完成")
            if args.get("revision", self.revision) != self.revision:
                raise ValueError("页面数据已更新，请刷新后再操作")
            self.job = {"status": "running", "message": name, "action": name, "started_at": time.time()}
            self.executor.submit(self._execute, name, args)
        return {"accepted": True}

    def _execute(self, name, args):
        import pythoncom
        pythoncom.CoInitialize()
        try:
            result = getattr(self, "do_"+name)(args)
            with self.lock:
                self.revision += 1
                self.job = {"status": "done", "message": "操作完成", "result": result}
        except Exception as exc:
            log = OUTPUT / "last_error.log"
            log.write_text(traceback.format_exc(), encoding="utf-8")
            with self.lock:
                self.job = {"status": "error", "message": str(exc), "log": str(log)}
        finally:
            pythoncom.CoUninitialize()

    def do_catia_sources(self, args):
        return {"sources": catia_sources()}

    def do_import_catia(self, args):
        if self.stage != "configure":
            raise ValueError("规划后不能更换端点，请重新导入工程")
        points = measure_sources(args["sources"])
        ids = {p["pipe_id"] for p in points["pipes"]}
        if self.config and ids == {p.pipe_id for p in self.config.pipes}:
            config = merge_measured_points(self.config, points)
        else:
            if points["guardians"]:
                raise ValueError("新点组包含挂点但没有明确的管路归属配置。请先提供挂点绑定，软件不会按名字猜测。")
            config = RunConfig.from_dict({"pipes": [{**p, "pipe_spec": "metal_9",
                "pipe_segments": [default_hose("metal_9")]} for p in points["pipes"]]})
        write_json(OUTPUT / "measured_points.json", points)
        self.config = config
        self.point_source = " / ".join(dict.fromkeys(s["document"] for s in args["sources"]))
        self.routes = []
        self.space_path, self.models_by_pipe = None, {}
        return {"message": f"已识别 {len(config.pipes)} 根管路"}

    def do_choose_model(self, args):
        if self.stage != "configure":
            raise ValueError("规划后不能更换环境模型")
        path = self._choose("选择环境模型", [("环境模型", "*.igs *.iges *.step *.stp *.CATPart *.CATProduct")])
        if not path:
            return {"message": "已取消选择"}
        source = Path(path)
        folder = OUTPUT / "imports" / stamp()
        folder.mkdir(parents=True)
        if source.suffix.lower() in {".catpart", ".catproduct"}:
            from upload import import_catia_client, export_igs
            client = import_catia_client()
            catia = client.Dispatch("CATIA.Application")
            export_igs(catia, source, folder / (source.stem+".igs"), close_after_export=False)
        else:
            shutil.copy2(source, folder / source.name)
        self.models.close()
        self.model_dir = folder
        self.models = ModelStore(folder, OUTPUT / "mesh_cache")
        self.models.start()
        self.routes = []
        self.space_path, self.models_by_pipe = None, {}
        return {"message": f"已导入 {source.name}", "reset_models": True}

    @staticmethod
    def _choose(title, types):
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        root.attributes("-topmost", True)
        try:
            return filedialog.askopenfilename(title=title, filetypes=types, parent=root)
        finally:
            root.destroy()

    def do_open_project(self, args):
        path = args.get("path") or self._choose("打开保存的管路工程或规划结果", [("管路工程", "*.json")])
        if not path:
            return {"message": "已取消打开"}
        data = read_json(path)
        if "config" in data:
            config = RunConfig.from_dict(data["config"])
            model_dir = Path(data["model_dir"])
        else:
            # Existing engineered JSON belongs to the currently selected input
            # project; never infer a different set of endpoints from it.
            if not self.config:
                raise ValueError("请先读取对应管路点，或打开工作台保存的完整工程")
            config, model_dir = self.config, self.model_dir
        raw = data.get("routes", [])
        if {r.get("pipe_id") for r in raw} != {p.pipe_id for p in config.pipes}:
            raise ValueError("工程路线与已识别的管路编号不一致")
        if any(not r.get("segments") or r.get("diagnostic_only") for r in raw):
            raise ValueError("结果包含未生成工程曲线的诊断路线，不能作为完整编辑工程打开")
        space_path = data.get("space_path")
        models_by_pipe = self._load_collision_models(config, space_path)
        pipes = {p.pipe_id: p for p in config.pipes}
        updated = [refresh_route(r, pipes[r["pipe_id"]], hose_entry(r, pipes[r["pipe_id"]]),
                    models_by_pipe.get(r["pipe_id"])) for r in raw]
        if any(not r["geometry_report"]["valid"] or not r["protected_waypoint_geometry"]["valid"] for r in updated):
            raise ValueError("工程曲线与固定起终点、接口方向或挂点不一致，原工程已保留")
        check_pairs(updated, config)
        reset_models = model_dir != self.model_dir
        if reset_models:
            self.models.close()
            self.model_dir = model_dir
            self.models = ModelStore(model_dir, OUTPUT / "mesh_cache")
            self.models.start()
        self.config, self.routes, self.stage = config, updated, "edit"
        self.point_source = data.get("point_source", "已保存工程（未记录点来源）")
        self.space_path, self.models_by_pipe = space_path, models_by_pipe
        self.saved = str(Path(path).resolve().parent)
        self.result_project = str(Path(path).resolve())
        if "config" not in data:
            # Raw route JSON needs its matched inputs saved for later restoration.
            self.save_project(OUTPUT / "sessions" / ("opened_"+stamp()))
        return {"message": "已打开工程路线", "reset_models": reset_models}

    def _load_collision_models(self, config, space_path):
        if not space_path:
            return {}
        from main2 import DualSectionModel, section_polygons
        space = read_json(space_path)
        # Same sectional acceptance model as the planner. These are sampled
        # checks, never presented as a full native-CAD collision proof.
        sections = (section_polygons(space, "x", extra_clearance=0.),
                    section_polygons(space, "y", extra_clearance=0.))
        return {p.pipe_id: DualSectionModel(*sections, allow_hard=False,
            pipe_radius=p.planning_radius, required_clearance=p.required_clearance) for p in config.pipes}

    def do_configure(self, args):
        """Switch to input view while retaining the completed engineering result."""
        if self.stage == "edit" and self.routes:
            self.save_project(OUTPUT / "sessions" / ("before_replan_"+stamp()))
        self.stage = "configure"
        return {"message": "已返回模型与规格；已完成路线保留，可点击第 3 步继续微调与导出。"}

    def do_resume_result(self, args):
        # Importing new points/model invalidates the in-memory geometry, but the
        # last completed project still restores its own inputs and collision space.
        if self.routes:
            self.stage = "edit"
            return {"message": "已返回上次规划结果及其对应规格，无需重新规划。"}
        if not self.result_project:
            raise ValueError("尚无已完成的规划结果，请先生成规划")
        result = self.do_open_project({"path": self.result_project})
        result["message"] = "已恢复上次规划结果及其模型、端点和规格，无需重新规划。"
        return result

    def do_plan(self, args):
        if self.stage != "configure" or not self.config:
            raise ValueError("请先识别管路并选择规格")
        mesh = self.models.snapshot()
        if mesh["status"] != "done" or not mesh["items"] or any(i["status"] != "ready" for i in mesh["items"]):
            raise ValueError("环境模型尚未完整读取，请等待模型加载完成")
        config = with_specs(self.config, args["specs"], args.get("parameters"))
        reuse_space_json = None
        if self.space_path:
            candidate_space = Path(self.space_path)
            cache_manifest = candidate_space.with_name("planning_space.cache.json")
            if candidate_space.is_file() and cache_manifest.is_file():
                from multi_pipeline import planning_space_cache_key
                try:
                    cache_record = read_json(cache_manifest)
                    expected_key = planning_space_cache_key(self.model_dir, config.pipes)
                    if cache_record.get("cache_key") == expected_key:
                        reuse_space_json = candidate_space
                except (OSError, ValueError, KeyError):
                    # A stale or incomplete cache is safe to ignore; the worker
                    # will build a fresh planning space for this run.
                    reuse_space_json = None
        run_id = "ui_"+stamp()
        task_dir = OUTPUT / "jobs" / run_id
        write_json(task_dir / "config.json", config_payload(config))
        run_dir = OUTPUT / "runs" / run_id
        log_path = task_dir / "planning.log"
        with log_path.open("w", encoding="utf-8") as log:
            command = [sys.executable, "-m", "pipe_ui.plan_worker",
                "--config", str(task_dir / "config.json"), "--model-dir", str(self.model_dir),
                "--output-root", str(OUTPUT / "runs"), "--run-id", run_id]
            if reuse_space_json is not None:
                command.extend(["--reuse-space-json", str(reuse_space_json)])
            self.planning_process = subprocess.Popen(command,
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT,
                env={**os.environ, "PYTHONUTF8": "1", "PYTHONUNBUFFERED": "1"},
                creationflags=subprocess.CREATE_NO_WINDOW if os.name == "nt" else 0)
            while self.planning_process.poll() is None:
                self.update_planning_progress(run_dir / "manifest.json")
                time.sleep(1)
            code = self.planning_process.returncode
        self.planning_process = None
        if code:
            raise RuntimeError(f"规划未完成（退出码 {code}），已保留日志：{log_path}")
        with self.lock:
            self.job["progress"] = {"message": "正在校核并保存最终路线", "completed": 0, "total": len(config.pipes)}
        self.update_planning_progress(run_dir / "manifest.json")
        routes = read_json(run_dir / "routes_engineered.json")["routes"]
        if any(not r.get("segments") or r.get("diagnostic_only") for r in routes):
            raise ValueError(f"部分管路未生成可编辑工程曲线。结果已保留：{run_dir}。请检查失败原因。")
        space_path = str(run_dir / "planning_space.json")
        models_by_pipe = self._load_collision_models(config, space_path)
        lookup = {p.pipe_id: p for p in config.pipes}
        updated = [refresh_route(r, lookup[r["pipe_id"]], hose_entry(r, lookup[r["pipe_id"]]),
                    models_by_pipe[r["pipe_id"]]) for r in routes]
        check_pairs(updated, config)
        self.config, self.routes, self.stage = config, updated, "edit"
        self.space_path, self.models_by_pipe = space_path, models_by_pipe
        with self.lock:
            self.job["progress"] = {"message": "正在校核并保存最终路线", "completed": len(updated), "total": len(config.pipes)}
        self.save_project(OUTPUT / "sessions" / run_id)
        return {"message": "规划完成，已显示最终工程路线"}

    def update_planning_progress(self, manifest_path):
        """Show recorded planner progress; an incomplete file read is retried next poll."""
        try:
            manifest = read_json(manifest_path)
        except (FileNotFoundError, json.JSONDecodeError):
            return
        records = manifest.get("pipes", [])
        current = next((r["pipe_id"] for r in records if r["status"] == "running"), None)
        completed = sum(r["status"] == "completed" for r in records)
        message = f"正在规划 {current}" if current else ("正在汇总与校核路线" if records else "正在准备环境与规划空间")
        with self.lock:
            self.job["progress"] = {"message": message, "completed": completed,
                                    "total": manifest["pipe_count"]}

    def do_edit(self, args):
        if self.stage != "edit":
            raise ValueError("请先生成工程路线")
        pipe_id = args["pipe_id"]
        pipe = next(p for p in self.config.pipes if p.pipe_id == pipe_id)
        index = next(i for i,r in enumerate(self.routes) if r["pipe_id"] == pipe_id)
        updated = copy.deepcopy(self.routes)
        updated[index] = edit_route(updated[index], pipe, args["edit"], self.models_by_pipe.get(pipe_id))
        check_pairs(updated, self.config)
        # Commit only after all geometry operations finish. Failed edits cannot
        # leave a half-updated route, hose or stale other-pipe clearance report.
        self.routes = updated
        self.save_project(OUTPUT / "sessions" / ("edit_"+stamp()))
        return {"message": "修改已应用，约束警告已更新"}

    def save_project(self, folder):
        folder = Path(folder)
        project = {"config": config_payload(self.config), "model_dir": str(self.model_dir),
                   "point_source": self.point_source,
                   "space_path": self.space_path, "routes": self.routes,
                   "export_policy": "constraint_warnings_allowed"}
        write_json(folder / "ui_project.json", project)
        write_json(folder / "routes_engineered.json", {"routes": self.routes})
        self.saved = str(folder)
        self.result_project = str((folder / "ui_project.json").resolve())
        return folder

    def do_export(self, args):
        if not self.routes or any(not r.get("exportable") for r in self.routes):
            raise ValueError("存在缺少有效工程曲线的管路，不能生成 CATPart")
        from main4 import create_catia_part, safe_name
        folder = self.save_project(OUTPUT / "exports" / stamp())
        manifest = {"status": "running", "files": [], "warnings_allowed": True}
        try:
            for route in self.routes:
                output = folder / (safe_name(route["pipe_id"])+".CATPart")
                saved_path = create_catia_part(route, output, route["core_outer_radius"], True, False,
                                               allow_constraint_warnings=True)
                manifest["files"].append(str(saved_path))
            manifest["status"] = "completed"
        except Exception as exc:
            manifest.update(status="failed", error=str(exc))
            raise
        finally:
            write_json(folder / "export_manifest.json", manifest)
        return {"message": f"已导出 {len(manifest['files'])} 个 CATPart", "folder": str(folder)}

    def do_show_folder(self, args):
        if not self.saved:
            raise ValueError("尚无保存结果")
        os.startfile(self.saved)
        return {"message": "已打开结果目录"}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        if args and (str(args[0]).startswith("GET /api/job") or str(args[0]).startswith("GET /api/models")):
            return
        super().log_message(fmt, *args)

    def json(self, payload, status=200):
        data = json.dumps(payload, ensure_ascii=False, allow_nan=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        app = self.server.app
        path = urlparse(self.path).path
        try:
            if path == "/api/state":
                return self.json(app.snapshot())
            if path == "/api/job":
                return self.json(app.job)
            if path == "/api/models":
                return self.json(app.models.snapshot())
            if path.startswith("/api/model/"):
                file = app.models.mesh_path(path.rsplit("/", 1)[-1])
            else:
                file = STATIC / ("index.html" if path == "/" else path.lstrip("/"))
                if not file.resolve().is_relative_to(STATIC.resolve()):
                    file = None
            if file is None or not file.is_file():
                return self.json({"error": "文件不存在"}, 404)
            self.send_response(200)
            self.send_header("Content-Type", mimetypes.guess_type(str(file))[0] or "application/octet-stream")
            self.send_header("Content-Length", str(file.stat().st_size))
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            with file.open("rb") as stream:
                shutil.copyfileobj(stream, self.wfile)
        except (BrokenPipeError, ConnectionResetError):
            pass
        except Exception as exc:
            self.json({"error": str(exc)}, 500)

    def do_POST(self):
        app = self.server.app
        if self.headers.get("X-Workbench-Token") != app.token:
            return self.json({"error": "请求来源无效，请刷新工作台"}, 403)
        try:
            size = int(self.headers.get("Content-Length", "0"))
            if not 0 < size < 1024*1024:
                raise ValueError("请求大小无效")
            args = json.loads(self.rfile.read(size))
            action = urlparse(self.path).path.removeprefix("/api/")
            if action not in {"catia_sources", "import_catia", "choose_model", "open_project", "configure", "resume_result", "plan", "edit", "export", "show_folder"}:
                return self.json({"error": "未知操作"}, 404)
            self.json(app.submit(action, args), 202)
        except (ValueError, KeyError) as exc:
            self.json({"error": str(exc)}, 400)


def main():
    parser = argparse.ArgumentParser(description="空调管路建模工作台")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--config", type=Path)
    parser.add_argument("--points", type=Path)
    parser.add_argument("--model-dir", type=Path)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    # The existing workspace input is the initial project, not an inferred CAD
    # association. New input is selected through the workbench import controls.
    config = args.config or ROOT / "outputs" / "multi_pipe_config_20261010.json"
    points = args.points or (ROOT / "outputs" / "measured_points_20261010.json" if args.config is None else None)
    app = Workbench(config if config.is_file() else None, points if points and points.is_file() else None, args.model_dir)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    server.app = app
    url = f"http://127.0.0.1:{args.port}"
    print(f"空调管路工作台：{url}", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    finally:
        app.models.close()
        if app.planning_process is not None and app.planning_process.poll() is None:
            app.planning_process.terminate()
        app.executor.shutdown(wait=False, cancel_futures=True)
        server.server_close()


if __name__ == "__main__":
    main()
