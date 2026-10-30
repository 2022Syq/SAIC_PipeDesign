"""Editing invariants: fixed constraints, material intervals and truthful export gates."""
import copy
import json
import math
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from pipe_config import PipeInstance, RunConfig
from pipe_ui.engineering import (default_hose, refresh_route, edit_route, hose_entry,
                                 fixed_intervals, parameter_rows, check_pairs, with_specs)
from pipe_ui.geometry import RouteCurve
from main4 import create_catia_part


def straight_pipe(pipe_id="A", y=0, guardian=False):
    pipe = PipeInstance.from_dict(dict(pipe_id=pipe_id, pipe_spec="metal_9",
        start=[0,y,0], goal=[600,y,0], start_dir=[10,y,0], goal_dir=[610,y,0],
        required_hangers=[dict(name="GUADIAN_01#1", point=[300,y,0])] if guardian else [],
        pipe_segments=[default_hose("metal_9")]))
    segments = [dict(type="line", start=[0,y,0], end=[300,y,0]),
                dict(type="line", start=[300,y,0], end=[600,y,0])]
    route = dict(pipe_id=pipe_id, name=pipe_id, segments=segments)
    entry = {**default_hose("metal_9"), "position_s_mm": 450.}
    return pipe, refresh_route(route, pipe, entry)


class PipeUiTests(unittest.TestCase):
    def test_parameter_edits_are_saved_and_reach_engineering_profile(self):
        from main3 import make_engineering_params
        pipe, _ = straight_pipe()
        config = with_specs(RunConfig((pipe,)), {"A": "metal_9"}, {"A": {
            "metal_outer_diameter": 10,
            "metal_bend_radius_minimum": 18,
            "bend_angle_minimum": 8,
            "required_clearance": 2,
        }})
        edited = config.pipes[0]
        self.assertEqual(edited.planning_radius, 5)
        self.assertEqual(edited.required_clearance, 2)
        profile = make_engineering_params(edited)["constraint_profile"]
        self.assertEqual(profile["minimum_bend_radius"], 18)
        self.assertEqual(profile["bend_angle_minimum"], 8)
        self.assertTrue(any(row["actual_key"] == "metal_outer_diameter" for row in parameter_rows(edited)))

    def test_planning_progress_reports_only_recorded_completions(self):
        from pipe_ui.app import Workbench, write_json
        app = Workbench.__new__(Workbench)
        app.lock = threading.RLock(); app.job = {"status": "running"}
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "manifest.json"
            write_json(path, {"pipe_count": 4, "pipes": [
                {"pipe_id": "A", "status": "completed"},
                {"pipe_id": "B", "status": "diagnostic_failed"},
                {"pipe_id": "C", "status": "running"}]})
            app.update_planning_progress(path)
            self.assertEqual(app.job["progress"], {"message": "正在规划 C", "completed": 1, "total": 4})
            # The manifest can briefly be incomplete while the worker writes it.
            path.write_text('{"pipes":', encoding="utf-8")
            app.update_planning_progress(path)
            self.assertEqual(app.job["progress"]["completed"], 1)

    def test_step_navigation_preserves_completed_geometry_and_collision_context(self):
        from pipe_ui.app import Workbench, read_json
        pipe, route = straight_pipe(guardian=True)
        app = Workbench.__new__(Workbench)
        app.point_source = "original-points.json"
        app.config = RunConfig((pipe,)); app.routes = [route]; app.stage = "edit"
        app.space_path = "original-space.json"; app.models_by_pipe = {"A": "model"}
        app.model_dir = Path("original-model-dir"); app.saved = None
        app.result_project = None
        app.lock = threading.RLock(); app.token = "test"; app.revision = 0
        app.job = {"status": "idle"}
        original_config = app.config
        original_routes, original_models = app.routes, app.models_by_pipe
        with tempfile.TemporaryDirectory() as directory, patch("pipe_ui.app.OUTPUT", Path(directory)):
            app.do_configure({})
            saved = read_json(Path(app.saved) / "ui_project.json")
            self.assertEqual(saved["routes"], [route])
            self.assertEqual(saved["space_path"], "original-space.json")
            self.assertEqual(saved["config"]["pipes"][0]["required_hangers"][0]["name"], "GUADIAN_01#1")
            state = app.snapshot()
            self.assertEqual(state["stage"], "configure")
            self.assertEqual(state["routes"], [])
            self.assertTrue(state["has_result"])
            # Refresh/repeated step-1 selection must not overwrite the completed result.
            saved_path = app.result_project
            app.do_configure({})
            self.assertEqual(app.result_project, saved_path)
            with patch.object(app, "do_open_project", side_effect=AssertionError("unneeded reload")):
                app.do_resume_result({})
            state = app.snapshot()
            self.assertEqual(state["stage"], "edit")
            self.assertEqual(len(state["routes"]), 1)
        self.assertIs(app.config, original_config)
        self.assertEqual(app.model_dir, Path("original-model-dir"))
        self.assertIs(app.routes, original_routes)
        self.assertEqual(app.space_path, "original-space.json")
        self.assertIs(app.models_by_pipe, original_models)

    def test_new_endpoint_input_can_return_to_complete_original_project(self):
        from pipe_ui.app import Workbench
        pipe, route = straight_pipe()
        app = Workbench.__new__(Workbench)
        app.point_source = "original-points.json"
        app.config = RunConfig((pipe,)); app.routes = [route]; app.stage = "edit"
        app.space_path = "original-space.json"; app.models_by_pipe = {}
        app.model_dir = Path("original-model-dir"); app.saved = None
        app.result_project = None
        points = {"pipes": [dict(pipe_id="B",start=[0,50,0],goal=[600,50,0],
                  start_dir=[10,50,0],goal_dir=[610,50,0])], "guardians": []}
        with tempfile.TemporaryDirectory() as directory, patch("pipe_ui.app.OUTPUT", Path(directory)):
            app.do_configure({})
            saved_path = app.result_project
            with patch("pipe_ui.app.measure_sources", return_value=points):
                app.do_import_catia({"sources": [{"document": "new-points.CATPart"}]})
            self.assertEqual(app.config.pipes[0].pipe_id, "B")
            self.assertEqual(app.point_source, "new-points.CATPart")
            self.assertEqual(app.routes, [])
            self.assertEqual(app.result_project, saved_path)
            with patch.object(app, "_load_collision_models", return_value={}):
                app.do_resume_result({})
            self.assertEqual(app.stage, "edit")
            self.assertEqual(app.config.pipes[0].pipe_id, "A")
            self.assertEqual(app.point_source, "original-points.json")
            self.assertEqual(app.space_path, "original-space.json")
            self.assertEqual(app.routes[0]["segments"], route["segments"])

    def test_step_three_is_unavailable_before_any_completed_result(self):
        from pipe_ui.app import Workbench
        app = Workbench.__new__(Workbench)
        app.point_source = "original-points.json"
        app.routes = []; app.result_project = None; app.stage = "configure"
        with self.assertRaisesRegex(ValueError, "尚无已完成"):
            app.do_resume_result({})
        self.assertEqual(app.stage, "configure")

    def test_failed_archive_keeps_current_editing_session(self):
        from pipe_ui.app import Workbench
        app = Workbench.__new__(Workbench)
        app.point_source = "original-points.json"
        app.routes = [{"pipe_id": "A"}]; app.stage = "edit"
        app.space_path = "original-space.json"; app.models_by_pipe = {"A": "model"}
        with patch.object(app, "save_project", side_effect=OSError("disk full")):
            with self.assertRaisesRegex(OSError, "disk full"):
                app.do_configure({})
        self.assertEqual(app.stage, "edit")
        self.assertEqual(app.routes, [{"pipe_id": "A"}])
        self.assertEqual(app.space_path, "original-space.json")
        self.assertEqual(app.models_by_pipe, {"A": "model"})

    def test_failed_open_keeps_existing_collision_space(self):
        from pipe_ui.app import Workbench
        from pipe_ui.engineering import config_payload
        pipe,route=straight_pipe()
        app=Workbench.__new__(Workbench)
        app.config=RunConfig((pipe,));app.routes=[route];app.stage="edit"
        app.space_path="original-space.json";app.models_by_pipe={"A":"original-model"}
        app.model_dir=Path("original-model-dir");app.saved="original-saved"
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/"ui_project.json"
            path.write_text(json.dumps(dict(config=config_payload(app.config),routes=[route],
                model_dir=str(app.model_dir),space_path="new-space.json")),encoding="utf-8")
            with patch.object(app,"_load_collision_models",return_value={"A":"new-model"}), \
                 patch("pipe_ui.app.refresh_route",side_effect=ValueError("invalid route")):
                with self.assertRaisesRegex(ValueError,"invalid route"):
                    app.do_open_project({"path":str(path)})
        self.assertEqual(app.space_path,"original-space.json")
        self.assertEqual(app.models_by_pipe,{"A":"original-model"})
        self.assertEqual(app.routes,[route])
        self.assertEqual(app.saved,"original-saved")

    def test_drag_changes_only_local_geometry_and_keeps_guardian_bridge(self):
        pipe, route = straight_pipe(guardian=True)
        changed = edit_route(route, pipe, dict(kind="route", station=130, delta=[0,12,0], influence=200))
        self.assertTrue(changed["geometry_report"]["valid"])
        self.assertTrue(changed["protected_waypoint_geometry"]["valid"])
        self.assertLess(RouteCurve(changed["segments"]).at(0)[0].dot([1,0,0]), 1e-6)
        self.assertEqual(changed["segments"][-1]["end"], [600,0,0])
        self.assertAlmostEqual(hose_entry(changed,pipe)["position_s_mm"],450)
        self.assertGreater(changed["engineered_length"],route["engineered_length"])
        self.assertEqual(fixed_intervals(route,pipe),[[285,315]])
        # The complete frozen interval remains on the same straight line.
        points=np.array(changed["engineered_path"])
        fixed=points[(points[:,0]>=285)&(points[:,0]<=600)]
        self.assertLess(np.max(np.abs(fixed[:,1:])),1e-8)

    def test_fixed_guardian_drag_is_rejected_without_mutating_route(self):
        pipe, route = straight_pipe(guardian=True)
        original=copy.deepcopy(route)
        with self.assertRaisesRegex(ValueError,"挂点"):
            edit_route(route,pipe,dict(kind="route",station=300,delta=[0,20,0],influence=200))
        self.assertEqual(route,original)

    def test_hose_resize_and_slide_preserves_sleeve_lengths(self):
        pipe,route=straight_pipe()
        changed=edit_route(route,pipe,dict(kind="hose",center=400,length=80,sleeve_length_mm=999))
        parts=[p for p in changed["component_segments"] if p["kind"]!="aluminum"]
        self.assertEqual([(p["kind"],p["start_s"],p["end_s"]) for p in parts],
                         [("sleeve",330,360),("hose",360,440),("sleeve",440,470)])
        self.assertAlmostEqual(sum(p["length_mm"] for p in changed["component_segments"]),600)

    def test_short_hose_is_warning_but_out_of_bounds_is_not_silently_changed(self):
        pipe,route=straight_pipe()
        changed=edit_route(route,pipe,dict(kind="hose",center=400,length=10))
        self.assertTrue(changed["exportable"])
        self.assertTrue(any(w["rule"]=="hose_length" for w in changed["constraint_report"]["hard_failures"]))
        for edit in (dict(kind="hose",center=5,length=30),dict(kind="hose",center=300,length=590)):
            with self.assertRaises(ValueError):edit_route(route,pipe,edit)

    def test_hose_only_bend_does_not_become_metal_bend(self):
        pipe=PipeInstance.from_dict(dict(pipe_id="A",pipe_spec="metal_9",start=[0,0,0],goal=[210,210,0],start_dir=[10,0,0],goal_dir=[210,220,0]))
        segments=[dict(type="line",start=[0,0,0],end=[200,0,0]),
                  dict(type="arc",start=[200,0,0],end=[210,10,0],center=[200,10,0],radius=10,angle_deg=90),
                  dict(type="line",start=[210,10,0],end=[210,210,0])]
        r=refresh_route(dict(pipe_id="A",segments=segments),pipe,{**default_hose("metal_9"),"position_s_mm":200+math.pi*2.5,"hose_length_mm":40})
        row=next(x for x in parameter_rows(pipe,r) if x["label"]=="金属最小弯曲半径")
        self.assertEqual(row["actual"],"直线段")
        rules={x["rule"] for x in r["constraint_report"]["hard_failures"]}
        self.assertNotIn("bend_radius",rules)
        self.assertTrue(any(x["rule"]=="hose_bend_radius" for x in r["constraint_report"]["warnings"]))

    def test_pair_status_recomputed_after_other_pipe_moves(self):
        a,ra=straight_pipe("A",0);b,rb=straight_pipe("B",3)
        ra["ui_environment_checked"]=rb["ui_environment_checked"]=True
        config=RunConfig((a,b));check_pairs([ra,rb],config)
        self.assertFalse(ra["formal_valid"])
        moved,new_rb=straight_pipe("B",50);new_rb["ui_environment_checked"]=True
        check_pairs([ra,new_rb],RunConfig((a,moved)))
        self.assertTrue(ra["formal_valid"])
        self.assertFalse(ra["inter_pipe_conflicts"])

    def test_export_warning_permission_does_not_forge_reports(self):
        pipe,r=straight_pipe()
        r["formal_valid"]=False;r["hard_conflicts"]=[{"component":"fixture"}]
        before=copy.deepcopy(r)
        with patch("main4.import_catia_client",side_effect=RuntimeError("COM boundary reached")):
            with self.assertRaises(ValueError):create_catia_part(r,Path("unused.CATPart"),4.5,False,False)
            with self.assertRaisesRegex(RuntimeError,"COM boundary reached"):
                create_catia_part(r,Path("unused.CATPart"),4.5,False,False,allow_constraint_warnings=True)
        self.assertEqual(r,before)
        r["geometry_report"]["valid"]=False
        with self.assertRaises(ValueError):
            create_catia_part(r,Path("unused.CATPart"),4.5,False,False,allow_constraint_warnings=True)


if __name__ == "__main__":
    unittest.main()
