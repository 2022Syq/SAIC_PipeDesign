import math
import unittest
from types import SimpleNamespace

import numpy as np
from shapely.geometry import box

import main3
from hose_segments import build_hose_layout


SPECS = {
    "metal_9": {"kind": "metal", "outer_diameter": 9.0},
    "hose_8DE3": {"kind": "hose", "outer_diameter": 15.0, "minimum_bend_radius": 50.0},
}


class HoseSegmentTests(unittest.TestCase):
    def test_hose_slides_away_from_reserved_larger_component(self):
        # Static space is wide open; only another pipe makes the center invalid.
        model = SimpleNamespace(
            x_values=np.array([0.]), y_values=np.array([0.]),
            x_sections=[{'polys': [box(-1000, -1000, 1000, 1000)]}],
            y_sections=[{'polys': [box(-1000, -1000, 1000, 1000)]}],
            required_clearance=0., reserved_routes=[{
                'path': [[100., 13., 0.], [200., 13., 0.]], 'radius': 7.5,
            }],
        )
        path = np.array([[0., 0., 0.], [300., 0., 0.]])
        entry = {'core_spec': 'metal_9', 'cover_spec': 'hose_8DE3'}
        segments = [{'type': 'line', 'start': path[0].tolist(), 'end': path[1].tolist()}]
        automatic = main3._build_hose_layout_for_route(
            path, [entry], SPECS, segments, {'pipe_radius': 4.5}, model,
        )
        self.assertTrue(automatic['valid'])
        self.assertTrue(automatic['local_clearance']['valid'])
        self.assertTrue(any(w['rule'] == 'hose_position_auto_shifted' for w in automatic['warnings']))
        manual = main3._build_hose_layout_for_route(
            path, [{**entry, 'position_ratio': .5}], SPECS, segments,
            {'pipe_radius': 4.5}, model,
        )
        self.assertFalse(manual['valid'])
        self.assertTrue(any(w['rule'] == 'hose_local_clearance' for w in manual['hard_failures']))

    def test_component_boundaries_clip_original_line_and_arc_geometry(self):
        segments = [
            {"type": "line", "start": [0, 0, 0], "end": [100, 0, 0]},
            {
                "type": "arc", "center": [100, 50, 0], "radius": 50.0,
                "angle_deg": 90.0, "start": [100, 0, 0], "end": [150, 50, 0],
            },
        ]
        clipped = main3.clip_engineered_segments(segments, 80.0, 130.0)
        self.assertEqual([item["type"] for item in clipped], ["line", "arc"])
        self.assertAlmostEqual(clipped[0]["start"][0], 80.0)
        self.assertAlmostEqual(clipped[0]["end"][0], 100.0)
        self.assertAlmostEqual(clipped[1]["angle_deg"], math.degrees(30.0 / 50.0))

    def test_default_center_layout_uses_30_mm_hose_and_two_30_mm_sleeves(self):
        layout = build_hose_layout(
            300.0,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}],
            SPECS,
            [{"type": "line", "start": [0, 0, 0], "end": [300, 0, 0]}],
        )
        self.assertTrue(layout["valid"], layout)
        self.assertEqual([item["kind"] for item in layout["components"]], [
            "aluminum", "sleeve", "hose", "sleeve", "aluminum",
        ])
        hose = next(item for item in layout["components"] if item["kind"] == "hose")
        self.assertEqual(hose["length_mm"], 30.0)
        self.assertAlmostEqual(hose["start_s"], 135.0)
        self.assertAlmostEqual(hose["end_s"], 165.0)
        self.assertTrue(all(item["outer_radius"] == 7.5 for item in layout["components"] if item["kind"] == "sleeve"))

    def test_75_mm_option_and_manual_position_are_preserved(self):
        layout = build_hose_layout(
            300.0,
            [{
                "core_spec": "metal_9", "cover_spec": "hose_8DE3",
                "min_length_mm": 75.0, "hose_length_mm": 80.0,
                "sleeve_length_mm": 30.0, "position_s_mm": 180.0,
            }],
            SPECS,
            [{"type": "line", "start": [0, 0, 0], "end": [300, 0, 0]}],
        )
        hose = next(item for item in layout["components"] if item["kind"] == "hose")
        self.assertAlmostEqual(hose["start_s"], 140.0)
        self.assertAlmostEqual(hose["end_s"], 220.0)
        self.assertEqual(hose["position_mode"], "manual_s")
        self.assertAlmostEqual(layout["position_s_mm"], 180.0)
        self.assertAlmostEqual(layout["position_ratio"], 0.6)

    def test_hose_bend_radius_is_warning_only(self):
        better_arc = {
            "type": "arc", "center": [0.0, 0.0, 0.0], "radius": 49.0,
            "angle_deg": 90.0, "start": [49.0, 0.0, 0.0], "end": [0.0, 49.0, 0.0],
        }
        segments = [
            {"type": "arc", "center": [0.0, 0.0, 0.0], "radius": 40.0,
             "angle_deg": 90.0, "start": [40.0, 0.0, 0.0], "end": [0.0, 40.0, 0.0]},
            better_arc,
        ]
        warning = build_hose_layout(
            139.800, [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}], SPECS, segments,
        )
        self.assertTrue(warning["valid"])
        warning_item = next(item for item in warning["warnings"] if item["rule"] == "hose_bend_radius")
        self.assertEqual(warning_item["action"], "generate_anyway")
        self.assertAlmostEqual(warning_item["minimum_actual_radius_mm"], 40.0)
        hard = build_hose_layout(
            139.800,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}],
            SPECS,
            segments,
        )
        self.assertTrue(hard["valid"])
        self.assertIn("hose_bend_radius", {item["rule"] for item in hard["warnings"]})

        manual = build_hose_layout(
            139.800,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3", "position_s_mm": 62.831853}],
            SPECS,
            segments,
        )
        hose = next(item for item in manual["components"] if item["kind"] == "hose")
        self.assertAlmostEqual((hose["start_s"] + hose["end_s"]) / 2.0, 62.831853)
        self.assertTrue(any(item["rule"] == "hose_bend_radius" for item in manual["warnings"]))

    def test_total_span_over_route_falls_back_to_full_aluminum(self):
        layout = build_hose_layout(
            89.0,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}],
            SPECS,
        )
        self.assertTrue(layout["valid"])
        self.assertFalse(layout["enabled"])
        self.assertEqual(layout["fallback"], "all_aluminum")
        self.assertEqual(layout["fallback_reason"], "hose_total_span")
        warning = next(item for item in layout["warnings"] if item["rule"] == "hose_total_span")
        self.assertEqual(warning["action"], "fallback_to_full_aluminum")
        self.assertEqual(layout["components"], [])

    def test_mapping_and_minimums_are_rejected(self):
        with self.assertRaisesRegex(ValueError, "must use hose_8DE3"):
            build_hose_layout(
                300.0,
                [{"core_spec": "metal_9", "cover_spec": "hose_11DE3"}],
                {**SPECS, "hose_11DE3": {"kind": "hose", "outer_diameter": 19.0, "minimum_bend_radius": 60.0}},
            )
        with self.assertRaisesRegex(ValueError, "one of"):
            build_hose_layout(
                300.0,
                [{"core_spec": "metal_9", "cover_spec": "hose_8DE3", "min_length_mm": 40.0}],
                SPECS,
            )
        with self.assertRaisesRegex(ValueError, "sleeve_length_mm must be >= 1e-06"):
            build_hose_layout(
                300.0,
                [{"core_spec": "metal_9", "cover_spec": "hose_8DE3", "sleeve_length_mm": -1.0}],
                SPECS,
            )

    def test_engineering_route_emits_material_meshes_and_clipped_segments(self):
        source = {"name": "hose_demo", "path": [[0.0, 0.0, 0.0], [300.0, 0.0, 0.0]], "search": {}}
        pipe = {
            "pipe_id": "PIPE_HOSE",
            "pipe_spec": "metal_9",
            "outer_radius": 4.5,
            "start": [0.0, 0.0, 0.0],
            "goal": [300.0, 0.0, 0.0],
            "start_dir": [30.0, 0.0, 0.0],
            "goal_dir": [330.0, 0.0, 0.0],
            "pipe_segments": [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}],
        }
        route = main3.engineer_route(source, "#2563eb", main3.make_engineering_params(pipe))
        self.assertEqual(route["color"], "#c7b79d")
        self.assertEqual([item["kind"] for item in route["component_meshes"]], [
            "aluminum", "sleeve", "hose", "sleeve", "aluminum",
        ])
        self.assertTrue(route["constraint_report"]["valid"])
        self.assertTrue(all(item["segments"] for item in route["component_segments"]))


if __name__ == "__main__":
    unittest.main()
