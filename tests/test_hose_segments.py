import math
import unittest

import main3
from hose_segments import build_hose_layout


SPECS = {
    "metal_9": {"kind": "metal", "outer_diameter": 9.0},
    "hose_8DE3": {"kind": "hose", "outer_diameter": 15.0, "minimum_bend_radius": 50.0},
}


class HoseSegmentTests(unittest.TestCase):
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

    def test_hose_bend_radius_is_warning_or_hard_failure_by_mode(self):
        arc = {
            "type": "arc", "center": [0.0, 0.0, 0.0], "radius": 40.0,
            "angle_deg": 180.0, "start": [40.0, 0.0, 0.0], "end": [-40.0, 0.0, 0.0],
        }
        segments = [arc]
        warning = build_hose_layout(
            125.663706, [{"core_spec": "metal_9", "cover_spec": "hose_8DE3"}], SPECS, segments,
        )
        self.assertTrue(warning["valid"])
        self.assertTrue(any(item["rule"] == "hose_bend_radius" for item in warning["warnings"]))
        hard = build_hose_layout(
            125.663706,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3", "bend_radius_mode": "hard"}],
            SPECS,
            segments,
        )
        self.assertFalse(hard["valid"])
        self.assertIn("hose_bend_radius", {item["rule"] for item in hard["hard_failures"]})

        manual = build_hose_layout(
            125.663706,
            [{"core_spec": "metal_9", "cover_spec": "hose_8DE3", "position_s_mm": 62.831853}],
            SPECS,
            segments,
        )
        hose = next(item for item in manual["components"] if item["kind"] == "hose")
        self.assertAlmostEqual((hose["start_s"] + hose["end_s"]) / 2.0, 62.831853)
        self.assertTrue(any(item["rule"] == "hose_bend_radius" for item in manual["warnings"]))

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
