import copy
import unittest
from unittest.mock import patch

import numpy as np

import main3


class EngineeringGeometryTests(unittest.TestCase):
    def setUp(self):
        self.guide = np.asarray([
            [0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [100.0, 20.0, 0.0],
            [200.0, 20.0, 0.0], [270.0, 0.0, 0.0], [300.0, 0.0, 0.0],
        ])
        self.direction = [1.0, 0.0, 0.0]

    def build(self, guide=None, max_arc_angle=90):
        return main3.build_line_arc_route(
            self.guide if guide is None else guide, 28.575, max_arc_angle, 4, 90,
        )

    def geometry(self, segments):
        return main3.validate_engineered_geometry(
            segments, self.guide[0], self.guide[-1], self.direction, self.direction,
        )

    def test_endpoint_corners_are_filleted_and_tangent_continuous(self):
        # 原实现跳过两端锚点，各遗留 15.9454 度的直线尖角。
        route = self.build()
        report = self.geometry(route["segments"])
        self.assertEqual(route["turn_count"], 4)
        self.assertTrue(report["valid"], report)
        self.assertLess(report["max_joint_tangent_error_deg"], 1e-8)
        self.assertLess(report["max_joint_gap_mm"], 1e-8)
        self.assertAlmostEqual(report["start_position_error_mm"], 0)
        self.assertAlmostEqual(report["goal_position_error_mm"], 0)
        self.assertLess(report["start_straight_length_mm"], 30)
        constraints = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertTrue(constraints["valid"], constraints)

    def test_old_sharp_polyline_is_rejected(self):
        segments = [
            {"type": "line", "start": a.tolist(), "end": b.tolist()}
            for a, b in zip(self.guide[:-1], self.guide[1:])
        ]
        report = self.geometry(segments)
        self.assertFalse(report["valid"])
        self.assertAlmostEqual(report["max_joint_tangent_error_deg"], 15.9453959009)
        self.assertTrue(all(item["rule"] == "joint_tangent" for item in report["violations"]))

    def test_joint_gap_and_changed_endpoint_are_rejected(self):
        segments = copy.deepcopy(self.build()["segments"])
        segments[1]["start"][2] += 0.1
        segments[0]["start"][2] += 0.2
        report = self.geometry(segments)
        rules = {item["rule"] for item in report["violations"]}
        self.assertFalse(report["valid"])
        self.assertIn("joint_gap", rules)
        self.assertIn("endpoint_position", rules)
        self.assertIn("endpoint_direction", rules)

    def test_small_turn_merge_keeps_prescribed_endpoint_directions(self):
        guide = self.guide.copy()
        guide[2:4, 1] = 1
        reduced = main3.merge_small_turns(guide, 5)
        np.testing.assert_array_equal(reduced[1], guide[1])
        np.testing.assert_array_equal(reduced[-2], guide[-2])
        report = self.geometry(self.build(reduced)["segments"])
        self.assertTrue(report["valid"], report)

    def test_split_arcs_preserve_exact_tangent_continuity(self):
        route = self.build(max_arc_angle=10)
        self.assertGreater(route["arc_count"], route["turn_count"])
        self.assertTrue(self.geometry(route["segments"])["valid"])

    def test_actual_bends_below_five_degrees_are_rejected(self):
        guide = self.guide.copy()
        guide[2:4, 1] = 3
        route = self.build(guide)
        report = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertFalse(report["valid"])
        self.assertIn("bend_angle", {item["rule"] for item in report["hard_failures"]})
        self.assertLess(report["minimum_actual_bend_angle_deg"], 5)

    def test_split_arcs_are_one_bend_for_angle_and_spacing_acceptance(self):
        route = self.build(max_arc_angle=4)
        self.assertLess(max(s["angle_deg"] for s in route["segments"] if s["type"] == "arc"), 5)
        report = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertTrue(report["valid"], report)
        self.assertAlmostEqual(report["minimum_actual_bend_angle_deg"], 15.9453959009)

    def test_adjust_small_end_bends_keeps_endpoints_and_directions(self):
        guide = np.asarray([
            [0., 0, 0], [30., 0, 0], [80., 3, 0],
            [320., 3, 0], [370., 0, 0], [400., 0, 0],
        ])
        corrected = main3.adjust_endpoint_small_turns(guide, 5)
        np.testing.assert_array_equal(corrected[[0, -1]], guide[[0, -1]])
        route = self.build(corrected)
        geometry = main3.validate_engineered_geometry(
            route["segments"], guide[0], guide[-1], self.direction, self.direction,
        )
        constraints = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertTrue(geometry["valid"], geometry)
        self.assertTrue(constraints["valid"], constraints)
        self.assertAlmostEqual(constraints["minimum_actual_bend_angle_deg"], 5)
        self.assertGreater(geometry["start_straight_length_mm"], 30)
        self.assertGreater(geometry["goal_straight_length_mm"], 30)

    def test_bend_spacing_under_twenty_is_rejected_even_with_valid_radius(self):
        guide = np.asarray([[0., 0, 0], [100., 0, 0], [100., 70, 0], [200., 70, 0]])
        route = self.build(guide)
        report = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertAlmostEqual(report["minimum_arc_radius"], 28.575)
        self.assertFalse(report["valid"])
        self.assertIn("bend_spacing", {item["rule"] for item in report["hard_failures"]})

    def test_short_adjacent_bends_do_not_bypass_minimum_radius(self):
        guide = np.asarray([[0., 0, 0], [30., 0, 0], [30., 10, 0], [60., 10, 0]])
        route = self.build(guide)
        report = main3.validate_route_constraints(
            route["sampled_path"], route, main3.active_constraint_profile(),
        )
        self.assertFalse(report["valid"])
        self.assertIn("bend_radius", {item["rule"] for item in report["hard_failures"]})

    def test_engineer_route_exports_final_geometry_report(self):
        source = {
            "name": "endpoint_corner_regression", "path": self.guide.tolist(),
            "search": {"endpoint_tangency": {
                "applied": True,
                "start": {"curve_point_count": 3}, "goal": {"curve_point_count": 3},
            }},
        }
        params = {
            "constraint_profile": main3.active_constraint_profile(),
            "z_tolerance": 0, "simplify_tolerance": 25, "min_segment_length": 40,
            "s_bend_min_turn_deg": 110, "s_bend_max_span": 420, "s_bend_min_detour": 60,
            "wiggle_smooth_route_names": set(), "max_turn_count": 10,
            "bend_radius_step": 10, "max_bend_angle_deg": 90,
            "arc_sample_angle_deg": 4, "min_corner_angle_deg": 90, "tube_segments": 16,
        }
        with patch.multiple(
            main3.settings, START=self.guide[0].tolist(), GOAL=self.guide[-1].tolist(),
            START_DIR_POINT=[30., 0, 0], GOAL_DIR_POINT=[330., 0, 0],
        ):
            result = main3.engineer_route(source, "red", params)
        self.assertTrue(result["constraint_report"]["valid"])
        self.assertTrue(result["geometry_report"]["valid"])
        self.assertEqual(result["turn_count"], 4)
        self.assertIn("collision_report", result)
        self.assertIsNone(result["collision_report"])


if __name__ == "__main__":
    unittest.main()
