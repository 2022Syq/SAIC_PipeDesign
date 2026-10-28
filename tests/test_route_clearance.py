import pytest

from pipe_config import PipeInstance
from route_clearance import (
    check_pipe_clearance,
    check_route_clearance,
    double_clamp_report,
    sample_route,
)


def test_sampling_adds_points_at_requested_spacing():
    sampled = sample_route([[0, 0, 0], [10, 0, 0]], step=3)
    assert len(sampled) == 5
    assert sampled[0] == [0.0, 0.0, 0.0]
    assert sampled[-1] == [10.0, 0.0, 0.0]


def test_swept_envelopes_pass_when_wall_gap_meets_requirement():
    report = check_route_clearance(
        [[0, 0, 0], [100, 0, 0]], 2,
        [[0, 30, 0], [100, 30, 0]], 3,
        required_clearance=20,
        sample_step=10,
    )
    assert report["centerline_min_distance"] == pytest.approx(30)
    assert report["minimum_clearance"] == pytest.approx(25)
    assert report["valid"] is True


def test_swept_envelopes_fail_when_wall_gap_is_too_small():
    report = check_route_clearance(
        [[0, 0, 0], [100, 0, 0]], 2,
        [[0, 8, 0], [100, 8, 0]], 3,
        required_clearance=5,
    )
    assert report["minimum_clearance"] == pytest.approx(3)
    assert report["valid"] is False


def test_zero_extra_gap_accepts_exactly_tangent_swept_envelopes():
    report = check_route_clearance(
        [[0, 0, 0], [100, 0, 0]], 2,
        [[0, 5, 0], [100, 5, 0]], 3,
        required_clearance=0,
    )
    assert report["minimum_clearance"] == pytest.approx(0)
    assert report["required_clearance"] == pytest.approx(0)
    assert report["valid"] is True


def test_pipe_clearance_uses_each_spec_radius_and_required_clearance():
    first = PipeInstance.from_dict({
        "pipe_id": "a", "start": [0, 0, 0], "goal": [100, 0, 0],
        "start_dir": [1, 0, 0], "goal_dir": [101, 0, 0],
        "pipe_spec": "metal_9", "required_clearance": 4,
    })
    second = PipeInstance.from_dict({
        "pipe_id": "b", "start": [0, 20, 0], "goal": [100, 20, 0],
        "start_dir": [1, 20, 0], "goal_dir": [101, 20, 0],
        "pipe_spec": "metal_12", "required_clearance": 2,
    })
    report = check_pipe_clearance(first, [first.start, first.goal], second, [second.start, second.goal])
    assert report["radius_a"] == pytest.approx(4.5)
    assert report["radius_b"] == pytest.approx(6)
    assert report["required_clearance"] == 4
    assert report["valid"] is True


def test_pipe_clearance_zero_extra_gap_still_blocks_swept_intersection():
    first = PipeInstance.from_dict({
        "pipe_id": "a", "start": [0, 0, 0], "goal": [100, 0, 0],
        "start_dir": [1, 0, 0], "goal_dir": [101, 0, 0],
        "pipe_spec": "metal_9", "required_clearance": 0,
    })
    second = PipeInstance.from_dict({
        "pipe_id": "b", "start": [0, 0, 0], "goal": [100, 0, 0],
        "start_dir": [1, 0, 0], "goal_dir": [101, 0, 0],
        "pipe_spec": "metal_12", "required_clearance": 0,
    })
    report = check_pipe_clearance(first, [first.start, first.goal], second, [second.start, second.goal])
    assert report["required_clearance"] == 0
    assert report["minimum_clearance"] == pytest.approx(-10.5)
    assert report["valid"] is False


def test_double_clamp_reports_spacing_and_perpendicular_tangents():
    report = double_clamp_report(
        [[0, 0, 0], [0, 100, 0]], [0, 50, 0], 7.5,
        [[20, 0, 0], [20, 100, 0]], [20, 50, 0], 7.5,
        tangent_tolerance_deg=1.0,
    )
    assert report["center_distance_mm"] == pytest.approx(20.0)
    assert report["minimum_clearance_mm"] == pytest.approx(5.0)
    assert report["pipe_a"]["perpendicular_deviation_deg"] == pytest.approx(0.0)
    assert report["pipe_b"]["perpendicular_deviation_deg"] == pytest.approx(0.0)
    assert report["valid"] is True


def test_double_clamp_rejects_diagonal_route_at_fixed_points():
    report = double_clamp_report(
        [[0, 0, 0], [50, 50, 0], [100, 100, 0]], [50, 50, 0], 7.5,
        [[20, 0, 0], [20, 100, 0]], [20, 50, 0], 7.5,
        tangent_tolerance_deg=5.0,
    )
    assert report["center_distance_mm"] == pytest.approx(30.0)
    assert report["pipe_a"]["perpendicular_deviation_deg"] > 5.0
    assert report["valid"] is False


def test_local_hose_envelopes_block_overlap_but_do_not_expand_distant_metal():
    def pipe(name, y):
        return PipeInstance(name, (0, y, 0), (100, y, 0), (10, y, 0),
                            (110, y, 0), 'metal_9')
    def route(y, lo, hi):
        return {'engineered_path': [[0, y, 0], [100, y, 0]],
                'component_segments': [
                    {'kind': 'aluminum', 'start_s': 0, 'end_s': lo, 'outer_radius': 4.5},
                    {'kind': 'hose', 'start_s': lo, 'end_s': hi, 'outer_radius': 7.5},
                    {'kind': 'aluminum', 'start_s': hi, 'end_s': 100, 'outer_radius': 4.5},
                ]}
    a, b = pipe('A', 0), pipe('B', 13)
    # 13 mm is enough for aluminum/hose, but not two adjacent hoses.
    overlap = check_pipe_clearance(a, route(0, 20, 40), b, route(13, 20, 40))
    assert overlap['minimum_clearance'] == pytest.approx(-2)
    assert not overlap['valid']
    moved = check_pipe_clearance(a, route(0, 20, 40), b, route(13, 70, 90))
    assert moved['minimum_clearance'] == pytest.approx(1)
    assert moved['valid']
