import pytest

from pipe_config import PipeInstance
from route_clearance import check_pipe_clearance, check_route_clearance, sample_route


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
