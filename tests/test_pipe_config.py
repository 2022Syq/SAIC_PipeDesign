import json

import pytest

import settings
from pipe_config import (
    PipeInstance,
    RunConfig,
    load_run_config,
    merge_measured_points,
    spec_outer_radius,
)


def pipe_data(pipe_id="pipe_01", priority=1, **overrides):
    value = {
        "pipe_id": pipe_id,
        "start": [0, 0, 0],
        "goal": [100, 0, 0],
        "start_dir": [20, 0, 0],
        "goal_dir": [120, 0, 0],
        "pipe_spec": "metal_19_05",
        "required_clearance": 20,
        "priority": priority,
    }
    value.update(overrides)
    return value


def test_pipe_radius_is_derived_from_settings_spec():
    pipe = PipeInstance.from_dict(pipe_data())
    expected = settings.PIPE_SPECS[pipe.pipe_spec]["outer_diameter"] / 2
    assert pipe.outer_radius == expected
    assert pipe.radius == expected
    assert spec_outer_radius(pipe.pipe_spec) == expected


def test_required_guardians_are_named_and_keep_coordinates():
    pipe = PipeInstance.from_dict(
        pipe_data(
            pipe_id="PIPE_01",
            pipe_spec="metal_9",
            required_clearance=0,
            required_hangers=[{"name": "GUADIAN_01#1", "point": [25, 1, 2]}],
        )
    )
    assert pipe.required_hangers[0].name == "GUADIAN_01#1"
    assert pipe.required_hangers[0].point == (25.0, 1.0, 2.0)


def test_pipe_segments_are_preserved_as_local_engineering_metadata():
    pipe = PipeInstance.from_dict(
        pipe_data(
            pipe_id="PIPE_HOSE",
            pipe_spec="metal_9",
            pipe_segments=[{
                "core_spec": "metal_9",
                "cover_spec": "hose_8DE3",
                "min_length_mm": 30.0,
                "sleeve_length_mm": 30.0,
            }],
        )
    )
    assert pipe.pipe_segments[0]["cover_spec"] == "hose_8DE3"
    assert pipe.outer_radius == 8.5


def test_pipe_segments_must_be_objects():
    with pytest.raises(ValueError, match="pipe_segments"):
        PipeInstance.from_dict(pipe_data(pipe_segments=["hose_8DE3"]))


def test_required_guardians_can_be_resolved_from_measured_points(tmp_path):
    config_path = tmp_path / "specs.json"
    config_path.write_text(
        json.dumps({"pipes": [{**pipe_data("PIPE_01", priority=1), "required_hangers": ["GUADIAN_01#1"]}]}),
        encoding="utf-8",
    )
    points_path = tmp_path / "points.json"
    points_path.write_text(
        json.dumps(
            {
                "pipes": [
                    {
                        "pipe_id": "PIPE_01",
                        "start": [0, 0, 0],
                        "goal": [100, 0, 0],
                        "start_dir": [20, 0, 0],
                        "goal_dir": [120, 0, 0],
                    }
                ],
                "guardians": [{"name": "GUADIAN_01#1", "point": [50, 0, 0]}],
            }
        ),
        encoding="utf-8",
    )
    config = load_run_config(config_path, measured=points_path)
    assert config.pipes[0].required_hangers[0].point == (50.0, 0.0, 0.0)


def test_run_config_validates_ids_and_preserves_priority_order():
    config = RunConfig.from_dict({"pipes": [pipe_data("late", 2), pipe_data("early", 1)]})
    assert [pipe.pipe_id for pipe in config.pipes] == ["late", "early"]
    assert [pipe.pipe_id for pipe in config.ordered_pipes()] == ["early", "late"]

    with pytest.raises(ValueError, match="unique"):
        RunConfig.from_dict({"pipes": [pipe_data("same"), pipe_data("same")]})


@pytest.mark.parametrize(
    "change, message",
    [
        ({"pipe_spec": "missing"}, "Unknown pipe spec"),
        ({"start": [0, 0]}, "exactly three"),
        ({"required_clearance": -1}, ">= 0"),
        ({"start_dir": [0, 0, 0]}, "start_dir"),
    ],
)
def test_pipe_entry_validation(change, message):
    with pytest.raises(ValueError, match=message):
        PipeInstance.from_dict(pipe_data(**change))


def test_load_run_config_reads_json(tmp_path):
    path = tmp_path / "run.json"
    path.write_text(json.dumps({"pipes": [pipe_data()]}), encoding="utf-8")
    config = load_run_config(path)
    assert isinstance(config.pipes[0], PipeInstance)
    assert config.pipes[0].pipe_id == "pipe_01"


def test_load_run_config_rejects_invalid_json(tmp_path):
    path = tmp_path / "run.json"
    path.write_text("{", encoding="utf-8")
    with pytest.raises(ValueError, match="invalid JSON"):
        load_run_config(path)


def test_merge_measured_points_replaces_only_coordinates(tmp_path):
    config = RunConfig.from_dict({"pipes": [pipe_data("pipe_01", priority=7)]})
    measured = {
        "body": "Part1(Route_Point)",
        "pipes": [
            {
                "pipe_id": "pipe_01",
                "start": [10, 20, 30],
                "goal": [110, 20, 30],
                "start_dir": [40, 20, 30],
                "goal_dir": [140, 20, 30],
                "point_names": {"start": "PIPE_01_START"},
            }
        ],
    }
    path = tmp_path / "measured.json"
    path.write_text(json.dumps(measured), encoding="utf-8")

    merged = merge_measured_points(config, path)
    pipe = merged.pipes[0]
    assert pipe.start == (10.0, 20.0, 30.0)
    assert pipe.goal_dir == (140.0, 20.0, 30.0)
    assert pipe.pipe_spec == "metal_19_05"
    assert pipe.required_clearance == 20.0
    assert pipe.priority == 7


def test_merge_measured_points_requires_exact_pipe_ids():
    config = RunConfig.from_dict({"pipes": [pipe_data("pipe_01")]})
    measured = {
        "pipes": [
            {
                "pipe_id": "PIPE_01",
                "start": [0, 0, 0],
                "goal": [100, 0, 0],
                "start_dir": [20, 0, 0],
                "goal_dir": [120, 0, 0],
            }
        ]
    }
    with pytest.raises(ValueError, match="do not match run config"):
        merge_measured_points(config, measured)


def test_load_run_config_can_fill_coordinate_free_spec_config(tmp_path):
    config_path = tmp_path / "specs.json"
    config_path.write_text(
        json.dumps(
            {
                "pipes": [
                    {
                        "pipe_id": "PIPE_01",
                        "pipe_spec": "metal_19_05",
                        "required_clearance": 25,
                        "priority": 2,
                    }
                ]
            }
        ),
        encoding="utf-8",
    )
    points_path = tmp_path / "points.json"
    points_path.write_text(
        json.dumps(
            {
                "pipes": [
                    {
                        "pipe_id": "PIPE_01",
                        "start": [0, 0, 0],
                        "goal": [100, 0, 0],
                        "start_dir": [20, 0, 0],
                        "goal_dir": [120, 0, 0],
                    }
                ]
            }
        ),
        encoding="utf-8",
    )

    config = load_run_config(config_path, measured=points_path)
    assert config.pipes[0].start == (0.0, 0.0, 0.0)
    assert config.pipes[0].pipe_spec == "metal_19_05"
    assert config.pipes[0].required_clearance == 25.0
    assert config.pipes[0].priority == 2
