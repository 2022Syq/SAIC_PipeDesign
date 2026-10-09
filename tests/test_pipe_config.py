import json

import pytest

import settings
from pipe_config import PipeInstance, RunConfig, load_run_config, spec_outer_radius


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
