from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace

import pytest

from multi_pipeline import (
    StageAdapterRequired,
    _channel_width_report,
    _point_conflict_reports,
    _double_clamp_reports,
    cli_main,
    planning_space_cache_key,
    run_pipeline,
)
from main4 import create_catia_part
from shapely.geometry import Polygon


@dataclass
class Pipe:
    pipe_id: str
    priority: int


@dataclass
class Config:
    pipes: list[Pipe]


def test_run_pipeline_orders_pipes_and_writes_isolated_artifacts(tmp_path):
    config = Config([Pipe("slow", 20), Pipe("fast", 1)])
    calls: list[tuple[str, str]] = []

    def planning(pipe, context):
        calls.append(("planning", pipe.pipe_id))
        return {"planned": pipe.pipe_id}

    def engineering(pipe, context):
        calls.append(("engineering", pipe.pipe_id))
        assert context.results["planning"] == {"planned": pipe.pipe_id}
        return {"engineered": pipe.pipe_id}

    def export(pipe, context):
        calls.append(("export", pipe.pipe_id))
        assert context.results["engineering"] == {"engineered": pipe.pipe_id}
        return {"exported": pipe.pipe_id}

    result = run_pipeline(
        config,
        planning=planning,
        engineering=engineering,
        export=export,
        output_root=tmp_path,
        run_id="test-run",
    )

    assert calls == [
        ("planning", "fast"),
        ("engineering", "fast"),
        ("export", "fast"),
        ("planning", "slow"),
        ("engineering", "slow"),
        ("export", "slow"),
    ]
    run_dir = tmp_path / "test-run"
    assert result["run_dir"] == run_dir.resolve()
    assert (run_dir / "fast" / "planning.json").exists()
    assert (run_dir / "slow" / "export.json").exists()
    assert json.loads((run_dir / "aggregate.json").read_text(encoding="utf-8"))["status"] == "completed"


def test_default_stage_explains_required_injection(tmp_path):
    with pytest.raises(StageAdapterRequired, match="injected per-pipe callable"):
        run_pipeline(
            Config([Pipe("one", 1)]),
            output_root=tmp_path,
            run_id="adapter-error",
        )

    manifest = json.loads((tmp_path / "adapter-error" / "manifest.json").read_text(encoding="utf-8"))
    assert manifest["status"] == "failed"
    assert manifest["error"]["type"] == "StageAdapterRequired"


def test_cli_passes_points_file_to_multi_run(monkeypatch, tmp_path):
    calls = []

    def fake_run(config, **kwargs):
        calls.append((config, kwargs))
        return {"run_dir": tmp_path / "run-001"}

    monkeypatch.setattr("multi_pipeline.run_multi_pipe", fake_run)
    assert cli_main(
        [
            "config.json",
            "--points",
            "measured.json",
            "--output-root",
            str(tmp_path),
            "--run-id",
            "run-001",
            "--skip-catia",
        ]
    ) == 0
    assert calls == [
        (
            Path("config.json"),
            {
                "points_path": Path("measured.json"),
                "output_root": tmp_path,
                "run_id": "run-001",
                "centerline_only": False,
                "catia_visible": False,
                "skip_catia": True,
                "reuse_space_json": None,
            },
        )
    ]


def test_point_conflicts_identify_input_measurements_before_routing():
    left = SimpleNamespace(
        pipe_id="A",
        outer_radius=5.0,
        start=(0.0, 0.0, 0.0),
        goal=(100.0, 0.0, 0.0),
        required_hangers=(),
    )
    right = SimpleNamespace(
        pipe_id="B",
        outer_radius=4.0,
        start=(8.0, 0.0, 0.0),
        goal=(100.0, 50.0, 0.0),
        required_hangers=(),
    )

    reports = _point_conflict_reports([left, right])

    assert reports[0]["type"] == "point_conflict"
    assert reports[0]["point_a"] == "A.start"
    assert reports[0]["point_b"] == "B.start"
    assert reports[0]["center_distance"] == pytest.approx(8.0)
    assert reports[0]["required_center_distance"] == pytest.approx(9.0)


def test_catia_export_blocks_diagnostic_route_before_connecting_to_catia(tmp_path):
    with pytest.raises(ValueError, match="diagnostic or non-exportable"):
        create_catia_part(
            {"name": "diagnostic", "diagnostic_only": True},
            tmp_path / "diagnostic.CATPart",
            4.0,
            False,
            False,
        )


def test_channel_report_exposes_available_required_and_gap_widths():
    model = SimpleNamespace(
        x_values=[0.0],
        y_values=[0.0],
        x_sections=[{"polys": [Polygon([(-10, -10), (10, -10), (10, 10), (-10, 10)])]}],
        y_sections=[{"polys": [Polygon([(-10, -10), (10, -10), (10, 10), (-10, 10)])]}],
        _nearest_index=lambda _values, _value: 0,
    )
    pipe = SimpleNamespace(outer_radius=4.0, required_clearance=0.0)

    report = _channel_width_report(model, [[0.0, 0.0, 0.0]], pipe)

    assert report["available_width_mm"] == pytest.approx(20.0)
    assert report["required_width_mm"] == pytest.approx(8.0)
    assert report["gap_mm"] == pytest.approx(12.0)


def test_planning_space_cache_key_tracks_cad_and_space_inputs(tmp_path):
    source = tmp_path / "body.igs"
    source.write_text("v1", encoding="utf-8")
    pipe = SimpleNamespace(
        pipe_id="A",
        start=[0.0, 0.0, 0.0],
        goal=[100.0, 0.0, 0.0],
        planning_radius=4.5,
    )

    first = planning_space_cache_key(tmp_path, [pipe])
    source.write_text("version-two", encoding="utf-8")
    changed_model = planning_space_cache_key(tmp_path, [pipe])
    moved_pipe = SimpleNamespace(**{**vars(pipe), "goal": [101.0, 0.0, 0.0]})
    changed_space = planning_space_cache_key(tmp_path, [moved_pipe])

    assert first != changed_model
    assert first != changed_space


def test_double_clamp_report_keeps_angle_details_and_reads_local_radii():
    from pipe_config import RunConfig
    pipes = [{
        'pipe_id': name, 'start': [0, y, 0], 'goal': [100, y, 0],
        'start_dir': [10, y, 0], 'goal_dir': [110, y, 0], 'pipe_spec': 'metal_9',
        'required_hangers': [{'name': 'G' + name, 'point': [50, y, 0]}],
        'pipe_segments': [{'cover_spec': 'hose_8DE3'}],
    } for name, y in [('A', 0), ('B', 20)]]
    config = RunConfig.from_dict({'pipes': pipes, 'double_clamps': [{
        'name': 'AB', 'pipe_a': 'A', 'pipe_b': 'B', 'guardian_a': 'GA', 'guardian_b': 'GB',
    }]})
    report = _double_clamp_reports(config, {p.pipe_id: p for p in config.pipes}, {
        'A': {'path': [[0, 0, 0], [100, 0, 0]]},
        'B': {'path': [[0, 20, 0], [100, 20, 0]]},
    })[0]
    assert report['valid']
    assert report['pipe_a']['perpendicular_deviation_deg'] == 0
    assert report['pipe_id_a'] == 'A'
    assert report['radius_a_mm'] == 4.5


def test_catia_rejects_invalid_double_clamp_before_com(tmp_path):
    with pytest.raises(ValueError, match='invalid double clamps'):
        create_catia_part({'double_clamp_reports': [{'valid': False}]},
                          tmp_path / 'blocked.CATPart', 4.5, False, False)
