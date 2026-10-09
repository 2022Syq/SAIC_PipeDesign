from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path

import pytest

from multi_pipeline import StageAdapterRequired, cli_main, run_pipeline


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
            },
        )
    ]
