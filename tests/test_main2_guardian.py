from types import SimpleNamespace

import main2


def test_plan_pipe_routes_continues_after_one_profile_runtime_failure(monkeypatch):
    class FakeModel:
        def __init__(self, *args, **kwargs):
            pass

    profiles = [
        main2.RouteProfile("blocked", "", False, 0.5),
        main2.RouteProfile("usable", "", False, 0.5),
    ]
    calls = []

    def fake_build(profile, *args, **kwargs):
        calls.append(profile.name)
        if profile.name == "blocked":
            raise RuntimeError("required guardian cannot be connected")
        return {
            "name": profile.name,
            "path": [[0.0, 0.0, 0.0], [100.0, 0.0, 0.0]],
            "logs": [],
            "cost": 1.0,
            "hard_bad_segments": 0,
            "soft_bad_segments": 0,
        }

    monkeypatch.setattr(main2, "section_polygons", lambda *args, **kwargs: [{"value": 0.0}])
    monkeypatch.setattr(main2, "DualSectionModel", FakeModel)
    monkeypatch.setattr(main2, "make_profiles", lambda: profiles)
    monkeypatch.setattr(main2, "build_joint_route", fake_build)

    pipe = SimpleNamespace(
        pipe_id="PIPE_01",
        start=(0.0, 0.0, 0.0),
        goal=(100.0, 0.0, 0.0),
        start_dir=(20.0, 0.0, 0.0),
        goal_dir=(120.0, 0.0, 0.0),
        pipe_spec="metal_9",
        outer_radius=4.5,
        required_hangers=(),
    )
    result = main2.plan_pipe_routes(
        {"start": [0.0, 0.0, 0.0], "goal": [100.0, 0.0, 0.0]},
        pipe,
    )

    assert calls == ["blocked", "usable"]
    assert [route["name"] for route in result["routes"]] == ["PIPE_01_usable"]
    assert result["profile_failures"] == [{
        "name": "blocked",
        "type": "RuntimeError",
        "reason": "required guardian cannot be connected",
    }]
