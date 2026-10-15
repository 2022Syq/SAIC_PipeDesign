# Exhaust Pipe Route Generation

This project generates candidate exhaust pipe routes inside an automotive chassis package space.

The workflow has these stages:

1. `upload.py` converts uploaded CATIA Part files under `upload/` to IGES files under `base/`.
2. `main1.py` slices CAD chassis components and builds the planning space.
3. `main2.py` runs bidirectional route search and exports candidate guide paths.
4. `main3.py` converts guide paths into engineering-style pipe candidates made from straight lines and tangent arcs, then exports an interactive HTML viewer.
5. `main4.py` creates a CATIA CATPart from one engineered pipe route using line and arc centerline segments.
6. `main_point.py` measures four CATIA helper points for route endpoint settings.

`pipeline.py` runs `main_point.py`, `main1.py`, `main2.py`, `main3.py`, and
`main4.py` in that order. The measured CATIA helper points are written to
`settings.py` before planning starts. Use `--centerline-only` when the final
CATIA stage should create only the route centerline.

## Project Layout

- `upload/`: local uploaded CATIA Part files. This folder is ignored by Git.
- `base/`: local STEP/IGES input models. This folder is ignored by Git; place test models here before running, or generate IGES files with `upload.py`.
- `outputs/`: generated JSON, HTML, and images. This folder is ignored by Git.
- `settings.py`: project paths and route generation parameters.
- `common_space.py`: shared planning-space and HTML utilities.
- `occ_mesh.py`: STEP/IGES loading and mesh conversion helpers.
- `occ_section.py`: mesh slicing helpers.
- `main1.py`: planning-space generation.
- `main2.py`: bidirectional route search.
- `main3.py`: engineering pipe reconstruction and final HTML viewer.
- `main4.py`: CATIA CATPart export for a selected engineered route, preserving the line/arc structure from `main3.py`.
- `main_point.py`: reads helper points from a CATIA geometrical set and prints `settings.py` endpoint coordinates.
- `pipeline.py`: runs point measurement, planning-space generation, route search, engineering reconstruction, and CATIA modeling.
- `upload.py`: converts uploaded Part files to IGES files for `main1.py`.

## 本地文件维护规则

仓库把源码、测试和少量正式文档纳入 Git；CAD 输入、CATIA 模型、运行输出和工具缓存只保留在本机。`base/` 是 `occ_mesh.py` 读取的 IGES/STEP 输入，`model/` 是 CATIA 原始模型及装配引用，二者不能按普通临时文件清理，也不要直接改名或移动其中的装配件。

`outputs/<run_id>/` 按运行编号隔离每次多管路结果。运行结束后保留 `manifest.json`、路线/验证 JSON 和需要复盘的诊断文件；`planning_space_checkpoint.json` 只是阶段快照，不能断点续算，完成后可以删除。规划空间 JSON、交互式 HTML 和图片都能由当前代码重新生成，失败或中断的旧运行只保留小型清单与诊断，避免重复占用磁盘。唯一完成运行需要完整保留时，应以其 `manifest.json` 的 `status` 为依据。

当前本地运行结果采用两份代表性记录：保留 `outputs/run_20261001/` 作为旧单管基线，保留 `outputs/run_20261012_multi4_directguide_hard4_fix/` 作为最终多管路版本；其他 run 目录视为迭代产物，清理时可以整体删除。根目录的测点和多管配置 JSON 独立于 run 目录，按后续排障需要保留。

清理前先执行 `git status --short --branch`，确认没有未提交源码；不要使用 `git clean -xdf`，因为它会连同本地 CAD、模型和运行结果一起删除。清理后使用 `python -m pytest -q`、`python -m py_compile` 和 `git diff --check` 验证代码边界。Git 提交只记录本规则和源码变化，不把本地二进制 CAD 或 `outputs/` 运行产物加入版本库。

## Environment

The project expects an environment with OpenCascade/pythonocc support.

Install the Python dependencies listed in `requirements.txt`. If using conda, install `pythonocc-core` from a conda channel that provides it.

Example run order:

```powershell
python upload.py
python main1.py
python main2.py
python main3.py
python main4.py --case1
```

Complete automatic workflow:

```powershell
python pipeline.py
```

Generated route files are written to `outputs/`.

## Air-conditioning pipe specifications

The active single-pipe specification is `settings.ACTIVE_PIPE_SPEC`. The
available IDs mirror the SAAA table in
[`doc/SAAA工艺要求-20260706.xlsx`](doc/SAAA工艺要求-20260706.xlsx):

| ID | Type | Outside diameter | Recommended / minimum bend radius | Recommended / minimum straight length |
|---|---|---:|---:|---:|
| `metal_9` | 3003 H14 metal | 9 mm | R15 / R15 | 13.5 / 9 mm |
| `metal_12` | 3003 H14 metal | 12 mm | R20 / R20 | 18 / 12 mm |
| `metal_16` | 3003 H14 metal | 16 mm | R25 / R25 | 20 / 16 mm |
| `metal_19_05` | 3103 H112 metal | 19.05 mm | R27.5 / R28.575 | 25 / 20 mm |
| `hose_8DE3` | 8DE3 hose | 15 mm | R50 / R50 | local cover metadata |
| `hose_11DE3` | 11DE3 hose | 19 mm | R60 / R60 | local cover metadata |
| `hose_15DE3` | 15DE3 hose | 24 mm | R75 / R75 | local cover metadata |

The planning envelope uses the largest selected outside radius. The current
active value is `metal_19_05`, so `PIPE_DIAMETER=19.05` mm and
`PIPE_RADIUS=9.525` mm. Hose rows remain available as specification metadata;
the current engineering flow requires a continuous metal core and only applies
hose or sleeve data to configured `PIPE_SEGMENTS`.

## Multi-pipe run

Create a JSON file with one object per pipe. `pipe_spec` must match an ID in
`settings.PIPE_SPECS`; `required_clearance` is the additional wall-to-wall gap
between two pipe envelopes. Set it to `0.0` when no extra inter-pipe gap is
required—the two outside radii are still included in the swept-envelope check.
The outside diameter is the full pipe width; the outside radius is half that
diameter and is the value used to build each swept envelope. A zero gap allows
exact tangency, while a negative signed gap means the pipe envelopes intersect
and the route fails.

```json
{
  "pipes": [
    {
      "pipe_id": "PIPE_01",
      "pipe_spec": "metal_9",
      "required_clearance": 0.0,
      "priority": 1,
      "required_hangers": ["GUADIAN_01#1"],
      "start": [0, 0, 0],
      "goal": [1000, 0, 0],
      "start_dir": [30, 0, 0],
      "goal_dir": [1030, 0, 0]
    },
    {
      "pipe_id": "PIPE_02",
      "pipe_spec": "metal_12",
      "required_clearance": 0.0,
      "priority": 2,
      "required_hangers": ["GUADIAN_02#1"],
      "start": [0, 120, 0],
      "goal": [1000, 120, 0],
      "start_dir": [30, 120, 0],
      "goal_dir": [1030, 120, 0]
    }
  ]
}
```

每条管路的 `required_hangers` 必须按该管路的 `START → GOAL` 方向排列；多个
挂点时继续沿这条方向列出。示例中的 `GUADIAN_01#1` 与 `GUADIAN_02#1`
分别属于两条管路。

When `required_hangers` names are measured from CATIA, use a point set such as
`Route_Point` and run:

```powershell
python main_point.py --body "Part1(Route_Point)" --multi-output outputs\measured_points.json --no-write-settings
python multi_pipeline.py config\multi_pipe.json --points outputs\measured_points.json --run-id run-001 --skip-catia
```

`GUADIAN_01#1` and `GUADIAN_02#1` are named CATIA points, not the solid bracket itself. A
multi-pipe required hanger is inserted into the guide route as a hard waypoint;
the legacy single-pipe `HANGER_POINTS` setting remains a distance report.

### Multi-pipe acceptance and diagnostic results

The multi-pipe runner keeps one route per pipe in the delivered result. Search
may try several local strategies internally, but a route is marked
`completed` only when all of these checks pass: no hard obstacle conflict, no
negative inter-pipe swept-envelope clearance, valid engineering geometry, and
no engineering fallback. `allow_hard_conflicts` is useful for a diagnostic
attempt when the obstacle model is uncertain; it never turns a hard-conflict
route into a completed route.

All extra clearances are currently `0 mm`: the pipe outside radius still
counts, so `required_clearance: 0.0` does not permit overlap. If a candidate
hits an already accepted pipe, the runner tries local backtracking and other
directions for the current pipe. The configured priority order stays fixed. If
the local search is exhausted, the result records the blocking reason and
asks for a possible reorder instead of changing the order automatically.

When strict planning or engineering fails, the HTML viewer still contains one
diagnostic route so the failure can be inspected. It is explicitly marked as
diagnostic/failed and is never sent to CATIA. A pipe with an endpoint, goal, or
hard hanger point already closer than the sum of the two outside radii is
reported as an input point conflict; it is not silently treated as a valid
exception.

The same failure details are kept in the existing `manifest.json` and route
JSON files: `blocking_reasons`, `channel_report`, `input_point_conflicts`, and
`reorder_recommendation`. The runner keeps the configured order and only asks
for a reorder after that order is exhausted.

Engineering routes use a constant outside diameter and circular bends with
the same required centerline bend radius. A turn whose adjacent straight
segments cannot accommodate that radius is rejected and retried through local
backtracking; a sharp polyline or a reduced-radius corner is diagnostic only.
When a narrow passage is the cause, the report includes the actual local
free-space width, the current pipe's required width, and the width deficit.
Pipe-to-pipe checks separately report the signed clearance against the sum of
both outside radii.

`upload.py` requires CATIA and `pywin32`. By default it scans `upload/` for
`.CATPart`, `.CATProduct`, `.part`, and `.prt` files, then exports `.igs` files
to `base/`. Existing `.igs` files are kept unless `--overwrite` is used.

`main4.py` requires CATIA and `pywin32`. Route selection uses `--case1` to `--case5`, or `--route-index` / `--route-name`.

`main_point.py` requires CATIA and `pycatia`. By default it reads the geometrical set named `point` from the active CATPart and updates the four active coordinate assignments in `settings.py`. Use `--no-write-settings` for print-only mode, `--documents` to list open CATIA documents, or `--document` to select a specific CATPart.

The `constraint-update` branch records its air-conditioning pipe constraint
changes in [doc/版本说明-constraint-update.md](doc/版本说明-constraint-update.md).

## Notes

- CAD models under `upload/` and `base/` are local test inputs and are not committed to GitHub.
- `settings.BASE_STEP_DIR` is defined relative to the project folder, so moving the project folder should not break local paths.
- The final HTML viewer from `main3.py` has a left control panel and a right 3D model viewport.
