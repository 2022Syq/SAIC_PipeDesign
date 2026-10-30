# Exhaust Pipe Route Generation

This project generates candidate exhaust pipe routes inside an automotive chassis package space.

## 空调管路建模工作台

双击 `启动管路工作台.cmd`，在浏览器中完成模型与端点读取、规格选择、自动规划、
管路与胶管拖动微调，以及 CATPart 导出。使用方法和验证边界见 [工作台说明](pipe_ui/README.md)。

The workflow has these stages:

1. `upload.py` converts uploaded CATIA Part files under `upload/` to IGES files under `base/`.
2. `main1.py` slices CAD chassis components and builds the planning space.
3. `main2.py` runs bidirectional route search and exports candidate guide paths.
4. `main3.py` converts guide paths into engineering-style pipe candidates made from straight lines and tangent arcs, then exports an interactive HTML viewer.
5. `main4.py` creates a CATIA CATPart from one engineered pipe route using line and arc centerline segments.
6. `main_point.py` measures four CATIA helper points for route endpoint settings.

`multi_pipeline.py` is the canonical entry point for the current four-pipe workflow. It reads the measured-point JSON and multi-pipe configuration, keeps one accepted route per pipe, applies GUADIAN and double-clamp constraints, and writes the route JSON/HTML plus CATIA parts when enabled.

`pipeline.py` remains the legacy single-pipe staged workflow. It is kept for historical comparison and single-route debugging; do not use it for the current four-pipe deliverable. The measured CATIA helper points are written to `settings.py` before the legacy planning stages start. Use `--centerline-only` when the final CATIA stage should create only the route centerline.

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
- `multi_pipeline.py`: canonical four-pipe runner with one final route per pipe, GUADIAN/double-clamp acceptance, local hose/sleeve geometry, HTML, and CATIA export.
- `pipeline.py`: legacy single-pipe staged runner retained for historical comparison.
- `upload.py`: converts uploaded Part files to IGES files for `main1.py`.
- `JIAOGUAN_HISTORY.md`: 胶管、双夹具和失败原因的合并迭代记录。
- `PROJECT_CLEANUP_20261028.md`: 运行产物保留/删除清单和本次整理结果。

## 本地文件维护规则

仓库把源码、测试和少量正式文档纳入 Git；CAD 输入、CATIA 模型、运行输出和工具缓存只保留在本机。`base/` 是 `occ_mesh.py` 读取的 IGES/STEP 输入，`model/` 是 CATIA 原始模型及装配引用，二者不能按普通临时文件清理，也不要直接改名或移动其中的装配件。

`outputs/<run_id>/` 按运行编号隔离每次多管路结果。运行结束后保留 `manifest.json`、路线/验证 JSON 和需要复盘的诊断文件；`planning_space_checkpoint.json` 只是阶段快照，不能断点续算，完成后可以删除。规划空间 JSON、交互式 HTML 和图片都能由当前代码重新生成，失败或中断的旧运行只保留小型清单与诊断，避免重复占用磁盘。唯一完成运行需要完整保留时，应以其 `manifest.json` 的 `status` 为依据。

当前保留四份有明确用途的运行记录：`outputs/run_20261001/` 是旧单管基线，`outputs/run_20261012/` 是双夹具改动前的四管基线，`outputs/run_20261015_pipe3_final/` 是此前 `feat/pipe3` 的参考结果，`outputs/run_20261028_jiaoguan_doubleclamp_4pipes/` 是当前胶管/双夹具正式四管结果。其他失败、重复或中间运行按 `PROJECT_CLEANUP_20261028.md` 清理。根目录的测点、多管配置和双夹具紧凑审计 JSON独立于 run 目录，继续保留。

清理前先执行 `git status --short --branch`；不要使用 `git clean -xdf`，因为它会连同本地 CAD、模型和运行结果一起删除。清理后使用 `python -m pytest -q`、OCC Python 的 `py_compile` 和 `git diff --check` 验证代码边界。Git 提交只记录源码、测试和维护文档，不把本地二进制 CAD 或 `outputs/` 运行产物加入版本库。

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

| ID | Type | Outside diameter | SAAA value / enforced minimum bend radius | Recommended / minimum straight length |
|---|---|---:|---:|---:|
| `metal_9` | 3003 H14 metal | 9 mm | R15 / R15 | 13.5 / 9 mm |
| `metal_12` | 3003 H14 metal | 12 mm | R20 / R20 | 18 / 12 mm |
| `metal_16` | 3003 H14 metal | 16 mm | R25 / R25 | 20 / 16 mm |
| `metal_19_05` | 3103 H112 metal | 19.05 mm | R27.5 / R28.575 | 25 / 20 mm |
| `hose_8DE3` | 8DE3 hose | 15 mm | R50 / R50 | local cover metadata |
| `hose_11DE3` | 11DE3 hose | 19 mm | R60 / R60 | local cover metadata |
| `hose_15DE3` | 15DE3 hose | 24 mm | R75 / R75 | local cover metadata |

SAAA radii such as R15 and R20 are **minimum requirements**: the actual
centerline bend radius may equal or exceed them. For metal pipes the enforced
minimum is `max(SAAA minimum, 1.5 × outside diameter)`. In `PIPE_SPECS`,
`minimum_bend_radius` is the hard limit; `recommended_bend_radius` supplies a
search reference, raised to at least that limit. It is not a fixed result or
an upper bound. The selected radius must also fit the straight lengths,
endpoint tangents, hard hangers and clearance constraints.

For turns above 90°, `ENGINEERING_ADAPTIVE_BEND_RADIUS_STEP_MM=1.0` searches
larger radii within the estimated tangent room, trying larger candidates
first for each control path. One selected radius is used throughout each route.
The current `PIPE_03` result uses **R22 with a minimum of R15** (`metal_9`).
Read a route's `bend_radius` for its selected radius and
`requested_bend_radius` for its search reference; `segments[].radius` must
match the selected radius. The reference value need not match the result.

The shared planning envelope uses the continuous metal-core radius. The
current single-pipe active value is `metal_19_05`, so
`PIPE_DIAMETER=19.05` mm and `PIPE_RADIUS=9.525` mm. A route with local hose
segments keeps the hose/sleeve envelope on its assigned component intervals;
it does not enlarge every section of the metal-core centerline search.

## Local hose and aluminium sleeves

`settings.PIPE_SEGMENTS` (or a per-pipe `pipe_segments` array in a multi-pipe
JSON file) describes a local replacement of the metal tube:

```json
{
  "core_spec": "metal_9",
  "cover_spec": "hose_8DE3",
  "min_length_mm": 30.0,
  "hose_length_mm": 30.0,
  "sleeve_length_mm": 30.0
}
```

`metal_9` is 9 mm outside diameter and maps to `hose_8DE3` at 15 mm outside
diameter with SAAA minimum bend radius R50. `min_length_mm` is currently 30 or
75 mm; the actual hose length must be at least that value. Each end has one
30 mm aluminium sleeve by default, with the same 15 mm outside diameter as the
hose, so the occupied centreline span is
`hose_length_mm + 2 * sleeve_length_mm`. With no explicit position the span is
placed at the route centre. `position_s_mm` or `position_ratio` can provide a
manual hose centre later. The hose and sleeves follow the already planned route
geometry; no new arc is fitted from the hose endpoints. The R50 value is
checked against that existing geometry for a warning only, so a route is still
generated when the warning is present.

If `hose_length_mm + 2 * sleeve_length_mm` is longer than the route, the local
layout is disabled and the route is exported as one full aluminium tube. The
fallback and its reason are recorded in `hose_layout`.
Engineering HTML uses CATIA-like colours: light aluminium, red-brown hose and
ochre sleeves. CATPart export creates separate sweeps for these intervals when
`component_segments` are present; an empty `PIPE_SEGMENTS` keeps the historical
single aluminium sweep.

## Multi-pipe run

Create a JSON file with one object per pipe. `pipe_spec` must match an ID in
`settings.PIPE_SPECS`; `required_clearance` is the additional wall-to-wall gap
between two pipe envelopes. Set it to `0.0` when no extra inter-pipe gap is
required—the two outside radii are still included in the swept-envelope check.
The outside diameter is the full pipe width; the outside radius is half that
diameter and is the value used to build each swept envelope. A zero gap allows
exact tangency, while a negative signed gap means the pipe envelopes intersect
and the route fails.

### Double clamps

`double_clamps` explicitly identifies a two-pipe clamp. It links one required
GUADIAN point on each pipe. At both linked points, the local centerline tangent
must be perpendicular to the line joining the two measured points. The default
`tangent_tolerance_deg` is 5 degrees. The report also checks the fixed center
distance against the two local outside radii; it does not replace the ordinary
route-to-route swept-envelope check.

```json
{
  "double_clamps": [{
    "name": "DOUBLE_CLAMP_PIPE_01_PIPE_02",
    "pipe_a": "PIPE_01",
    "guardian_a": "GUADIAN_01#1",
    "pipe_b": "PIPE_02",
    "guardian_b": "GUADIAN_02#1",
    "tangent_tolerance_deg": 5.0
  }]
}
```

The two points are measured centers. For the current pair their distance is
19.861 mm. With two 15 mm local component envelopes, the center-distance
clearance is 19.861 - 7.5 - 7.5 = 4.861 mm. This number alone is insufficient
when the two routes cross the clamp at an oblique angle. For parallel local
axes with unit tangent `t` and point displacement `d`, the axis distance is
`sqrt(dot(d,d) - dot(d,t)**2)`. Only when `dot(d,t)=0` does it equal the
measured point distance. Curved routes still require a separate minimum-distance
check over the complete component intervals.

Planning checks the larger radii of already reserved hose/sleeve intervals.
Final pipe-to-pipe acceptance uses each material interval's actual radius,
instead of treating the entire route as either 9 mm or 15 mm. Automatic hose
placement also checks reserved pipes; an unresolved local collision is a hard
failure. R50 bend-radius warnings remain warnings. Both members of an invalid
double clamp are excluded from formal CATPart export.

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
one selected centerline bend radius per route. That selected radius must be
at least the SAAA minimum. A turn whose adjacent straight segments cannot
accommodate a candidate radius is rejected and retried with another candidate
or control path; a sharp polyline or a reduced-radius corner is diagnostic
only.
When a narrow passage is the cause, the report includes the actual local
free-space width, the current pipe's required width, and the width deficit.
Pipe-to-pipe checks separately report the signed clearance against the sum of
both outside radii.

`upload.py` requires CATIA and `pywin32`. By default it scans `upload/` for
`.CATPart`, `.CATProduct`, `.part`, and `.prt` files, then exports `.igs` files
to `base/`. Existing `.igs` files are kept unless `--overwrite` is used.

`main4.py` requires CATIA and `pywin32`. Route selection uses `--case1` to `--case5`, or `--route-index` / `--route-name`.

`main_point.py` requires CATIA and `pycatia`. By default it reads the geometrical set named `point` from the active CATPart and updates the four active coordinate assignments in `settings.py`. Use `--no-write-settings` for print-only mode, `--documents` to list open CATIA documents, or `--document` to select a specific CATPart.

The `feat/JIAOGUAN` branch records the current air-conditioning, hose, and clamp constraint changes in [JIAOGUAN_HISTORY.md](JIAOGUAN_HISTORY.md) and [doc/版本说明-JIAOGUAN.md](doc/版本说明-JIAOGUAN.md).

## Notes

- CAD models under `upload/` and `base/` are local test inputs and are not committed to GitHub.
- `settings.BASE_STEP_DIR` is defined relative to the project folder, so moving the project folder should not break local paths.
- The final HTML viewer from `main3.py` has a left control panel and a right 3D model viewport.
