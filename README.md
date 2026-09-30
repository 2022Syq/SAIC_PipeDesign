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
