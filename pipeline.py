# -*- coding: utf-8 -*-
"""Run the complete CATIA-point -> planning -> routing workflow."""

from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path


PROJECT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Measure CATIA helper points, update settings.py, then run "
            "main1.py, main2.py, main3.py and main4.py in order."
        )
    )
    parser.add_argument("--body", default="point", help='CATIA point set. Default: "point".')
    parser.add_argument("--document", help="Open CATIA document name or unique name fragment.")
    parser.add_argument("--selection", action="store_true", help="Use the selected CATIA point container.")
    parser.add_argument("--start", default="START", help="CATIA START point name.")
    parser.add_argument("--goal", default="GOAL", help="CATIA GOAL point name.")
    parser.add_argument("--start-dir", default="START_DIR_POINT", help="CATIA start-direction point name.")
    parser.add_argument("--goal-dir", default="GOAL_DIR_POINT", help="CATIA goal-direction point name.")
    parser.add_argument("--route-index", type=int, default=1, help="1-based engineered route for the CATIA stage.")
    parser.add_argument("--catia-visible", action="store_true", help="Show CATIA during the final modeling stage.")
    parser.add_argument("--centerline-only", action="store_true", help="Create only the CATIA centerline in the final stage.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the stage commands without connecting to CATIA or running them.",
    )
    return parser.parse_args()


def runtime_environment() -> dict[str, str]:
    """Supply Conda DLL paths even when its python.exe was called directly."""
    env = os.environ.copy()
    env["PYTHONUNBUFFERED"] = "1"
    if os.name != "nt":
        return env

    prefix = Path(sys.prefix)
    candidates = [prefix, prefix / "Library" / "bin", prefix / "Scripts", prefix / "bin"]
    existing = [str(path) for path in candidates if path.exists()]
    old_path = env.get("PATH", "")
    env["PATH"] = os.pathsep.join(existing + ([old_path] if old_path else []))
    env.setdefault("CONDA_PREFIX", str(prefix))
    return env


def point_stage_command(args: argparse.Namespace) -> list[str]:
    command = [
        sys.executable,
        "-u",
        str(PROJECT_DIR / "main_point.py"),
        "--write-settings",
        "--settings-file",
        str(PROJECT_DIR / "settings.py"),
        "--body",
        args.body,
        "--start",
        args.start,
        "--goal",
        args.goal,
        "--start-dir",
        args.start_dir,
        "--goal-dir",
        args.goal_dir,
    ]
    if args.document:
        command.extend(["--document", args.document])
    if args.selection:
        command.append("--selection")
    return command


def pipeline_stages(args: argparse.Namespace) -> list[tuple[str, list[str]]]:
    python_script = lambda name: [sys.executable, "-u", str(PROJECT_DIR / name)]
    stages = [
        ("CATIA helper points -> settings.py", point_stage_command(args)),
        ("X/Y planning-space sections", python_script("main1.py")),
        ("Joint X/Y route search", python_script("main2.py")),
        ("Engineering route reconstruction", python_script("main3.py")),
    ]
    catia_command = python_script("main4.py") + ["--route-index", str(int(args.route_index))]
    if args.catia_visible:
        catia_command.append("--visible")
    if args.centerline_only:
        catia_command.append("--centerline-only")
    stages.append(("CATIA engineered route modeling", catia_command))
    return stages


def display_command(command: list[str]) -> str:
    return subprocess.list2cmdline(command)


def main() -> int:
    args = parse_args()
    stages = pipeline_stages(args)
    print("=== Exhaust Pipe Full Pipeline ===", flush=True)
    print(f"Python: {sys.executable}", flush=True)
    print(f"Project: {PROJECT_DIR}", flush=True)

    if args.dry_run:
        for index, (name, command) in enumerate(stages, start=1):
            print(f"[{index}/{len(stages)}] {name}")
            print(f"  {display_command(command)}")
        return 0

    env = runtime_environment()
    pipeline_start = time.monotonic()
    for index, (name, command) in enumerate(stages, start=1):
        stage_start = time.monotonic()
        print("", flush=True)
        print(f"=== [{index}/{len(stages)}] {name} ===", flush=True)
        print(f"[RUN] {display_command(command)}", flush=True)
        try:
            completed = subprocess.run(command, cwd=PROJECT_DIR, env=env, check=False)
        except KeyboardInterrupt:
            print("[PIPELINE] interrupted by user", file=sys.stderr, flush=True)
            return 130
        elapsed = time.monotonic() - stage_start
        if completed.returncode != 0:
            print(
                f"[PIPELINE] stage {index} failed with exit code "
                f"{completed.returncode}; remaining stages were not run.",
                file=sys.stderr,
                flush=True,
            )
            return int(completed.returncode)
        print(f"[PIPELINE] stage {index} completed in {elapsed:.1f}s", flush=True)

    elapsed = time.monotonic() - pipeline_start
    print("", flush=True)
    print(f"=== Pipeline completed successfully in {elapsed:.1f}s ===", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
