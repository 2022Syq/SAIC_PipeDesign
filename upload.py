# -*- coding: utf-8 -*-
"""
Convert uploaded CATIA Part files to IGES files for the planning pipeline.

Default workflow:
    python upload.py

It scans ./upload for CATIA Part-like files and writes .igs files to ./base.
CATIA and pywin32 are required because native CATPart/CATProduct conversion is
handled through CATIA's ExportData API.
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import settings


DEFAULT_PART_EXTENSIONS = (".CATPart", ".CATProduct", ".catpart", ".catproduct", ".part", ".prt")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Convert uploaded Part files from upload/ to IGES files in base/."
    )
    parser.add_argument(
        "--input-dir",
        type=Path,
        default=Path(getattr(settings, "UPLOAD_DIR", settings.PROJECT_DIR / "upload")),
        help="Folder containing uploaded Part files. Default: ./upload.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(getattr(settings, "BASE_STEP_DIR", settings.PROJECT_DIR / "base")),
        help="Folder where .igs files are written. Default: ./base.",
    )
    parser.add_argument(
        "--overwrite",
        action="store_true",
        help="Overwrite existing .igs files in the output folder.",
    )
    parser.add_argument(
        "--visible",
        action="store_true",
        help="Show CATIA while converting.",
    )
    parser.add_argument(
        "--only",
        help="Convert only files whose name contains this text.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        help="Convert at most this many files after filtering.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="List conversion actions without opening CATIA.",
    )
    parser.add_argument(
        "--close-after-export",
        action="store_true",
        help="Close documents opened by this script after export. Disabled by default to avoid CATIA save prompts.",
    )
    parser.add_argument(
        "--per-file-timeout",
        type=int,
        default=600,
        help="Maximum seconds allowed for each single-file CATIA export. Default: 600.",
    )
    parser.add_argument("--_worker", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--_source", type=Path, help=argparse.SUPPRESS)
    parser.add_argument("--_target", type=Path, help=argparse.SUPPRESS)
    return parser.parse_args()


def safe_stem(value: str) -> str:
    stem = re.sub(r"[^0-9A-Za-z_\-\u4e00-\u9fff]+", "_", str(value)).strip("_")
    return stem or "part"


def scan_part_files(input_dir: Path) -> list[Path]:
    input_dir = Path(input_dir)
    if not input_dir.exists():
        raise FileNotFoundError(f"Upload folder does not exist: {input_dir}")
    if not input_dir.is_dir():
        raise NotADirectoryError(f"Upload path is not a folder: {input_dir}")

    suffixes = {suffix.lower() for suffix in DEFAULT_PART_EXTENSIONS}
    files = [
        path for path in input_dir.iterdir()
        if path.is_file()
        and not path.name.startswith("~$")
        and path.suffix.lower() in suffixes
    ]
    return sorted(files, key=lambda p: p.name.lower())


def output_path_for(source: Path, output_dir: Path, used: set[Path]) -> Path:
    base = output_dir / f"{safe_stem(source.stem)}.igs"
    candidate = base
    index = 2
    while candidate in used:
        candidate = output_dir / f"{safe_stem(source.stem)}_{index}.igs"
        index += 1
    used.add(candidate)
    return candidate


def import_catia_client():
    try:
        import win32com.client
        return win32com.client
    except Exception as exc:
        raise RuntimeError(
            "upload.py requires pywin32 and a local CATIA installation. "
            "Install pywin32 in the Python environment used to run this script."
        ) from exc


def close_document(doc) -> None:
    try:
        try:
            doc.Saved = True
        except Exception:
            pass
        doc.Close()
    except Exception:
        pass


def find_open_document(catia, source: Path):
    source_name = source.name.lower()
    try:
        count = int(catia.Documents.Count)
    except Exception:
        return None
    for index in range(1, count + 1):
        try:
            doc = catia.Documents.Item(index)
            if str(doc.Name).lower() == source_name:
                return doc
        except Exception:
            continue
    return None


def export_igs(catia, source: Path, target: Path, close_after_export: bool = False) -> None:
    doc = None
    opened_by_script = False
    try:
        doc = find_open_document(catia, source)
        if doc is None:
            print(f"[CATIA] open: {source}", flush=True)
            doc = catia.Documents.Open(str(source))
            opened_by_script = True
        else:
            print(f"[CATIA] reuse open document: {source.name}", flush=True)
        print(f"[CATIA] export IGES: {target}", flush=True)
        doc.ExportData(str(target), "igs")
    finally:
        if doc is not None and opened_by_script and close_after_export:
            close_document(doc)


def main() -> None:
    args = parse_args()
    if args._worker:
        if args._source is None or args._target is None:
            raise ValueError("worker mode requires --_source and --_target")
        win32 = import_catia_client()
        catia = win32.Dispatch("CATIA.Application")
        try:
            catia.Visible = bool(args.visible)
        except Exception:
            pass
        export_igs(
            catia,
            args._source.resolve(),
            args._target.resolve(),
            close_after_export=bool(args.close_after_export),
        )
        return

    input_dir = args.input_dir.resolve()
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    part_files = scan_part_files(input_dir)
    if args.only:
        needle = args.only.lower()
        part_files = [path for path in part_files if needle in path.name.lower()]
    if args.limit is not None:
        part_files = part_files[: max(0, int(args.limit))]
    print("=== Upload: Part -> IGES ===", flush=True)
    print(f"UPLOAD_DIR: {input_dir}", flush=True)
    print(f"BASE_STEP_DIR: {output_dir}", flush=True)
    print(f"part files: {len(part_files)}", flush=True)

    if not part_files:
        print("[DONE] no Part files found", flush=True)
        return

    used: set[Path] = set()
    jobs: list[tuple[Path, Path]] = []
    skipped: list[Path] = []
    for source in part_files:
        target = output_path_for(source, output_dir, used)
        if target.exists() and not args.overwrite:
            skipped.append(target)
            print(f"[SKIP] exists: {target.name} (use --overwrite to replace)", flush=True)
            continue
        jobs.append((source, target))
        print(f"[PLAN] {source.name} -> {target.name}", flush=True)

    if args.dry_run:
        print("[DONE] dry run only", flush=True)
        return
    if not jobs:
        print(f"[DONE] nothing to convert, skipped={len(skipped)}", flush=True)
        return

    converted = 0
    failed: list[tuple[Path, str]] = []
    for source, target in jobs:
        try:
            if target.exists() and args.overwrite:
                target.unlink()
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--_worker",
                "--_source",
                str(source),
                "--_target",
                str(target),
            ]
            if args.visible:
                command.append("--visible")
            if args.close_after_export:
                command.append("--close-after-export")
            subprocess.run(
                command,
                cwd=str(settings.PROJECT_DIR),
                check=True,
                timeout=max(1, int(args.per_file_timeout)),
            )
            converted += 1
            print(f"[OK] {source.name} -> {target.name}", flush=True)
        except subprocess.TimeoutExpired:
            failed.append((source, f"TimeoutExpired: exceeded {args.per_file_timeout}s"))
            print(f"[FAIL] {source.name}: exceeded {args.per_file_timeout}s", flush=True)
        except subprocess.CalledProcessError as exc:
            failed.append((source, f"CalledProcessError: exit={exc.returncode}"))
            print(f"[FAIL] {source.name}: worker exit={exc.returncode}", flush=True)
        except Exception as exc:
            failed.append((source, f"{type(exc).__name__}: {exc}"))
            print(f"[FAIL] {source.name}: {type(exc).__name__}: {exc}", flush=True)

    print(
        f"[DONE] converted={converted}, skipped={len(skipped)}, failed={len(failed)}, "
        f"output={output_dir}",
        flush=True,
    )
    if failed:
        raise RuntimeError(f"{len(failed)} file(s) failed to convert")


if __name__ == "__main__":
    main()
    
