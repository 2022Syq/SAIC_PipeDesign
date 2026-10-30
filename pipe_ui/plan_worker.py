"""Run OCC/planning outside the HTTP process so native CAD reads cannot freeze it."""
from __future__ import annotations

import argparse
from pathlib import Path

import settings
from multi_pipeline import run_multi_pipe


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--model-dir", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, required=True)
    parser.add_argument("--run-id", required=True)
    parser.add_argument("--reuse-space-json", type=Path)
    args = parser.parse_args()
    settings.BASE_STEP_DIR = args.model_dir
    run_multi_pipe(args.config, output_root=args.output_root, run_id=args.run_id,
                   skip_catia=True, write_html=False,
                   reuse_space_json=args.reuse_space_json)


if __name__ == "__main__":
    main()
