#!/usr/bin/env python3
"""Save the SDF returned by Gazebo's generate_world_sdf service."""

from __future__ import annotations

import argparse
import ast
from pathlib import Path
import re
import subprocess


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--service", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    result = subprocess.run(
        [
            "gz", "service", "-s", args.service,
            "--reqtype", "gz.msgs.SdfGeneratorConfig",
            "--reptype", "gz.msgs.StringMsg",
            "--timeout", "10000",
            # Gazebo Harmonic's SdfGeneratorConfig uses nested optional
            # configuration.  An empty request preserves the running model
            # poses and avoids fields only available in other Gazebo versions.
            "--req", "",
        ],
        capture_output=True,
        text=True,
        timeout=15,
    )
    if result.returncode != 0:
        raise SystemExit(result.stderr.strip() or "Gazebo world-save service failed")

    match = re.search(r'data:\s*("(?:\\.|[^"\\])*")', result.stdout, re.DOTALL)
    if not match:
        raise SystemExit("Gazebo returned no world SDF")
    sdf = ast.literal_eval(match.group(1))
    if "<sdf" not in sdf or "<world" not in sdf:
        raise SystemExit("Gazebo returned an invalid world document")

    # Gazebo expands model:// includes to file:///absolute/developer/path when
    # generating the running world.  Restore portable model URIs before the
    # document becomes the next run's source world.
    models_root = args.output.resolve().parents[2] / "models"
    sdf = sdf.replace(f"file://{models_root}/", "model://")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    temporary = args.output.with_suffix(args.output.suffix + ".tmp")
    temporary.write_text(sdf, encoding="utf-8")
    temporary.replace(args.output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
