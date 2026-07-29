#!/usr/bin/env python3
"""Relocate absolute Gazebo model URIs after cloning EiraX."""

from __future__ import annotations

import argparse
from pathlib import Path
import re


PROJECT_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_WORLD = PROJECT_ROOT / "simulation/worlds/baylands_editable.world"
MODEL_URI_PATTERN = re.compile(r"file://[^<\r\n]*/simulation/models/")


def relocate(world: Path, project_root: Path) -> bool:
    original = world.read_text(encoding="utf-8")
    model_prefix = f"file://{project_root.resolve()}/simulation/models/"
    updated = MODEL_URI_PATTERN.sub(model_prefix, original)
    if updated == original:
        return False
    world.write_text(updated, encoding="utf-8")
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", type=Path, default=DEFAULT_WORLD)
    parser.add_argument("--project-root", type=Path, default=PROJECT_ROOT)
    args = parser.parse_args()

    world = args.world.resolve()
    if not world.is_file():
        parser.error(f"world file does not exist: {world}")

    changed = relocate(world, args.project_root)
    state = "updated" if changed else "already correct"
    print(f"{world}: {state}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
