#!/usr/bin/env python3
"""Create the isolated UAV-and-goal Baylands world copy."""

from __future__ import annotations

import argparse
from pathlib import Path
import xml.etree.ElementTree as ET


HERE = Path(__file__).resolve().parent
PROJECT_ROOT = HERE.parent
DEFAULT_SOURCE = PROJECT_ROOT / "simulation/worlds/baylands_editable.world"
DEFAULT_OUTPUT = HERE / "worlds/baylands_uav_goal.world"
REMOVE = frozenset({"husky", "waypoint_1", "waypoint_2", "waypoint_3"})


def entity_name(element: ET.Element) -> str | None:
    return element.get("name") or element.findtext("name")


def prepare(source: Path, output: Path) -> list[str]:
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"world element is missing from {source}")

    removed: list[str] = []
    for child in list(world):
        name = entity_name(child)
        if name in REMOVE:
            world.remove(child)
            removed.append(name)

    goal_count = sum(
        entity_name(child) == "goal" for child in list(world)
    )
    if goal_count != 1:
        raise ValueError(
            f"expected exactly one goal in {source}, found {goal_count}"
        )
    missing = REMOVE.difference(removed)
    if missing:
        raise ValueError(
            "source world is missing expected removable entities: "
            + ", ".join(sorted(missing))
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="unicode", xml_declaration=True)
    return sorted(removed)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--out", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    removed = prepare(args.source.resolve(), args.out.resolve())
    print(f"[WORLD] {args.out.resolve()}")
    print(f"[REMOVED] {', '.join(removed)}")
    print("[KEPT] goal and all Baylands terrain/obstacles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
