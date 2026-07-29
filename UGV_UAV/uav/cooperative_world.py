"""Build a run-specific Baylands world for one Husky and one UAV."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET


def build(source: Path, output: Path, uav_offset_east_m: float = 3.0) -> tuple[float, float, float]:
    """Use the lighter Husky and return a ground launch point behind it."""
    tree = ET.parse(source)
    world = tree.getroot().find("world")
    if world is None:
        raise ValueError(f"No world element in {source}")

    spawn: tuple[float, float, float] | None = None
    for include in world.findall("include"):
        if include.findtext("name") != "husky":
            continue
        values = [float(value) for value in include.findtext("pose", "0 0 0").split()]
        spawn = values[0], values[1], values[2]
        uri = include.find("uri")
        if uri is None:
            raise ValueError("Husky include has no URI")
        uri.text = "model://husky_cooperative"
        break
    if spawn is None:
        raise ValueError("Husky spawn is missing from Baylands")

    launch_x = spawn[0] + uav_offset_east_m
    launch_y = spawn[1]
    # PX4 adds another 0.02 m in the runner. A total 0.35 m above the saved
    # Husky reference places the x500 landing gear on the nearby terrain
    # without a visible pedestal or an initial physics drop.
    launch_z = spawn[2] + 0.33
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="unicode", xml_declaration=True)
    return launch_x, launch_y, launch_z
