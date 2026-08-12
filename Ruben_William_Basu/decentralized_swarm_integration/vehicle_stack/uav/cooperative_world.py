"""Build a run-specific Baylands world for one Husky and three UAVs."""

from __future__ import annotations

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml


DEFAULT_SITE_CONFIG = Path(__file__).resolve().parents[1] / "config/swarm_launch_site.yaml"


def build(
    source: Path,
    output: Path,
    site_config: Path = DEFAULT_SITE_CONFIG,
) -> tuple[float, float, float]:
    """Use the lighter Husky and build an open-air swarm staging deck."""
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

    site = yaml.safe_load(site_config.read_text(encoding="utf-8"))
    launch_x = float(site["center_x"])
    launch_y = float(site["center_y"])
    deck = site["deck"]
    deck_size_x = float(deck["size_x"])
    deck_size_y = float(deck["size_y"])
    deck_thickness = float(deck["thickness"])
    bays = site["bays"]
    if len(bays) != 3 or [bay["id"] for bay in bays] != ["uav0", "uav1", "uav2"]:
        raise ValueError("swarm_launch_site.yaml must define uav0, uav1 and uav2 in order")
    offsets = [(float(bay["offset_x"]), float(bay["offset_y"])) for bay in bays]
    if any(
        abs(x) > deck_size_x / 2 - 1.0 or abs(y) > deck_size_y / 2 - 1.0
        for x, y in offsets
    ):
        raise ValueError("every UAV bay must remain at least 1 m inside the deck edge")
    if any(
        ((x1 - x2) ** 2 + (y1 - y2) ** 2) ** 0.5 < 3.0
        for index, (x1, y1) in enumerate(offsets)
        for x2, y2 in offsets[index + 1 :]
    ):
        raise ValueError("UAV launch bays require at least 3 m separation")
    # Raise the deck clear of the uneven terrain. PX4 adds 0.02 m to launch_z;
    # the x500 landing-gear bottom is another 0.013 m above its model origin.
    # Therefore launch_z = deck_top - 0.033 gives exact level contact.
    pad_top_z = float(site["deck_top_z"])
    launch_z = pad_top_z - 0.033
    model = ET.SubElement(world, "model", {"name": "swarm_launch_deck"})
    ET.SubElement(model, "static").text = "true"
    ET.SubElement(model, "pose").text = (
        f"{launch_x} {launch_y} {pad_top_z - deck_thickness / 2} 0 0 0"
    )
    link = ET.SubElement(model, "link", {"name": "deck"})
    collision = ET.SubElement(link, "collision", {"name": "collision"})
    collision_geometry = ET.SubElement(collision, "geometry")
    collision_box = ET.SubElement(collision_geometry, "box")
    deck_size = f"{deck_size_x} {deck_size_y} {deck_thickness}"
    ET.SubElement(collision_box, "size").text = deck_size
    visual = ET.SubElement(link, "visual", {"name": "deck_visual"})
    visual_geometry = ET.SubElement(visual, "geometry")
    visual_box = ET.SubElement(visual_geometry, "box")
    ET.SubElement(visual_box, "size").text = deck_size
    material = ET.SubElement(visual, "material")
    ET.SubElement(material, "ambient").text = "0.12 0.14 0.16 1"
    ET.SubElement(material, "diffuse").text = "0.20 0.24 0.28 1"

    # Bright, flush circular bay markings make all three spawn positions easy
    # to recognize without adding collision edges that could catch landing gear.
    for index, bay in enumerate(bays):
        offset_x, offset_y = offsets[index]
        color = str(bay["color"])
        marker = ET.SubElement(link, "visual", {"name": f"uav{index}_bay_marker"})
        ET.SubElement(marker, "pose").text = (
            f"{offset_x} {offset_y} {deck_thickness / 2 + 0.007} 0 0 0"
        )
        marker_geometry = ET.SubElement(marker, "geometry")
        cylinder = ET.SubElement(marker_geometry, "cylinder")
        ET.SubElement(cylinder, "radius").text = "1.15"
        ET.SubElement(cylinder, "length").text = "0.014"
        marker_material = ET.SubElement(marker, "material")
        ET.SubElement(marker_material, "ambient").text = color
        ET.SubElement(marker_material, "diffuse").text = color
    # Four supports make the elevated surface visually grounded and keep its
    # collision body clear of any terrain protrusions.
    for index, (x, y) in enumerate(((-3.4, -5.4), (-3.4, 5.4), (3.4, -5.4), (3.4, 5.4))):
        support = ET.SubElement(link, "visual", {"name": f"support_{index}"})
        ET.SubElement(support, "pose").text = f"{x} {y} -0.70 0 0 0"
        support_geometry = ET.SubElement(support, "geometry")
        support_box = ET.SubElement(support_geometry, "box")
        ET.SubElement(support_box, "size").text = "0.22 0.22 1.20"
        support_material = ET.SubElement(support, "material")
        ET.SubElement(support_material, "ambient").text = "0.10 0.12 0.14 1"
        ET.SubElement(support_material, "diffuse").text = "0.18 0.20 0.22 1"
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="unicode", xml_declaration=True)
    return launch_x, launch_y, launch_z
