from __future__ import annotations


WORLD_NAME_ALIASES = {
    "construction": "office_construction",
}


def gazebo_world_name(world: str) -> str:
    name = str(world).strip()
    return WORLD_NAME_ALIASES.get(name, name)
