from __future__ import annotations

from pathlib import Path


def package_source_root(module_file: str) -> Path:
    module_path = Path(module_file).resolve()
    for parent in module_path.parents:
        if (parent / "setup.py").is_file() and (parent / "package.xml").is_file():
            return parent
    raise RuntimeError(f"Could not locate package source root from {module_file}")


def workspace_root(module_file: str) -> Path:
    return package_source_root(module_file).parents[1]
