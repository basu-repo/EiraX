from pathlib import Path
import os
import re


ROOT = Path(__file__).resolve().parents[1]


def test_python_runtime_does_not_import_existing_eirax_projects():
    forbidden = ("UGV_UAV", "UGV_Standalone", "UAV_5GSim", "UGV_UAV_5G_CoSimulation")
    sources = "\n".join(
        path.read_text(encoding="utf-8")
        for path in (ROOT / "decentralized_swarm_integration").glob("*.py")
    )
    for name in forbidden:
        assert f"import {name}" not in sources
        assert f"from {name}" not in sources


def test_no_runtime_source_references_original_project_paths():
    roots = (
        ROOT / "decentralized_swarm_integration",
        ROOT / "launch",
        ROOT / "scripts",
        ROOT / "vehicle_stack",
        ROOT / "ground_stack",
        ROOT / "omnet",
    )
    suffixes = {".py", ".yaml", ".yml", ".ini", ".ned", ".cc", ".h", ".sh"}
    forbidden = (
        "/home/basudeo/Documents/EiraX/UGV_UAV",
        "/home/basudeo/Documents/EiraX/UGV_Standalone",
        "/home/basudeo/Documents/EiraX/simulation",
        "/home/basudeo/Documents/EiraX/PX4-Autopilot",
        "../UAV_UGV-main",
        "../halmstad_ws-main",
    )
    for directory in roots:
        for path in directory.rglob("*"):
            if not path.is_file() or path.suffix not in suffixes:
                continue
            source = path.read_text(encoding="utf-8", errors="ignore")
            for value in forbidden:
                assert value not in source, f"{path} references {value}"


def test_all_runtime_symlinks_resolve_inside_project():
    for path in ROOT.rglob("*"):
        if not path.is_symlink():
            continue
        resolved = path.resolve()
        assert resolved.exists(), f"broken symlink: {path}"
        assert resolved == ROOT or ROOT in resolved.parents, (
            f"external symlink: {path} -> {os.readlink(path)}"
        )


def test_omnet_build_has_no_simu5g_link_dependency():
    build_script = (ROOT / "omnet" / "build.sh").read_text(encoding="utf-8")
    assert "-lsimu5g" not in build_script.lower()
    assert "SIMU5G_PROJ" not in build_script
    assert "UAV_UGV-main" not in build_script


def test_three_uav_defaults_are_consistent():
    launch = (ROOT / "launch/full_swarm.launch.py").read_text(encoding="utf-8")
    ini = (ROOT / "omnet/omnetpp.ini").read_text(encoding="utf-8")
    ned = (ROOT / "omnet/EiraXSwarmNetwork.ned").read_text(encoding="utf-8")
    assert 'default_value="uav0,uav1,uav2"' in launch
    assert "14540,udpin:0.0.0.0:14541,udpin:0.0.0.0:14542" in launch
    assert 'DeclareLaunchArgument("start_px4", default_value="false")' in launch
    assert re.search(r"^\*\.numUavs\s*=\s*3$", ini, re.MULTILINE)
    assert 'trackedModel = "uav0"' in ini
    assert 'trackedModel = "uav1"' in ini
    assert 'trackedModel = "uav2"' in ini
    assert "int numUavs = default(1);" in ned
