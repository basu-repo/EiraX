#!/usr/bin/env python3
"""Launch Baylands and validate exactly one local DJI M100."""

from __future__ import annotations

import os
from pathlib import Path
import signal
import shlex
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from robots.husky.runtime.process_manager import ProcessManager
from robots.husky.data_logging.run_dataset import RunDataset
from robots.husky.monitoring.topic_health import gazebo_running


WORLD = PROJECT_ROOT / "simulation/worlds/baylands/baylands_editable.world"
GAZEBO_WORLD_NAME = "baylands_editable"
DJI_INSTALL = PROJECT_ROOT / "install/setup.bash"
DJI_PARAMS = (
    PROJECT_ROOT
    / "src/lrs_halmstad/config/run_follow_defaults.yaml"
)
def named_pose(name: str) -> tuple[float, float, float]:
    root = ET.parse(WORLD).getroot()
    for include in root.findall(".//include"):
        if include.findtext("name") == name:
            values = [float(item) for item in include.findtext("pose").split()]
            return values[0], values[1], values[2]
    raise RuntimeError(f"Entity {name!r} is missing from {WORLD}")


def ros_command(arguments: list[str]) -> list[str]:
    command = shlex.join(arguments)
    return [
        "bash",
        "--noprofile",
        "--norc",
        "-c",
        f"source /opt/ros/jazzy/setup.bash; source {DJI_INSTALL}; exec {command}",
    ]


def build_one_dji_world(
    output: Path, x: float, y: float, z: float, env: dict[str, str]
) -> None:
    xacro_file = (
        PROJECT_ROOT
        / "robots/dji_m100/ros2_ws/src/lrs_halmstad/xacro/lrs_model.xacro"
    )
    generated = subprocess.run(
        ros_command([
            "xacro", str(xacro_file), "name:=dji0", "robot_type:=m100",
            "with_camera:=true", "model_static:=false",
            "base_link_kinematic:=true", "camera_pitch_offset:=45",
            "camera_name:=camera0", "camera_update_rate:=10",
        ]),
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    if generated.returncode != 0:
        raise RuntimeError(generated.stderr.strip() or "M100 Xacro generation failed")

    model = ET.fromstring(generated.stdout).find("model")
    if model is None or model.get("name") != "dji0":
        raise RuntimeError("Generated M100 SDF does not contain model dji0")
    pose = ET.Element("pose")
    pose.text = f"{x} {y} {z} 0 0 0"
    model.insert(0, pose)

    tree = ET.parse(WORLD)
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError(f"No world element in {WORLD}")
    world.append(model)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def build_dji_swarm_world(
    output: Path,
    aircraft: list[tuple[str, float, float, float]],
    env: dict[str, str],
) -> None:
    """Embed independent DJI M100 entities in a private Baylands world."""
    xacro_file = (
        PROJECT_ROOT
        / "robots/dji_m100/ros2_ws/src/lrs_halmstad/xacro/lrs_model.xacro"
    )
    tree = ET.parse(WORLD)
    world = tree.getroot().find("world")
    if world is None:
        raise RuntimeError(f"No world element in {WORLD}")
    for name, x, y, z in aircraft:
        generated = subprocess.run(
            ros_command([
                "xacro", str(xacro_file), f"name:={name}", "robot_type:=m100",
                "with_camera:=true", "model_static:=false",
                "base_link_kinematic:=true", "camera_pitch_offset:=45",
                # Three GPU cameras share the renderer in swarm mode. Five Hz
                # is sufficient for semantic observation and gimbal tracking
                # while leaving Gazebo GUI capacity for smooth visualization.
                "camera_name:=camera0", "camera_update_rate:=5",
            ]),
            env=env,
            capture_output=True,
            text=True,
            timeout=60,
            check=False,
        )
        if generated.returncode != 0:
            raise RuntimeError(
                generated.stderr.strip() or f"M100 Xacro generation failed for {name}"
            )
        model = ET.fromstring(generated.stdout).find("model")
        if model is None or model.get("name") != name:
            raise RuntimeError(f"Generated M100 SDF does not contain model {name}")
        pose = ET.Element("pose")
        pose.text = f"{x} {y} {z} 0 0 0"
        model.insert(0, pose)
        world.append(model)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)


def main() -> int:
    if gazebo_running():
        print("[FAILED] Another Gazebo server is already running. Close it first.")
        return 1
    if not DJI_INSTALL.is_file():
        print("[FAILED] Local DJI workspace is not built.")
        print("Run: cd robots/dji_m100/ros2_ws && colcon build")
        return 2

    env = os.environ.copy()
    model_path = str(PROJECT_ROOT / "simulation/models")
    env["GZ_SIM_RESOURCE_PATH"] = ":".join(
        value for value in (model_path, env.get("GZ_SIM_RESOURCE_PATH", "")) if value
    )
    env["SDF_PATH"] = ":".join(
        value for value in (model_path, env.get("SDF_PATH", "")) if value
    )
    env["PYTHONPATH"] = ":".join(
        value for value in (str(PROJECT_ROOT), env.get("PYTHONPATH", "")) if value
    )

    validation = subprocess.run(
        ["gz", "sdf", "-k", str(WORLD)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    if validation.returncode != 0:
        print("[FAILED] Baylands validation failed.")
        print((validation.stderr or validation.stdout).strip())
        return 2

    husky_x, husky_y, husky_z = named_pose("husky")
    # Put the first-aircraft validation directly in front of and above the
    # Husky so it is immediately visible from the saved Baylands GUI view.
    start_x = husky_x - 3.0
    start_y = husky_y
    start_z = husky_z + 2.5
    dataset = RunDataset(PROJECT_ROOT, WORLD)
    test_world = dataset.world / "one_dji_baylands.world"
    env["ROS_LOG_DIR"] = str(dataset.logs / "ros")
    try:
        build_one_dji_world(test_world, start_x, start_y, start_z, env)
    except (OSError, RuntimeError, subprocess.TimeoutExpired) as error:
        print(f"[FAILED] Could not build the one-DJI test world: {error}")
        return 2
    model_validation = subprocess.run(
        ["gz", "sdf", "-k", str(test_world)],
        env=env,
        capture_output=True,
        text=True,
        timeout=60,
        check=False,
    )
    if model_validation.returncode != 0:
        print("[FAILED] Generated one-DJI world is invalid.")
        print((model_validation.stderr or model_validation.stdout).strip())
        return 2
    manager = ProcessManager(dataset.logs, env)
    stopping = False

    def request_stop(_signum=None, _frame=None) -> None:
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    try:
        print("[PREFLIGHT] Baylands and the local DJI package are valid.")
        print("[ENTITY] dji0 is embedded directly in the test world.")
        gazebo = manager.start("gazebo", ["gz", "sim", str(test_world)])
        time.sleep(3.0)
        if gazebo.process.poll() is not None:
            print(f"[FAILED] Gazebo exited. Log: {dataset.logs / 'gazebo.log'}")
            return 2

        manager.start(
            "dji_bridges",
            ros_command([
                "ros2", "run", "ros_gz_bridge", "parameter_bridge",
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                f"/world/{GAZEBO_WORLD_NAME}/set_pose@ros_gz_interfaces/srv/SetEntityPose",
                "/dji0/camera0/image@sensor_msgs/msg/Image[gz.msgs.Image",
                "/dji0/camera0/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo",
                "--ros-args", "-r",
                "/dji0/camera0/image:=/dji0/camera0/image_raw",
            ]),
        )
        time.sleep(2.0)
        manager.start(
            "dji_simulator",
            ros_command([
                "ros2", "run", "lrs_halmstad", "simulator", "--ros-args",
                "--params-file", str(DJI_PARAMS), "-p", "use_sim_time:=true",
                "-p", f"world:={GAZEBO_WORLD_NAME}", "-p", "uav_name:=dji0",
                "-p", f"start_x:={start_x}", "-p", f"start_y:={start_y}",
                "-p", f"start_z:={start_z}",
            ]),
        )
        mission = manager.start(
            "dji_check",
            ros_command([
                "ros2", "run", "lrs_halmstad", "vastervik_dji_check",
                "--name", "dji0", "--x", str(start_x), "--y", str(start_y),
                "--z", str(start_z),
            ]),
        )
        print(f"[STARTED] One DJI M100: dji0 at ({start_x:.1f}, {start_y:.1f}, {start_z:.1f})")
        print("[CAMERA] /dji0/camera0/image_raw")
        print("[WAITING] Click Play in Gazebo. Press Ctrl+C once to stop.")

        while not stopping:
            code = mission.process.poll()
            if code is not None:
                if code != 0:
                    print(f"[FAILED] DJI check exited with code {code}.")
                    print(f"[LOG] {dataset.logs / 'dji_check.log'}")
                    return code
                print("[DJI CHECK COMPLETE] Gazebo remains open for inspection.")
                print("Press Ctrl+C to close everything.")
                while not stopping and gazebo.process.poll() is None:
                    time.sleep(0.5)
                return 0
            failures = manager.failures()
            if failures:
                print(f"[FAILED] {failures}")
                return 3
            time.sleep(0.5)
        return 0
    finally:
        print("[STOPPING] Closing the one-DJI test...")
        manager.stop_all()
        print(f"[SAVED] {dataset.root}")


if __name__ == "__main__":
    raise SystemExit(main())
