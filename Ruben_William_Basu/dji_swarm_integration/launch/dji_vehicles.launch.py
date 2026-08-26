"""Spawn and bridge three isolated DJI M100 aircraft in the Baylands world."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    lrs_share = get_package_share_directory("lrs_halmstad")
    params = os.path.join(lrs_share, "config", "run_follow_defaults.yaml")
    world = LaunchConfiguration("world")
    mode = LaunchConfiguration("mode")
    scenario = LaunchConfiguration("scenario")
    center_x = LaunchConfiguration("center_x")
    center_y = LaunchConfiguration("center_y")
    spawn_z = LaunchConfiguration("spawn_z")
    bay_y = (
        PythonExpression([center_y, " - 4.0"]),
        center_y,
        PythonExpression([center_y, " + 4.0"]),
    )

    spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lrs_share, "spawn_uavs.launch.py")),
        launch_arguments={
            "world": world,
            "uav_mode": "teleport",
            "camera_mode": "integrated",
            "camera_update_rate": "2",
            "dji0_x": center_x,
            "dji0_y": bay_y[0],
            "dji0_z": spawn_z,
            "dji1_x": center_x,
            "dji1_y": center_y,
            "dji1_z": spawn_z,
            "dji2_x": center_x,
            "dji2_y": bay_y[2],
            "dji2_z": spawn_z,
        }.items(),
    )

    nodes = []
    for index in range(3):
        name = f"dji{index}"
        start_y = bay_y[index]
        nodes.extend(
            [
                Node(
                    package="lrs_halmstad",
                    executable="simulator",
                    name=f"{name}_simulator",
                    output="screen",
                    parameters=[
                        params,
                        {
                            "use_sim_time": False,
                            "world": world,
                            "uav_name": name,
                            "camera_mode": "integrated_joint",
                            "update_rate_hz": 5.0,
                            "camera_x_offset_m": 0.0,
                            "camera_y_offset_m": 0.0,
                            "camera_z_offset_m": 0.27,
                            "camera_mount_pitch_deg": 45.0,
                            "camera_yaw_offset_deg": 0.0,
                            "camera_pan_sign": 1.0,
                            "gimbal_pitch_min_rad": -1.5708,
                            "gimbal_pitch_max_rad": 0.5235,
                            "pan_rate_deg_s": 60.0,
                            "tilt_rate_deg_s": 45.0,
                            "start_x": center_x,
                            "start_y": start_y,
                            "start_z": spawn_z,
                            "start_yaw_deg": 0.0,
                            "startup_set_pose_grace_s": 3.0,
                            "set_pose_future_timeout_s": 2.0,
                            "set_pose_failure_backoff_s": 2.0,
                            "pan_enable": True,
                            "tilt_enable": True,
                        },
                    ],
                ),
                Node(
                    package="lrs_halmstad",
                    executable="pose_cmd_to_odom",
                    name=f"{name}_pose_to_odom",
                    output="screen",
                    parameters=[{
                        "use_sim_time": False,
                        "pose_topic": f"/{name}/pose",
                        "odom_topic": f"/{name}/pose/odom",
                        "frame_id": "map",
                        "child_frame_id": f"{name}/base_link",
                    }],
                ),
                Node(
                    package="dji_swarm_integration",
                    executable="dji_vehicle_agent",
                    name=f"{name}_vehicle_agent",
                    output="screen",
                    parameters=[{
                        "use_sim_time": False,
                        "uav_id": name,
                        "mode": mode,
                        "hardware_authorized": False,
                    }],
                ),
            ]
        )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="baylands_editable"),
            DeclareLaunchArgument("mode", default_value="simulation"),
            DeclareLaunchArgument("scenario", default_value="normal"),
            DeclareLaunchArgument("center_x", default_value="218.766"),
            DeclareLaunchArgument("center_y", default_value="-275.249"),
            DeclareLaunchArgument("spawn_z", default_value="3.5217"),
            spawn,
            *nodes,
            Node(
                package="dji_swarm_integration",
                executable="dji_mission_director",
                name="dji_mission_director",
                output="screen",
                parameters=[{
                    # Mission timers must remain frozen while Gazebo is paused.
                    # Vehicle pose adapters deliberately use wall time, but no
                    # mission command is emitted until /clock advances.
                    "use_sim_time": True,
                    "scenario": scenario,
                    "start_delay_s": 30.0,
                }],
            ),
        ]
    )
