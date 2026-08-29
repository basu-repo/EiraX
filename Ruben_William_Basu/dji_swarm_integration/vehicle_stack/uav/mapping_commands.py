"""Robot Operating System 2 commands for aerial three-dimensional mapping."""

from __future__ import annotations

from pathlib import Path


POINTS = "/x500/lidar3d/points"
SENSOR_FRAME = "x500_mapping_0/base_link/lidar_32"


def bridge() -> list[str]:
    return [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        f"{POINTS}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "--ros-args", "-p", "override_timestamps_with_wall_time:=true",
    ]


def sensor_transform() -> list[str]:
    return [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0", "--y", "0", "--z", "-0.04",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "uav_base_link", "--child-frame-id", SENSOR_FRAME,
    ]


def cooperative_frame_transform(
    east_offset_m: float, north_offset_m: float, up_offset_m: float
) -> list[str]:
    """Align the PX4 local ENU origin with the Husky odometry origin."""
    return [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", str(east_offset_m),
        "--y", str(north_offset_m),
        "--z", str(up_offset_m),
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "odom", "--child-frame-id", "uav_px4_odom",
    ]


def aerial_obstacle_filter(
    east_offset_m: float,
    north_offset_m: float,
    up_offset_m: float,
    exclusions: list[tuple[float, float]],
) -> list[str]:
    command = [
        "python3", "-u", "-m", "uav.aerial_obstacle_filter",
        "--east-offset", str(east_offset_m),
        "--north-offset", str(north_offset_m),
        "--up-offset", str(up_offset_m),
    ]
    for x, y in exclusions:
        command.append(f"--exclude={x},{y}")
    return command


def ugv_path_relay() -> list[str]:
    return ["python3", "-u", "-m", "uav.ugv_path_relay"]


def communication_channel(
    output_dir: Path,
    config_file: Path,
    east_offset_m: float,
    north_offset_m: float,
) -> list[str]:
    return [
        "python3",
        "-u",
        "-m",
        "communication.channel",
        "--output-dir",
        str(output_dir),
        "--config",
        str(config_file),
        "--uav-east-offset",
        str(east_offset_m),
        "--uav-north-offset",
        str(north_offset_m),
        "--window-sec",
        "1.0",
    ]


def laser_odometry(*, cooperative: bool = False) -> list[str]:
    voxel_size = "0.50" if cooperative else "0.35"
    iterations = "10" if cooperative else "15"
    scan_max_size = "10000" if cooperative else "20000"
    return [
        "ros2", "run", "rtabmap_odom", "icp_odometry",
        "--Icp/PointToPlane", "true",
        "--Icp/Iterations", iterations,
        "--Icp/VoxelSize", voxel_size,
        "--Icp/PointToPlaneK", "20",
        "--Icp/MaxTranslation", "5",
        "--Icp/MaxCorrespondenceDistance", "2.0",
        "--Icp/Strategy", "1",
        "--Icp/CorrespondenceRatio", "0.01",
        "--Odom/ScanKeyFrameThr", "0.4",
        "--Odom/ResetCountdown", "3",
        "--OdomF2M/ScanMaxSize", scan_max_size,
        "--ros-args",
        "-r", "__node:=aerial_icp_odometry",
        "-p", "use_sim_time:=false",
        "-p", "frame_id:=uav_base_link",
        "-p", "odom_frame_id:=uav_lidar_odom",
        # Flight safety uses the stable PX4 odometry transform. Keep ICP
        # odometry as RTAB-Map input only so an ICP reset cannot re-parent the
        # aircraft or corrupt the obstacle planner's coordinate frame.
        "-p", "publish_tf:=false",
        "-p", "wait_for_transform:=0.3",
        "-r", f"scan_cloud:={POINTS}",
        "-r", "odom:=/uav/lidar_odom",
    ]


def mapping(database: Path, *, cooperative: bool = False) -> list[str]:
    voxel_size = "0.50" if cooperative else "0.35"
    range_max = "100" if cooperative else "120"
    return [
        "ros2", "run", "rtabmap_slam", "rtabmap", "-d",
        "--Reg/Strategy", "1",
        "--Reg/Force3DoF", "false",
        "--Icp/VoxelSize", voxel_size,
        "--Icp/RangeMin", "0.8",
        "--Icp/RangeMax", range_max,
        "--RGBD/LinearUpdate", "0.5",
        "--RGBD/AngularUpdate", "0.15",
        "--Grid/3D", "true",
        "--Grid/NormalsSegmentation", "true",
        "--Grid/RangeMax", range_max,
        "--ros-args",
        "-r", "__node:=aerial_rtabmap",
        "-r", "__ns:=/uav_rtabmap",
        "-p", "use_sim_time:=false",
        "-p", "frame_id:=uav_base_link",
        "-p", "map_frame_id:=uav_map",
        "-p", "publish_tf:=true",
        "-p", "wait_for_transform:=0.3",
        "-p", "database_path:=" + str(database),
        "-p", "subscribe_rgb:=false",
        "-p", "subscribe_depth:=false",
        "-p", "subscribe_stereo:=false",
        "-p", "subscribe_scan_cloud:=true",
        "-p", "subscribe_odom_info:=false",
        "-p", "approx_sync:=true",
        "-r", f"scan_cloud:={POINTS}",
        "-r", "odom:=/uav/lidar_odom",
        "-r", "map:=/uav/map",
        "-r", "tf:=/tf",
        "-r", "tf_static:=/tf_static",
    ]


def recorder(directory: Path) -> list[str]:
    return [
        "ros2", "bag", "record", "-o", str(directory),
        POINTS, "/uav/lidar_odom", "/uav_rtabmap/cloud_map",
        "/uav_rtabmap/mapGraph", "/uav_rtabmap/local_grid_obstacle",
        "/uav/planned_path", "/uav_nav/global_costmap/costmap",
        "/cooperative/aerial_obstacles", "/cooperative/ugv_global_path",
        "/tf", "/tf_static",
    ]


def rviz_viewer(config: Path) -> list[str]:
    return ["ros2", "run", "rviz2", "rviz2", "-d", str(config)]


def nav2_planner(params: Path) -> list[str]:
    return [
        "ros2", "run", "nav2_planner", "planner_server",
        "--ros-args", "--params-file", str(params),
        "-r", "__ns:=/uav_nav",
    ]


def nav2_lifecycle() -> list[str]:
    return [
        "ros2", "run", "nav2_lifecycle_manager", "lifecycle_manager",
        "--ros-args", "-r", "__node:=uav_planner_lifecycle",
        "-r", "__ns:=/uav_nav",
        "-p", "use_sim_time:=false", "-p", "autostart:=true",
        "-p", "node_names:=['planner_server']",
        "-p", "bond_timeout:=10.0",
        "-p", "service_timeout:=30.0",
    ]


def rtabmap_viewer(config: Path | None = None) -> list[str]:
    command = ["ros2", "run", "rtabmap_viz", "rtabmap_viz"]
    if config is not None:
        command.extend(["-d", str(config)])
    command.extend([
        "--ros-args",
        "-r", "__node:=aerial_rtabmap_viz",
        "-r", "__ns:=/uav_rtabmap",
        "-p", "use_sim_time:=false",
        "-p", "frame_id:=uav_base_link",
        "-p", "subscribe_rgb:=false",
        "-p", "subscribe_depth:=false",
        "-p", "subscribe_stereo:=false",
        "-p", "subscribe_scan_cloud:=true",
        "-p", "subscribe_odom_info:=false",
        "-p", "approx_sync:=true",
        "-r", f"scan_cloud:={POINTS}",
        "-r", "odom:=/uav/lidar_odom",
        "-r", "tf:=/tf",
        "-r", "tf_static:=/tf_static",
    ])
    return command
