"""Build commands for standalone UGV runtime components."""

from __future__ import annotations

from pathlib import Path


def gazebo(world: Path, headless: bool = False) -> list[str]:
    if headless:
        return ["gz", "sim", "-s", "-r", str(world)]
    return ["gz", "sim", str(world)]


def bridge() -> list[str]:
    return [
        "ros2", "run", "ros_gz_bridge", "parameter_bridge",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/husky/lidar3d/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked",
        "/model/husky/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose",
        "/wheel/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/model/husky/imu@sensor_msgs/msg/Imu[gz.msgs.IMU",
        "/husky/gps@sensor_msgs/msg/NavSatFix[gz.msgs.NavSat",
        "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
        "--ros-args",
        "-r", "/clock:=/gazebo_clock",
        "-r", "/model/husky/imu:=/imu",
        "-r", "/model/husky/pose:=/ground_truth/pose",
    ]


def monotonic_clock() -> list[str]:
    return ["python3", "-m", "baseline.core.monotonic_clock"]


def lidar3d_transform() -> list[str]:
    return [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0.374", "--y", "0", "--z", "0.435",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "base_link",
        "--child-frame-id", "husky/base_link/front_laser",
    ]


def imu_transform() -> list[str]:
    return [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0", "--y", "0", "--z", "0",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "base_link",
        "--child-frame-id", "husky/base_link/imu_sensor",
    ]


def gps_transform() -> list[str]:
    return [
        "ros2", "run", "tf2_ros", "static_transform_publisher",
        "--x", "0", "--y", "0", "--z", "0.45",
        "--roll", "0", "--pitch", "0", "--yaw", "0",
        "--frame-id", "base_link",
        "--child-frame-id", "husky/base_link/navsat_sensor",
    ]


def localization(params_file: Path) -> list[str]:
    return [
        "ros2", "run", "robot_localization", "ekf_node",
        "--ros-args", "--params-file", str(params_file),
        "-r", "odometry/filtered:=/odom",
    ]


def local_localization(params_file: Path) -> list[str]:
    return [
        "ros2", "run", "robot_localization", "ekf_node",
        "--ros-args", "--params-file", str(params_file),
        "-r", "__node:=local_ekf",
        "-r", "odometry/filtered:=/odometry/local",
    ]


def navsat_transform(params_file: Path) -> list[str]:
    return [
        "ros2", "run", "robot_localization", "navsat_transform_node",
        "--ros-args", "--params-file", str(params_file),
        "-r", "imu:=/imu",
        "-r", "gps/fix:=/husky/gps",
        "-r", "odometry/filtered:=/odometry/local",
        "-r", "odometry/gps:=/odometry/gps",
        "-r", "gps/filtered:=/gps/filtered",
    ]


def slam3d(database: Path) -> list[str]:
    return [
        "ros2", "run", "rtabmap_slam", "rtabmap",
        "-d",
        "--Reg/Strategy", "1",
        "--Reg/Force3DoF", "true",
        "--Icp/VoxelSize", "0.20",
        "--Icp/RangeMin", "0.8",
        "--Icp/RangeMax", "60",
        "--Grid/3D", "true",
        "--Grid/NormalsSegmentation", "true",
        "--Grid/MaxGroundAngle", "70",
        "--Grid/MinClusterSize", "20",
        "--Grid/RangeMin", "0.8",
        "--Grid/RangeMax", "30",
        "--Grid/MaxObstacleHeight", "2.5",
        "--ros-args",
        "-r", "__node:=rtabmap",
        "-r", "__ns:=/rtabmap3d",
        "-p", "use_sim_time:=true",
        "-p", "frame_id:=base_link",
        "-p", "map_frame_id:=map",
        "-p", "publish_tf:=true",
        "-p", "wait_for_transform:=0.2",
        "-p", "database_path:=" + str(database),
        "-p", "subscribe_rgb:=false",
        "-p", "subscribe_depth:=false",
        "-p", "subscribe_stereo:=false",
        "-p", "subscribe_scan_cloud:=true",
        "-p", "approx_sync:=true",
        "-r", "scan_cloud:=/husky/lidar3d/points",
        "-r", "odom:=/odom",
        "-r", "map:=/map",
    ]


def lidar3d_odometry() -> list[str]:
    """Independent scan-to-map odometry from the OS1-64 point cloud."""
    return [
        "ros2", "run", "rtabmap_odom", "icp_odometry",
        "--Icp/PointToPlane", "true",
        "--Icp/Iterations", "10",
        "--Icp/VoxelSize", "0.20",
        "--Icp/Epsilon", "0.001",
        "--Icp/PointToPlaneK", "20",
        "--Icp/MaxTranslation", "3",
        "--Icp/MaxCorrespondenceDistance", "1.0",
        "--Icp/Strategy", "1",
        "--Icp/OutlierRatio", "0.7",
        "--Odom/ScanKeyFrameThr", "0.4",
        "--OdomF2M/ScanSubtractRadius", "0.20",
        "--OdomF2M/ScanMaxSize", "15000",
        "--OdomF2M/BundleAdjustment", "false",
        "--Icp/CorrespondenceRatio", "0.01",
        "--ros-args",
        "-r", "__node:=icp_odometry",
        "-r", "__ns:=/lidar3d",
        "-p", "use_sim_time:=true",
        "-p", "frame_id:=base_link",
        "-p", "odom_frame_id:=lidar_odom",
        "-p", "publish_tf:=false",
        "-p", "wait_for_transform:=0.2",
        "-p", "approx_sync:=true",
        "-r", "scan_cloud:=/husky/lidar3d/points",
    ]


def lidar3d_slam(database: Path) -> list[str]:
    """Independent RTAB-Map graph driven only by 3D LiDAR odometry."""
    return [
        "ros2", "run", "rtabmap_slam", "rtabmap", "-d",
        "--Reg/Strategy", "1",
        "--Reg/Force3DoF", "true",
        "--Icp/VoxelSize", "0.20",
        "--Icp/RangeMin", "0.8",
        "--Icp/RangeMax", "60",
        "--RGBD/ProximityPathMaxNeighbors", "1",
        "--RGBD/ProximityMaxGraphDepth", "0",
        "--RGBD/LinearUpdate", "0.20",
        "--RGBD/AngularUpdate", "0.10",
        "--Grid/3D", "true",
        "--Grid/NormalsSegmentation", "true",
        "--Grid/MaxGroundAngle", "70",
        "--Grid/MinClusterSize", "20",
        "--ros-args",
        "-r", "__node:=rtabmap",
        "-r", "__ns:=/rtabmap_lidar",
        "-p", "use_sim_time:=true",
        "-p", "frame_id:=base_link",
        "-p", "map_frame_id:=lidar_map",
        "-p", "publish_tf:=false",
        "-p", "wait_for_transform:=0.2",
        "-p", "database_path:=" + str(database),
        "-p", "subscribe_rgb:=false",
        "-p", "subscribe_depth:=false",
        "-p", "subscribe_stereo:=false",
        "-p", "subscribe_scan_cloud:=true",
        "-p", "subscribe_odom_info:=false",
        "-p", "approx_sync:=true",
        "-r", "scan_cloud:=/husky/lidar3d/points",
        "-r", "odom:=/lidar3d/odom",
        "-r", "map:=/lidar_slam/map",
    ]


def slam3d_viewer(config: Path | None = None) -> list[str]:
    command = [
        "ros2", "run", "rtabmap_viz", "rtabmap_viz",
    ]
    if config is not None:
        command.extend(["-d", str(config)])
    command.extend([
        "--ros-args",
        "-r", "__node:=rtabmap_viz",
        "-r", "__ns:=/rtabmap3d",
        "-p", "use_sim_time:=true",
        "-p", "frame_id:=base_link",
        "-p", "subscribe_rgb:=false",
        "-p", "subscribe_depth:=false",
        "-p", "subscribe_stereo:=false",
        "-p", "subscribe_scan_cloud:=true",
        "-p", "approx_sync:=true",
        "-r", "scan_cloud:=/husky/lidar3d/points",
        "-r", "odom:=/odom",
    ])
    return command


def nav2(params_file: Path) -> list[str]:
    return [
        "ros2", "launch", "nav2_bringup", "navigation_launch.py",
        f"params_file:={params_file}", "use_sim_time:=true",
        "autostart:=true", "use_composition:=False",
    ]


def waypoint_mission(world: Path, events: Path, targets: list[str]) -> list[str]:
    return [
        "python3", "-m", "baseline.mission.waypoint_mission",
        "--world", str(world), "--events", str(events),
        "--targets", *targets,
    ]


def recorder(output: Path, topics: list[str]) -> list[str]:
    return [
        "ros2", "bag", "record",
        "--storage", "mcap",
        "--storage-preset-profile", "zstd_fast",
        "--output", str(output),
        *topics,
    ]


def localization_evaluator(output: Path) -> list[str]:
    return [
        "python3", "-m", "baseline.evaluation.localization_recorder",
        "--output", str(output),
    ]
