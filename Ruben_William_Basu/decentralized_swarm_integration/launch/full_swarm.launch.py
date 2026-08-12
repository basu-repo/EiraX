"""Launch the isolated PX4, YOLO, consensus, roles, and OMNeT bridges."""

from __future__ import annotations

import copy

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _csv(context, name):
    return [
        item.strip()
        for item in LaunchConfiguration(name).perform(context).split(",")
        if item.strip()
    ]


def _load_halmstad_params(node_name):
    path = get_package_share_directory("lrs_halmstad") + "/config/run_follow_defaults.yaml"
    with open(path, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    parameters = copy.deepcopy(data.get("/**", {}).get("ros__parameters", {}))
    parameters.update(copy.deepcopy(data[node_name]["ros__parameters"]))
    return parameters


def _default_yolo_weights():
    return (
        get_package_share_directory("lrs_halmstad")
        + "/weights/baylands-leader-v4-2-best.pt"
    )


def _build(context):
    uavs = _csv(context, "uav_names")
    endpoints = _csv(context, "px4_connection_urls")
    origin_tokens = _csv(context, "px4_map_origins")
    if not uavs or len(uavs) != len(set(uavs)):
        raise ValueError("uav_names must contain unique identifiers")
    if len(uavs) != len(endpoints):
        raise ValueError("uav_names and px4_connection_urls must have equal length")
    if len(uavs) != len(origin_tokens):
        raise ValueError("uav_names and px4_map_origins must have equal length")
    try:
        origins = [tuple(float(value) for value in item.split(":")) for item in origin_tokens]
    except ValueError as exc:
        raise ValueError("px4_map_origins must use x:y:z numeric triples") from exc
    if any(len(origin) != 3 for origin in origins):
        raise ValueError("every px4_map_origins entry must be an x:y:z triple")
    start_px4 = _bool(LaunchConfiguration("start_px4").perform(context))
    start_yolo = _bool(LaunchConfiguration("start_yolo").perform(context))
    start_camera_bridge = _bool(
        LaunchConfiguration("start_camera_bridge").perform(context)
    )
    control_enabled = _bool(LaunchConfiguration("control_enabled").perform(context))
    use_sim_time = _bool(LaunchConfiguration("use_sim_time").perform(context))
    camera_template = LaunchConfiguration("camera_topic_template").perform(context)
    camera_info_template = LaunchConfiguration("camera_info_topic_template").perform(context)
    depth_template = LaunchConfiguration("depth_topic_template").perform(context)
    yolo_weights = LaunchConfiguration("yolo_weights").perform(context)
    yolo_device = LaunchConfiguration("yolo_device").perform(context)
    evidence_root = LaunchConfiguration("yolo_evidence_root").perform(context)
    ugv_odom_topic = LaunchConfiguration("ugv_odom_topic").perform(context)
    pose_bridge_port = int(LaunchConfiguration("pose_bridge_port").perform(context))
    metrics_base_port = int(LaunchConfiguration("metrics_base_port").perform(context))
    perception_pose_template = LaunchConfiguration(
        "perception_pose_topic_template"
    ).perform(context)

    actions = []
    model_names = ["ugv", *uavs]
    uav_odom_template = LaunchConfiguration("uav_odom_topic_template").perform(context)
    odom_topics = [
        ugv_odom_topic,
        *[uav_odom_template.replace("{uav}", uav) for uav in uavs],
    ]
    # Track each PX4/Gazebo instance directly. This gives every network and
    # role peer distinct odometry without competing with the accepted MAVLink
    # follower that exclusively owns instance 0.
    for index, uav in enumerate(uavs):
        actions.append(
            Node(
                package="lrs_halmstad",
                executable="gazebo_model_pose_bridge",
                namespace=f"swarm/{uav}",
                name="gazebo_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "world": "baylands_editable",
                        "model_name": f"x500_mapping_{index}",
                        "pose_topic": "ground_truth/pose",
                        "odom_topic": "ground_truth/odom",
                        "frame_id": "map",
                        "child_frame_id": f"{uav}_base_link",
                        "publish_rate_hz": 2.0,
                        "use_sim_time": use_sim_time,
                    }
                ],
            )
        )
    actions.extend(
        [
            Node(
                package="decentralized_swarm_integration",
                executable="multi_pose_bridge",
                name="multi_pose_bridge",
                output="screen",
                parameters=[
                    {
                        "model_names": model_names,
                        "odom_topics": odom_topics,
                        "port": pose_bridge_port,
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
            Node(
                package="decentralized_swarm_integration",
                executable="multi_metrics_bridge",
                name="multi_metrics_bridge",
                output="screen",
                parameters=[
                    {
                        "endpoints": [
                            f"{uav}:{metrics_base_port + index}"
                            for index, uav in enumerate(uavs)
                        ],
                        "use_sim_time": use_sim_time,
                    }
                ],
            ),
        ]
    )

    detector_base = _load_halmstad_params("leader_detector") if start_yolo else None
    estimator_base = _load_halmstad_params("leader_estimator") if start_yolo else None
    for index, (uav, endpoint, origin) in enumerate(zip(uavs, endpoints, origins)):
        detection_topic = f"/coord/support/{uav}/leader_detection"
        detection_status_topic = f"/coord/support/{uav}/leader_detection_status"
        estimate_topic = f"/coord/support/{uav}/leader_estimate"
        camera_topic = camera_template.replace("{uav}", uav)
        camera_info_topic = camera_info_template.replace("{uav}", uav)
        depth_topic = depth_template.replace("{uav}", uav) if depth_template else ""
        if start_yolo and start_camera_bridge:
            gz_camera_root = (
                f"/world/baylands_editable/model/x500_mapping_{index}/"
                "link/base_link/sensor/uav_rgbd"
            )
            actions.append(
                Node(
                    package="ros_gz_bridge",
                    executable="parameter_bridge",
                    name=f"{uav}_rgbd_bridge",
                    output="screen",
                    arguments=[
                        f"{gz_camera_root}/image@sensor_msgs/msg/Image[gz.msgs.Image",
                        f"{gz_camera_root}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
                        f"{gz_camera_root}/depth_image@sensor_msgs/msg/Image[gz.msgs.Image",
                    ],
                    remappings=[
                        (f"{gz_camera_root}/image", camera_topic),
                        (f"{gz_camera_root}/camera_info", camera_info_topic),
                        (f"{gz_camera_root}/depth_image", depth_topic),
                    ],
                )
            )
        if start_px4:
            actions.append(
                Node(
                    package="decentralized_swarm_integration",
                    executable="px4_agent",
                    namespace=f"swarm/{uav}",
                    name="px4_agent",
                    output="screen",
                    parameters=[
                        {
                            "uav_id": uav,
                            "connection_url": endpoint,
                            "source_system": 240 + index,
                            "control_enabled": control_enabled,
                            "map_origin_x": origin[0],
                            "map_origin_y": origin[1],
                            "map_origin_z": origin[2],
                            "use_sim_time": use_sim_time,
                        }
                    ],
                )
            )
        if start_yolo:
            detector = copy.deepcopy(detector_base)
            detector.update(
                uav_name=uav,
                camera_topic=camera_topic,
                out_topic=detection_topic,
                status_topic=detection_status_topic,
                event_topic=f"/coord/swarm/{uav}/perception_events",
                publish_events=False,
                device=yolo_device,
                yolo_weights=yolo_weights,
                target_class_name="ugv",
                # Three simultaneous 5 Hz CPU workers starved Gazebo/Nav2.
                # Independent 0.5 Hz observations are sufficient for a UGV
                # bounded to 0.8 m/s while three CPU models run concurrently.
                predict_hz=0.5,
                evidence_root=evidence_root,
                evidence_positive_interval_s=2.0,
                evidence_negative_interval_s=20.0,
                evidence_jpeg_quality=85,
                evidence_max_positive=500,
                evidence_max_negative=100,
                use_sim_time=use_sim_time,
            )
            estimator = copy.deepcopy(estimator_base)
            estimator.update(
                uav_name=uav,
                camera_topic=camera_topic,
                camera_info_topic=camera_info_topic,
                depth_topic=depth_topic,
                uav_pose_topic=perception_pose_template.replace("{uav}", uav),
                camera_x_offset_m=0.153,
                camera_y_offset_m=0.0,
                camera_z_offset_m=-0.043,
                camera_mount_pitch_deg=45.0,
                external_detection_topic=detection_topic,
                external_detection_status_topic=detection_status_topic,
                radio_range_topic=f"/coord/swarm/{uav}/network/radio_distance_m",
                out_topic=estimate_topic,
                status_topic=f"/coord/support/{uav}/leader_estimate_status",
                fault_status_topic=f"/coord/support/{uav}/leader_estimate_fault",
                debug_image_topic=f"/coord/support/{uav}/leader_debug_image",
                event_topic=f"/coord/swarm/{uav}/perception_events",
                publish_selected_target=False,
                publish_debug_image=False,
                publish_events=False,
                use_sim_time=use_sim_time,
            )
            estimator["est_hz"] = 2.0
            estimator["use_receive_time_for_freshness"] = True
            actions.extend(
                [
                    Node(
                        package="decentralized_swarm_integration",
                        executable="isolated_leader_detector",
                        namespace=f"swarm/{uav}/perception",
                        name="leader_detector",
                        output="screen",
                        parameters=[detector],
                    ),
                    Node(
                        package="decentralized_swarm_integration",
                        executable="isolated_leader_estimator",
                        namespace=f"swarm/{uav}/perception",
                        name="leader_estimator",
                        output="screen",
                        parameters=[estimator],
                    ),
                ]
            )
        actions.extend(
            [
                Node(
                    package="decentralized_swarm_integration",
                    executable="semantic_peer",
                    namespace=f"swarm/{uav}",
                    name="semantic_peer",
                    output="screen",
                    parameters=[
                        {
                            "uav_id": uav,
                            "detection_topic": detection_topic,
                            "estimate_topic": estimate_topic,
                            "status_topic": detection_status_topic,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
                Node(
                    package="decentralized_swarm_integration",
                    executable="role_peer",
                    namespace=f"swarm/{uav}",
                    name="role_peer",
                    output="screen",
                    parameters=[
                        {
                            "uav_id": uav,
                            "odom_topic": uav_odom_template.replace("{uav}", uav),
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
                Node(
                    package="decentralized_swarm_integration",
                    executable="mission_peer",
                    namespace=f"swarm/{uav}",
                    name="mission_peer",
                    output="screen",
                    parameters=[
                        {
                            "uav_id": uav,
                            "slot": index,
                            "output_enabled": control_enabled,
                            "use_sim_time": use_sim_time,
                        }
                    ],
                ),
            ]
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_names", default_value="uav0,uav1,uav2"),
            DeclareLaunchArgument(
                "px4_connection_urls",
                default_value="udpin:0.0.0.0:14540,udpin:0.0.0.0:14541,udpin:0.0.0.0:14542",
            ),
            DeclareLaunchArgument(
                "px4_map_origins", default_value="3:0:0.35,3:3:0.35,3:6:0.35"
            ),
            DeclareLaunchArgument("start_px4", default_value="false"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("control_enabled", default_value="false"),
            DeclareLaunchArgument("start_yolo", default_value="false"),
            DeclareLaunchArgument("start_camera_bridge", default_value="true"),
            DeclareLaunchArgument("yolo_weights", default_value=_default_yolo_weights()),
            DeclareLaunchArgument("yolo_device", default_value="auto"),
            DeclareLaunchArgument("yolo_evidence_root", default_value=""),
            DeclareLaunchArgument(
                "camera_topic_template", default_value="/swarm/{uav}/camera/image_raw"
            ),
            DeclareLaunchArgument(
                "camera_info_topic_template", default_value="/swarm/{uav}/camera/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_topic_template", default_value="/swarm/{uav}/camera/depth_image"
            ),
            DeclareLaunchArgument("ugv_odom_topic", default_value="/odom"),
            DeclareLaunchArgument(
                "uav_odom_topic_template", default_value="/swarm/{uav}/ground_truth/odom"
            ),
            DeclareLaunchArgument(
                "perception_pose_topic_template", default_value="/swarm/{uav}/ground_truth/pose"
            ),
            DeclareLaunchArgument("pose_bridge_port", default_value="5555"),
            DeclareLaunchArgument("metrics_base_port", default_value="5556"),
            OpaqueFunction(function=_build),
        ]
    )
