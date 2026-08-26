"""Independent YOLO, 3D estimation, semantic and role peers for DJI0/1/2."""

from __future__ import annotations

import copy
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _params(node_name):
    share = get_package_share_directory("lrs_halmstad")
    with open(os.path.join(share, "config", "run_follow_defaults.yaml"), encoding="utf-8") as stream:
        data = yaml.safe_load(stream)
    result = copy.deepcopy(data.get("/**", {}).get("ros__parameters", {}))
    result.update(copy.deepcopy(data[node_name]["ros__parameters"]))
    return result


def _build(context):
    weights = LaunchConfiguration("yolo_weights").perform(context)
    evidence_root = LaunchConfiguration("evidence_root").perform(context)
    device = LaunchConfiguration("device").perform(context)
    detector_base = _params("leader_detector")
    estimator_base = _params("leader_estimator")
    actions = []
    for index in range(3):
        uav = f"dji{index}"
        detection = f"/coord/support/{uav}/leader_detection"
        status = f"/coord/support/{uav}/leader_detection_status"
        estimate = f"/coord/support/{uav}/leader_estimate"
        detector = copy.deepcopy(detector_base)
        detector.update({
            "use_sim_time": False,
            "uav_name": uav,
            "camera_topic": f"/{uav}/camera0/image_raw",
            "out_topic": detection,
            "status_topic": status,
            "event_topic": f"/coord/swarm/{uav}/perception_events",
            "publish_events": False,
            "backend": "ultralytics",
            "device": device,
            "yolo_weights": weights,
            "target_class_name": "ugv",
            "target_class_id": -1,
            "conf_threshold": 0.05,
            "iou_threshold": 0.5,
            "predict_hz": 0.5,
            "async_inference": True,
            "latest_frame_only": True,
            "evidence_root": evidence_root,
            "evidence_positive_interval_s": 2.0,
            "evidence_negative_interval_s": 20.0,
            "evidence_jpeg_quality": 85,
            "evidence_max_positive": 500,
            "evidence_max_negative": 100,
        })
        estimator = copy.deepcopy(estimator_base)
        estimator.update({
            "use_sim_time": False,
            "uav_name": uav,
            "camera_topic": f"/{uav}/camera0/image_raw",
            "camera_info_topic": f"/{uav}/camera0/camera_info",
            "depth_topic": f"/{uav}/camera0/depth_image",
            "uav_pose_topic": f"/{uav}/pose",
            "external_detection_topic": detection,
            "external_detection_status_topic": status,
            "out_topic": estimate,
            "status_topic": f"/coord/support/{uav}/leader_estimate_status",
            "event_topic": f"/coord/swarm/{uav}/estimate_events",
            "radio_range_topic": f"/coord/swarm/{uav}/network/radio_distance_m",
            "cam_pitch_offset_deg": -45.0,
            "est_hz": 2.0,
        })
        actions.extend([
            Node(package="lrs_halmstad", executable="leader_detector", name=f"{uav}_leader_detector", output="screen", parameters=[detector]),
            Node(package="lrs_halmstad", executable="leader_estimator", name=f"{uav}_leader_estimator", output="screen", parameters=[estimator]),
            Node(
                package="dji_swarm_integration", executable="semantic_peer",
                namespace=f"swarm/{uav}", name="semantic_peer", output="screen",
                parameters=[{
                    "use_sim_time": False, "uav_id": uav,
                    "detection_topic": detection, "status_topic": status,
                    "estimate_topic": estimate,
                }],
            ),
            Node(
                package="dji_swarm_integration", executable="role_peer",
                namespace=f"swarm/{uav}", name="role_peer", output="screen",
                parameters=[{
                    "use_sim_time": False, "uav_id": uav,
                    "camera_capable": True, "lidar_capable": False,
                    "odom_topic": f"/{uav}/pose/odom",
                }],
            ),
        ])
    return actions


def generate_launch_description():
    default_weights = os.path.join(
        get_package_share_directory("lrs_halmstad"), "weights", "baylands-leader-v4-2-best.pt"
    )
    return LaunchDescription([
        DeclareLaunchArgument("yolo_weights", default_value=default_weights),
        DeclareLaunchArgument("evidence_root", default_value=""),
        DeclareLaunchArgument("device", default_value="auto"),
        OpaqueFunction(function=_build),
    ])

