"""Launch identical decentralized semantic peers for a UAV list."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _expand(template: str, uav: str) -> str:
    return template.replace("{uav}", uav)


def _build_peers(context):
    names = [
        item.strip()
        for item in LaunchConfiguration("uav_names").perform(context).split(",")
        if item.strip()
    ]
    if not names or len(names) != len(set(names)):
        raise ValueError("uav_names must contain unique, non-empty names")
    detection_template = LaunchConfiguration("detection_topic_template").perform(context)
    estimate_template = LaunchConfiguration("estimate_topic_template").perform(context)
    status_template = LaunchConfiguration("status_topic_template").perform(context)
    shared_topic = LaunchConfiguration("shared_topic").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    return [
        Node(
            package="decentralized_swarm_integration",
            executable="semantic_peer",
            namespace=f"swarm/{uav}",
            name="semantic_peer",
            output="screen",
            parameters=[
                {
                    "uav_id": uav,
                    "detection_topic": _expand(detection_template, uav),
                    "estimate_topic": _expand(estimate_template, uav),
                    "status_topic": _expand(status_template, uav),
                    "shared_topic": shared_topic,
                    "use_sim_time": use_sim_time,
                    "min_confidence": LaunchConfiguration("min_confidence"),
                    "min_independent_sources": LaunchConfiguration("min_independent_sources"),
                    "max_spread_m": LaunchConfiguration("max_spread_m"),
                    "allow_single_source": LaunchConfiguration("allow_single_source"),
                }
            ],
        )
        for uav in names
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("uav_names", default_value="uav"),
            DeclareLaunchArgument(
                "detection_topic_template",
                default_value="/coord/support/{uav}/leader_detection",
            ),
            DeclareLaunchArgument(
                "estimate_topic_template",
                default_value="/coord/support/{uav}/leader_estimate",
            ),
            DeclareLaunchArgument(
                "status_topic_template",
                default_value="/coord/support/{uav}/leader_detection_status",
            ),
            DeclareLaunchArgument(
                "shared_topic", default_value="/coord/swarm/semantic_observations"
            ),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("min_confidence", default_value="0.35"),
            DeclareLaunchArgument("min_independent_sources", default_value="2"),
            DeclareLaunchArgument("max_spread_m", default_value="3.0"),
            DeclareLaunchArgument("allow_single_source", default_value="false"),
            OpaqueFunction(function=_build_peers),
        ]
    )
