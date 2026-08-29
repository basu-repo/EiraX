"""ROS endpoints connecting DJI and Husky poses to the isolated OMNeT model."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="dji_swarm_integration", executable="multi_pose_bridge",
            name="dji_multi_pose_bridge", output="screen",
            parameters=[{
                "use_sim_time": False,
                "model_names": ["ugv", "dji0", "dji1", "dji2"],
                "odom_topics": ["/odom", "/dji0/pose/odom", "/dji1/pose/odom", "/dji2/pose/odom"],
                "port": 5555, "stale_timeout_s": 3.0,
            }],
        ),
        Node(
            package="dji_swarm_integration", executable="multi_metrics_bridge",
            name="dji_multi_metrics_bridge", output="screen",
            parameters=[{
                "use_sim_time": False,
                "endpoints": ["dji0:5556", "dji1:5557", "dji2:5558"],
            }],
        ),
    ])

