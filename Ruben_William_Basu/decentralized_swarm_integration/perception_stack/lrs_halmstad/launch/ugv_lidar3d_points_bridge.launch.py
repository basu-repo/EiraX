from launch import LaunchDescription
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    points_bridge = RosGzBridge(
        bridge_name='ugv_lidar3d_0_points_bridge',
        use_composition=False,
        extra_bridge_params={
            'bridges': {
                'points': {
                    'ros_topic_name': '/a201_0000/sensors/lidar3d_0/points',
                    'ros_type_name': 'sensor_msgs/msg/PointCloud2',
                    'gz_topic_name': '/a201_0000/sensors/lidar3d_0/scan/points',
                    'gz_type_name': 'gz.msgs.PointCloudPacked',
                    'direction': 'GZ_TO_ROS',
                    'lazy': False,
                },
            },
            'bridge_names': ['points'],
        },
    )

    # Keep the bridged Gazebo pointcloud frame compatible with the previous
    # Clearpath projection setup. The Baylands per-waypoint pc2ls height
    # windows were tuned against this effective frame, so do not shift it to
    # lidar3d_0_laser unless those route settings are retuned.
    lidar_static_tf = Node(
        name='lidar3d_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a201_0000',
        output='screen',
        arguments=[
            '--frame-id', 'lidar3d_0_link',
            '--child-frame-id', 'a201_0000/robot/base_link/lidar3d_0',
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        points_bridge,
        lidar_static_tf,
    ])
