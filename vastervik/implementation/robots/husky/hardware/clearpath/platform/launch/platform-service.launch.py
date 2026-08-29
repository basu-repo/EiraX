from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():

    setup_path = str(Path(__file__).resolve().parents[2])

    # Include Packages
    pkg_clearpath_common = FindPackageShare('clearpath_common')

    # Declare launch files
    launch_file_platform = PathJoinSubstitution([
        pkg_clearpath_common, 'launch', 'platform.launch.py'])

    # Include launch files
    launch_platform = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([launch_file_platform]),
        launch_arguments=
            [
                (
                    'setup_path'
                    ,
                    setup_path
                )
                ,
                (
                    'use_sim_time'
                    ,
                    'true'
                )
                ,
                (
                    'namespace'
                    ,
                    'a201_0000'
                )
                ,
                (
                    'enable_ekf'
                    ,
                    'true'
                )
                ,
                (
                    'use_manipulation_controllers'
                    ,
                    'true'
                )
                ,
            ]
    )

    # Nodes
    node_cmd_vel_bridge = Node(
        name='cmd_vel_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a201_0000',
        output='screen',
        arguments=
            [
                'a201_0000/cmd_vel@geometry_msgs/msg/TwistStamped[gz.msgs.Twist'
                ,
                '/model/a201_0000/robot/cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist'
                ,
            ]
        ,
        remappings=
            [
                (
                    'a201_0000/cmd_vel'
                    ,
                    'cmd_vel'
                )
                ,
                (
                    '/model/a201_0000/robot/cmd_vel'
                    ,
                    'platform/cmd_vel'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    node_odom_base_tf_bridge = Node(
        name='odom_base_tf_bridge',
        executable='parameter_bridge',
        package='ros_gz_bridge',
        namespace='a201_0000',
        output='screen',
        arguments=
            [
                '/model/a201_0000/robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
                ,
            ]
        ,
        remappings=
            [
                (
                    '/model/a201_0000/robot/tf'
                    ,
                    'tf'
                )
                ,
            ]
        ,
        parameters=
            [
                {
                    'use_sim_time': True
                    ,
                }
                ,
            ]
        ,
    )

    # Create LaunchDescription
    ld = LaunchDescription()
    ld.add_action(launch_platform)
    ld.add_action(node_cmd_vel_bridge)
    ld.add_action(node_odom_base_tf_bridge)
    return ld
