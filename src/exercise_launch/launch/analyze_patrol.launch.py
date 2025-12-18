"""
Phase 6 - Playback & analysis for patrol bag.

- Plays back the bag with --clock
- Does NOT run EKF (TF is in the bag)
- Starts RViz + PlotJuggler
- Starts patrol_analysis node to publish planned vs actual overlays/streams
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node


def generate_launch_description():
    pkg_exercise_launch = get_package_share_directory("exercise_launch")
    
    default_rviz_config = os.path.join(pkg_exercise_launch, "rviz", "patrol_analysis.rviz")
    default_map_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'house_map.yaml')

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="/home/dio/ros2_ws/patrol_data",
        description="Path to the recorded patrol bag directory",
    )

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=default_map_path,
        description="Path to the map yaml file to load for analysis",
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=default_rviz_config,
        description="RViz config for offline patrol analysis",
    )

    rate_arg = DeclareLaunchArgument(
        "rate",
        default_value="1.0",
        description="Playback rate multiplier",
    )

    # Path to the specialized analysis URDF (with fixed joints for playback)
    urdf_path = os.path.join(pkg_exercise_launch, "urdf", "rudimentary_bot_analysis.urdf.xacro")
    
    # Fallback to absolute path if not yet installed in share directory
    if not os.path.exists(urdf_path):
        urdf_path = "/home/dio/ros2_ws/src/exercise_launch/urdf/rudimentary_bot_analysis.urdf.xacro"

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )

    play_bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "play",
            LaunchConfiguration("bag_path"),
            "--clock",
            "--rate",
            LaunchConfiguration("rate"),
        ],
        output="screen",
    )

    # Start analysis node shortly after playback starts so /clock is available.
    analysis_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package="exercise_launch",
                executable="patrol_analysis",
                name="patrol_analysis",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    plotjuggler = ExecuteProcess(
        cmd=["ros2", "run", "plotjuggler", "plotjuggler"],
        output="screen",
    )

    # Add Map Server to provide the static background map
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': True, 'yaml_filename': LaunchConfiguration('map')}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True, 'node_names': ['map_server']}]
    )

    # Robot State Publisher - simplified for analysis
    # This now uses the URDF with fixed joints, ensuring wheels stay attached
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description,
            'use_tf_static': False,   # Force all TFs to /tf to ensure they overlay correctly
            'ignore_timestamp': True
        }]
    )

    return LaunchDescription(
        [
            bag_path_arg,
            map_arg,
            rviz_config_arg,
            rate_arg,
            play_bag,
            analysis_node,
            rviz_node,
            plotjuggler,
            map_server_node,
            lifecycle_manager_node,
            robot_state_publisher_node,
        ]
    )
