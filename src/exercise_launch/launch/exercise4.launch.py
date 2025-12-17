"""
Exercise 4 - Bag Playback with EKF (robot_localization)

This launch file plays back a recorded bag file and runs the Extended Kalman Filter
to fuse odometry and IMU data, visualizing the result in RViz.

Usage:
1. First record a bag using: ros2 launch exercise_launch exercise4_record.launch.py
2. Play back with EKF: ros2 launch exercise_launch exercise4.launch.py bag_path:=<path_to_bag>

The RViz visualization shows:
- Raw odometry (orange arrows)
- Filtered odometry from EKF (green arrows with covariance)
- Robot model
- Laser scan
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():

    # Get package directories
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    pkg_turtlebot4_description = get_package_share_directory('turtlebot4_description')
    
    # Paths to config files
    ekf_config_path = os.path.join(pkg_exercise_launch, 'config', 'ekf_params.yaml')
    rviz_config_path = os.path.join(pkg_exercise_launch, 'rviz', 'exercise4.rviz')
    urdf_path = os.path.join(pkg_turtlebot4_description, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    # Declare launch arguments
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='exercise4_recording',
        description='Path to the recorded bag file'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )

    # Play back the bag file with clock publishing.
    # We let the bag provide /tf and /tf_static directly, and we will
    # configure the EKF not to publish TF (publish_tf: false) so there
    # is no conflict. This restores the original Exercise 4 behaviour:
    # TF tree comes from the recording, EKF just outputs /odometry/filtered.
    play_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_path'),
            '--clock',
            '--rate', LaunchConfiguration('rate'),
        ],
        output='screen'
    )

    # Robot state publisher for TF tree (needed for RViz robot model)
    # Use xacro to process the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # EKF node from robot_localization
    # Delayed start to ensure bag playback has begun
    ekf_node = TimerAction(
        period=2.0,
        actions=[
            Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
                parameters=[ekf_config_path],
                remappings=[
                    ('odometry/filtered', '/odometry/filtered'),
                ]
            )
        ]
    )

    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        rate_arg,
        play_bag,
        robot_state_publisher,
        ekf_node,
        rviz_node,
    ])
