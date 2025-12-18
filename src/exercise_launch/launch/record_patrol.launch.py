"""
Phase 5 - Record autonomous house patrol (Nav2).

Includes navigation.launch.py, runs the patrol script, and records a rosbag.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_exercise_launch = get_package_share_directory("exercise_launch")

    bag_path_arg = DeclareLaunchArgument(
        "bag_path",
        default_value="/home/dio/ros2_ws/patrol_data",
        description="Directory/name for the recorded bag output",
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_exercise_launch, "launch", "navigation.launch.py")),
        launch_arguments={
            # Run headless for recording by default.
            "use_rviz": "false",
            "use_goal_bridge": "false",
        }.items(),
    )

    # Start the patrol after Nav2 has had some time to activate.
    patrol_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package="exercise_launch",
                executable="house_patrol",
                name="house_patrol",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    record_bag = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            LaunchConfiguration("bag_path"),
            "/odom",
            "/odometry/filtered",
            "/amcl_pose",
            "/scan",
            "/tf",
            "/tf_static",
            "/plan",
            "/cmd_vel",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            bag_path_arg,
            navigation_launch,
            record_bag,
            patrol_node,
        ]
    )

