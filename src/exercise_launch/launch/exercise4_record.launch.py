"""
Exercise 4 - Bag Recording Launch File

This launch file starts the TurtleBot4 simulation with teleop and records
the relevant topics for later playback with the EKF.

Usage:
1. Launch this file: ros2 launch exercise_launch exercise4_record.launch.py
2. Drive the robot around using the teleop keyboard
3. Press Ctrl+C to stop recording
4. The bag file will be saved in the current directory

Topics recorded:
- /odom: Wheel odometry
- /imu: IMU data
- /scan: LIDAR scan
- /tf: Transform data
- /tf_static: Static transforms
- /robot_description: Robot URDF
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # Get package share directories
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')
    pkg_exercise_launch = get_package_share_directory('exercise_launch')

    # Declare bag output path argument
    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='exercise4_recording',
        description='Path/name for the recorded bag file'
    )

    # Path to the pillar model SDF (same model used in Exercise 3)
    pillar_model_path = os.path.join(pkg_exercise_launch, 'models', 'pillar', 'model.sdf')

    # Include the Gazebo simulation. We intentionally do NOT override the
    # `world` argument here so that turtlebot4_gz_bringup launches its default
    # warehouse world, matching Exercise 3.
    include_sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'sim.launch.py')
        )
    )

    # Include the robot spawner
    include_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py')
        )
    )

    # Spawn the same pillar model used in Exercise 3 into the warehouse world so
    # the autonomous controller has a consistent target during recording.
    spawn_pillar = TimerAction(
        period=5.0,  # Wait for Gazebo to initialize
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', pillar_model_path,
                    '-name', 'pillar',
                    # Pose tuned for the default TurtleBot 4 warehouse spawn:
                    # place the pillar a few meters in front of the robot,
                    # centered laterally.
                    '-x', '2.5',
                    '-y', '0.0',
                    '-z', '0.0',
                ],
                output='screen',
            )
        ],
    )

    # Autonomous pillar-following controller from Exercise 3. Running this
    # during Exercise 4 recording lets the bag capture the full autonomous
    # sequence (startup backoff, crash/avoidance, approach to pillar) in the
    # warehouse environment.
    pillar_driver_node = TimerAction(
        period=8.0,  # Wait for pillar to spawn
        actions=[
            Node(
                package='exercise_launch',
                executable='pillar_driver',
                name='pillar_driver_node',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'linear_gain': 0.5,
                    'angular_gain': 2.0,
                    'stop_distance': 0.5,
                    'search_angle': 1.57,        # ±90° search cone
                    'lidar_angle_offset': 1.5708,  # +π/2: rotate LiDAR frame 90° CCW
                    'obstacle_distance': 0.25,
                }],
            )
        ],
    )

    # Record bag file with relevant topics from the autonomous run
    record_bag = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('bag_path'),
            '/odom',
            '/imu',
            '/scan', 
            '/tf',
            '/tf_static',
            '/robot_description',
            '/joint_states',
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_path_arg,
        include_sim_world,
        include_spawn_robot,
        spawn_pillar,
        pillar_driver_node,
        record_bag,
    ])

