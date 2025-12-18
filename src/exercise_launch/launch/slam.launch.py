import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Phase 3 Launch File: SLAM Setup
    
    Launches the base simulation (Gazebo, robot spawn, bridge, robot_state_publisher)
    with EKF for state estimation (publishing odom->base_link transform) and
    SLAM Toolbox for online mapping. Also launches RViz2 for visualization.
    """
    
    # Get package directories
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    
    # Paths to config files
    ekf_config_path = os.path.join(pkg_exercise_launch, 'config', 'ekf_slam_params.yaml')
    slam_config_path = os.path.join(pkg_exercise_launch, 'config', 'slam_params.yaml')
    default_rviz_config_path = os.path.join(pkg_exercise_launch, 'rviz', 'slam.rviz')
    
    # Launch arguments
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )
    
    # Include the base simulation launch file
    # Pass use_ekf_tf=true to skip static transform (EKF will handle it)
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_exercise_launch, 'launch', 'simulation.launch.py')
        ),
        launch_arguments={
            'use_ekf_tf': 'true'  # EKF will publish odom->base_link transform
        }.items()
    )
    
    # EKF Node - fuses /odom and /imu, publishes odom->base_link transform
    # Delayed start to ensure simulation, bridge, and clock are ready
    ekf_node = TimerAction(
        period=6.0,  # Increased delay to ensure clock topic is publishing
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
    
    # SLAM Toolbox Node - async_online_mapper for building map
    # Delayed start to ensure EKF is running and publishing transforms
    # NOTE: slam_toolbox is a lifecycle node and must be activated
    slam_node = TimerAction(
        period=10.0,  # Increased delay to give EKF more time to stabilize and start publishing
        actions=[
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
                parameters=[slam_config_path]
            )
        ]
    )
    
    # Lifecycle Manager for SLAM Toolbox
    # Automatically configures and activates the SLAM node
    slam_lifecycle_manager = TimerAction(
        period=12.0,  # Start after SLAM node is launched
        actions=[
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='slam_lifecycle_manager',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'autostart': True,
                    'node_names': ['slam_toolbox']
                }]
            )
        ]
    )
    
    # Topic throttle node to reduce scan rate for RViz (prevents message queue overflow)
    # SLAM Toolbox still gets full-rate scans from /scan, RViz gets throttled /scan_throttled
    scan_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        arguments=['messages', '/scan', '5', '/scan_throttled'],  # Throttle to 5Hz for RViz
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    # RViz2 Node - visualize map, robot, LIDAR, and TF tree
    # Use throttled scan topic to prevent message queue overflow
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}],
        output='screen'
    )
    
    return LaunchDescription([
        rviz_config_arg,
        simulation_launch,
        scan_throttle_node,
        ekf_node,
        slam_node,
        slam_lifecycle_manager,  # Activates SLAM node automatically
        rviz_node,
    ])
