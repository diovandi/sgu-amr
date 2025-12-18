import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Phase 2 Launch File: Sensor Processing
    
    Launches the base simulation (Gazebo, robot spawn, bridge, robot_state_publisher)
    and adds the LIDAR reader node for processing sensor data.
    Also launches RViz2 for visualizing the LIDAR scan.
    """
    
    # Get package directories
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    
    # Path to RViz config
    default_rviz_config_path = os.path.join(pkg_exercise_launch, 'rviz', 'phase2.rviz')
    
    # Launch arguments
    log_frequency_arg = DeclareLaunchArgument(
        'log_frequency',
        default_value='10',
        description='Log LIDAR statistics every N scans (default: 10)'
    )
    
    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Enable verbose logging (default: false)'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )
    
    # Include the base simulation launch file
    # This starts: Gazebo, robot spawn, bridge, robot_state_publisher
    simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_exercise_launch, 'launch', 'simulation.launch.py')
        )
    )
    
    # LIDAR Reader Node - processes /scan topic with Best Effort QoS
    lidar_reader_node = Node(
        package='exercise_launch',
        executable='lidar_reader',
        name='lidar_reader_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'log_frequency': LaunchConfiguration('log_frequency'),
            'verbose': LaunchConfiguration('verbose'),
        }]
    )
    
    # RViz2 Node - visualize LIDAR scan
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )
    
    return LaunchDescription([
        log_frequency_arg,
        verbose_arg,
        rviz_config_arg,
        simulation_launch,
        lidar_reader_node,
        rviz_node,
    ])
