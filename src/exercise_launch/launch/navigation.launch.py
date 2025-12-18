import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Phase 4 Launch File: Nav2 Navigation Setup
    
    Launches the base simulation (Gazebo, robot spawn, bridge, robot_state_publisher)
    with EKF for state estimation (publishing odom->base_link transform) and
    Nav2 stack for autonomous navigation using a pre-built map.
    """
    
    # Get package directories
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    
    # Default map path (from Phase 3 SLAM output)
    default_map_path = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'maps', 'house_map.yaml')
    
    # Paths to config files
    ekf_config_path = os.path.join(pkg_exercise_launch, 'config', 'ekf_slam_params.yaml')
    nav2_params_path = os.path.join(pkg_exercise_launch, 'config', 'nav2_params.yaml')
    default_rviz_config_path = os.path.join(pkg_exercise_launch, 'rviz', 'navigation.rviz')
    
    # Launch arguments
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=default_map_path,
        description='Full path to map yaml file to load'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config_path,
        description='Path to the RViz configuration file'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='If true, start RViz2'
    )

    use_goal_bridge_arg = DeclareLaunchArgument(
        'use_goal_bridge',
        default_value='true',
        description='If true, start nav2_goal_bridge (RViz goal tool -> Nav2 action)'
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
        period=6.0,  # Delay to ensure clock topic is publishing
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
    
    # Nav2 Bringup - includes all Nav2 components (map_server, amcl, planner, controller, etc.)
    # Note: All launch_arguments values must be strings, not LaunchConfiguration objects
    # Start immediately - Nav2's lifecycle manager will handle proper startup sequencing
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'params_file': str(nav2_params_path),
            'map': LaunchConfiguration('map'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': 'true',
        }.items()
    )
    
    # Topic throttle node to reduce scan rate for RViz (prevents message queue overflow)
    # Nav2 still gets full-rate scans from /scan, RViz gets throttled /scan_throttled
    scan_throttle_node = Node(
        package='topic_tools',
        executable='throttle',
        name='scan_throttle',
        arguments=['messages', '/scan', '5', '/scan_throttled'],  # Throttle to 5Hz for RViz
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen'
    )
    
    # Nav2 Goal Bridge Node - bridges RViz2's "2D Goal Pose" tool to Nav2 action server
    # This allows clicking on the map in RViz2 to set navigation goals
    # Delayed start to ensure Nav2 is ready
    nav2_goal_bridge_node = TimerAction(
        period=12.0,  # Start after Nav2 bringup has initialized
        actions=[
            Node(
                package='exercise_launch',
                executable='nav2_goal_bridge',
                name='nav2_goal_bridge',
                output='screen',
                parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
                condition=IfCondition(LaunchConfiguration('use_goal_bridge')),
            )
        ]
    )
    
    # RViz2 Node - visualize map, robot, costmaps, paths, and enable goal setting
    # The navigation.rviz config includes "2D Goal Pose" tool for clicking on map
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )
    
    return LaunchDescription([
        map_arg,
        use_sim_time_arg,
        rviz_config_arg,
        use_rviz_arg,
        use_goal_bridge_arg,
        simulation_launch,
        scan_throttle_node,
        ekf_node,
        nav2_bringup_launch,
        nav2_goal_bridge_node,
        rviz_node,
    ])
