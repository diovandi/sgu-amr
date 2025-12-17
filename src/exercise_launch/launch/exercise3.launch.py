import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    # 1. Get our own package share directory
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    
    # 2. Get the TB4 bringup package share directory
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')

    # 3. Get the path to our custom RViz config for Exercise 3
    default_rviz_config_path = os.path.join(pkg_exercise_launch, 'rviz', 'exercise3.rviz')

    # 4. Path to our pillar model SDF
    pillar_model_path = os.path.join(pkg_exercise_launch, 'models', 'pillar', 'model.sdf')

    # 5. Include the Gazebo world; use the default TurtleBot 4 warehouse world
    include_sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'sim.launch.py')
        ),
        # NOTE: Do NOT override the world to 'empty' here. Leaving the 'world'
        # argument at its default lets turtlebot4_gz_bringup start the standard
        # warehouse world used in Exercise 4 as well.
        # If you ever need to be explicit, you can use:
        #   launch_arguments={'world': 'warehouse'}.items()
        # but we keep it unset so both exercises share the same default.
        launch_arguments={}
    )

    # 6. Include the robot spawner (turtlebot4_spawn.launch.py)
    include_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py')
        )
    )

    # 7. Spawn the pillar model into Gazebo using ros_gz_sim create.
    #    In the warehouse world, place the pillar a few meters in front of the
    #    robot so it is clearly visible and reachable without colliding with
    #    the dock.
    spawn_pillar = TimerAction(
        period=5.0,  # Wait 5 seconds for Gazebo to initialize
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-file', pillar_model_path,
                    '-name', 'pillar',
                    # Approximate pose tuned for the default TurtleBot 4
                    # warehouse spawn: a pillar ~2.5 m straight ahead of the
                    # robot, centered laterally on its x-axis.
                    '-x', '2.5',
                    '-y', '0.0',
                    '-z', '0.0',
                ],
                output='screen'
            )
        ]
    )

    # 8. Start RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': True}]
    )

    # 9. Start our P-controller node (delayed to ensure simulation is ready)
    # NOTE: We now use the LiDAR's native frame (no manual offset). Positive scan
    # angle and positive angular.z both correspond to "turn left", matching REP-103.
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
                    'search_angle': 1.57,      # ±90° search cone
                    'lidar_angle_offset': 1.5708,  # +π/2: rotate LiDAR frame 90° CCW
                    # Obstacle avoidance tuned to not fight the final stop
                    'obstacle_distance': 0.25,
                }]
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config_path,
            description='Path to the RViz configuration file'),
            
        include_sim_world,
        include_spawn_robot,
        spawn_pillar,
        rviz_node,
        pillar_driver_node
    ])
