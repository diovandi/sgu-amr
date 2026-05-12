import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, TextSubstitution, PythonExpression
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

def generate_launch_description():
    
    # Launch argument to control static TF publisher (for Phase 2 compatibility)
    # When use_ekf_tf=true, EKF will handle odom->base_link transform (Phase 3+)
    # When use_ekf_tf=false, static transform is used (Phase 2)
    use_ekf_tf_arg = DeclareLaunchArgument(
        'use_ekf_tf',
        default_value='false',
        description='If true, skip static odom->base_link transform (EKF will handle it)'
    )
    
    # Get package directories
    pkg_exercise_launch = get_package_share_directory('exercise_launch')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths to files
    world_path = PathJoinSubstitution([
        pkg_exercise_launch,
        'worlds',
        'home.sdf'
    ])
    urdf_path = PathJoinSubstitution([
        pkg_exercise_launch,
        'urdf',
        'rudimentary_bot.urdf.xacro'
    ])
    gz_bridge_params_path = os.path.join(
        pkg_exercise_launch,
        'config',
        'gz_bridge.yaml'
    )
    
    # Process URDF with xacro
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str
    )
    
    # Set environment variable for GUI compatibility
    set_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    
    # Launch Gazebo with the world file using ros_gz_sim
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': [
                world_path,
                TextSubstitution(text=' -r -v 1')
            ],
            'on_exit_shutdown': 'true'
        }.items()
    )
    
    # Spawn robot after a delay to ensure Gazebo is ready
    spawn_robot = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                name='spawn_rudimentary_bot',
                arguments=[
                    '-name', 'rudimentary_bot',
                    '-topic', 'robot_description',
                    '-x', '0.0',
                    '-y', '0.0',
                    '-z', '0.1',  # Spawn at wheel axle height (chassis is raised)
                    '-Y', '0.0'
                ],
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )
    
    # Joint State Publisher - publishes joint states for continuous joints (wheels)
    # This is needed so robot_state_publisher can publish transforms for wheel links
    # When source_list is not provided, it defaults to publishing default joint states (zeros)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    
    # Robot State Publisher - publishes TF tree from URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': robot_description
        }]
    )
    
    # Topic bridge using parameter_bridge with YAML config
    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )
    
    # Static transform publisher for odom -> base_link
    # Since publish_odom_tf=false (EKF will handle this later), we need a temporary
    # static transform for Phase 2 so RViz can visualize the robot
    # Only include this if use_ekf_tf=false (Phase 2 mode)
    static_tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_odom_base',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link'],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(PythonExpression(["'", LaunchConfiguration('use_ekf_tf'), "' == 'false'"]))
    )
    
    return LaunchDescription([
        use_ekf_tf_arg,
        set_env,
        world_launch,
        joint_state_publisher,
        robot_state_publisher,
        spawn_robot,
        gz_bridge_node,
        static_tf_odom,
    ])
