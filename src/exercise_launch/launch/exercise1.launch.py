import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # 1. Get the path to the TurtleBot4 bringup package
    pkg_turtlebot4_gz_bringup = get_package_share_directory('turtlebot4_gz_bringup')

    # 2. Include the Gazebo world launch file
    include_sim_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'sim.launch.py')
        )
    )

    # 3. Include the robot spawner launch file (This is the critical part)
    include_spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_gz_bringup, 'launch', 'turtlebot4_spawn.launch.py')
        )
    )

    # 4. Create the Node action for teleop_twist_keyboard
    #    This will use the version you compiled in ~/ros2_ws
    node_teleop_twist = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'stamped': True}], # TB4 needs unstamped Twist
    )

    # 5. Return all three components
    return LaunchDescription([
        include_sim_world,
        include_spawn_robot,
        node_teleop_twist
    ])
