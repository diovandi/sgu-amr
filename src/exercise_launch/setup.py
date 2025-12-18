import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'exercise_launch'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # Install world files (both .world and .sdf)
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world') + glob('worlds/*.sdf')),

        # Install config files (YAML and XML)
        (os.path.join('share', package_name, 'config'), glob('config/*.xml') + glob('config/*.yaml')),

        # Install URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf*')),

        # Install model files (pillar for Exercise 3)
        (os.path.join('share', package_name, 'models', 'pillar'), glob('models/pillar/*.sdf')),

        # Install RViz config files (for Exercise 4)
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),

        # Install launch wrapper scripts (for Cosmic/NVIDIA compatibility)
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dio',
    maintainer_email='dio@todo.todo',
    description='ROS 2 exercises for TurtleBot4 simulation',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # This creates the 'pillar_driver' executable
            'pillar_driver = exercise_launch.pillar_driver:main',
            # Phase 2: LIDAR reader node
            'lidar_reader = exercise_launch.lidar_reader:main',
            # Phase 4: Nav2 goal bridge (connects RViz2 goal tool to Nav2 action server)
            'nav2_goal_bridge = exercise_launch.nav2_goal_bridge:main',
            # Phase 5: one-time waypoint capture from RViz
            'patrol_waypoint_recorder = exercise_launch.patrol_waypoint_recorder:main',
            # Phase 5: automated Nav2 patrol
            'house_patrol = exercise_launch.house_patrol:main',
            # Phase 6: offline analysis helper (planned vs actual)
            'patrol_analysis = exercise_launch.patrol_analysis:main',
        ],
    },
)
