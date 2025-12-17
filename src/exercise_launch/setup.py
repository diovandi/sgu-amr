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
        
        # Install world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),

        # Install config files (YAML and XML)
        (os.path.join('share', package_name, 'config'), glob('config/*.xml') + glob('config/*.yaml')),

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
        ],
    },
)
