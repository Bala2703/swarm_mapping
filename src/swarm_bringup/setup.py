import os
from glob import glob
from setuptools import setup

package_name = 'swarm_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'),
            glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Heterogeneous ground-rover swarm bringup for Gazebo Fortress',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'map_merge_node = swarm_bringup.map_merge_node:main',
        'swarm_teleop = swarm_bringup.swarm_teleop:main',
    ]},
)
