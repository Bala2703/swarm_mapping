"""
Coordinated exploration: one frontier_coordinator node plans for every robot.

It cuts the frontier of /map_merged into pieces, gives each robot a
different piece, sends it to that robot's own Nav2, and brings every robot
home when the map is done (swarm_bringup/frontier_coordinator.py). It
replaces explore_swarm.launch.py (one independent explore_lite per robot),
which sent all robots to the same frontier. Keep that one only as the
baseline to compare against.

Needs: slam_swarm.launch.py (SLAM, `world` frame, /map_merged) and
nav2_swarm.launch.py (every robot's bt_navigator active). Close the teleop
UI first: it would fight Nav2 for cmd_vel.

Usage:
    ros2 launch swarm_bringup coordinator.launch.py                   # all robots
    ros2 launch swarm_bringup coordinator.launch.py robots:=scout_1,mapper_1
In RViz, the "Coordinator" display shows the frontier (blue dots), the
pieces (green) and each robot's goal in that robot's colour.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots",
            default_value="",
            description="Comma-separated robots to coordinate; empty = every robot in fleet_config",
        ),
        Node(
            package="swarm_bringup",
            executable="frontier_coordinator",
            name="frontier_coordinator",
            output="screen",
            parameters=[{
                # Wall clock on purpose: the node takes sim time from the
                # /map_merged stamps instead of subscribing to /clock.
                "use_sim_time": False,
                "robots": ParameterValue(LaunchConfiguration("robots"), value_type=str),
            }],
        ),
    ])
