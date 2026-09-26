"""
Brings up the swarm (via spawn_swarm.launch.py) and adds one
async_slam_toolbox_node per robot, each mapping independently into its
own namespaced frame tree.

This does NOT merge the per-robot maps into a single shared map yet —
that's the next piece. For now each robot in `ros2 topic list` gets its
own /<name>/map, useful on its own for verifying SLAM is working before
merging is layered on top.

Usage:
    ros2 launch swarm_bringup slam_swarm.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from swarm_bringup.fleet_config import FLEET


def generate_launch_description():
    pkg_share = get_package_share_directory("swarm_bringup")
    slam_params_path = os.path.join(pkg_share, "config", "slam_toolbox_params.yaml")

    # --- bring up the world + all robots + bridges, unchanged ---
    spawn_swarm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "spawn_swarm.launch.py")
        )
    )

    nodes = [spawn_swarm]

    for name, robot_class, x, y, yaw in FLEET:
        nodes.append(
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                namespace=name,
                name="slam_toolbox",
                output="screen",
                parameters=[
                    slam_params_path,
                    {
                        # frame_id strings are plain data, not ROS graph
                        # names, so namespacing doesn't auto-apply to
                        # them the way it does to "scan" below —
                        # they're set explicitly here per robot.
                        "odom_frame": f"{name}/odom",
                        "base_frame": f"{name}/base_link",
                        "map_frame": f"{name}/map",
                        # relative topic: resolves to /<name>/scan
                        # because this node's namespace is <name>.
                        "scan_topic": "scan",
                    },
                ],
                remappings=[
                    ("/map", f"/{name}/map"),
                    ("/map_metadata", f"/{name}/map_metadata"),
                    ("/map_updates", f"/{name}/map_updates"),
                ],
            )
        )

    return LaunchDescription(nodes)
