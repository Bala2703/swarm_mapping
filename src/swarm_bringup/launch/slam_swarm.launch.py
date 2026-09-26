"""
Brings up the swarm (via spawn_swarm.launch.py) and adds one
async_slam_toolbox_node per robot, each mapping independently into its
own namespaced frame tree.

It also publishes a static world -> <name>/map transform per robot (its
fleet_config spawn pose), so all robots share one `world` root frame —
the frame map_merge_node publishes /map_merged in and RViz uses as its
fixed frame. The merge node itself is still started separately.

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

        # --- anchor this robot's map frame in the shared `world` frame ---
        # slam_toolbox's <name>/map starts at the robot's spawn pose, so the
        # world -> <name>/map offset is exactly its fleet_config pose. This
        # joins every robot's TF tree under one root.
        nodes.append(
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=name,
                name="world_to_map",
                arguments=[
                    "--x", str(x),
                    "--y", str(y),
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", str(yaw),
                    "--frame-id", "world",
                    "--child-frame-id", f"{name}/map",
                ],
            )
        )

    return LaunchDescription(nodes)
