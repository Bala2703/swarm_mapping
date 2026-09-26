"""
Starts Nav2 for every robot in the swarm, each in its own namespace, so all
robots can navigate at the same time.

Why not nav2_bringup/navigation_launch.py: it remaps /tf -> tf inside the
namespace (one TF tree per robot with plain frame names). This project keeps
a single global /tf with prefixed frames (scout_1/base_link, ...), so that
remap would leave Nav2 blind. Same Nav2 nodes, just without the /tf remap.

Per robot, config/nav2_params.yaml gets <robot> and <footprint> filled in and
is nested under the robot's namespace. The global costmap plans on
/map_merged in `world`, so run map_merge_node (with use_sim_time:=true)
alongside slam_swarm.launch.py. Close the teleop UI while Nav2 drives: it
publishes cmd_vel at 10 Hz and would fight Nav2 for the selected robot.

Usage:
    ros2 launch swarm_bringup nav2_swarm.launch.py                 # all robots
    ros2 launch swarm_bringup nav2_swarm.launch.py robots:=scout_1
Send a goal: RViz "2D Goal Pose" with its topic set to /<robot>/goal_pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

from swarm_bringup.fleet_config import FLEET

# Footprint polygons around base_link (the wheel axle), from the
# rover.urdf.xacro dimensions including chassis_offset_x, plus ~1 cm margin.
FOOTPRINTS = {
    "scout": "[[0.14, 0.12], [0.14, -0.12], [-0.08, -0.12], [-0.08, 0.12]]",
    "mapper": "[[0.25, 0.195], [0.25, -0.195], [-0.13, -0.195], [-0.13, 0.195]]",
}

LIFECYCLE_NODES = [
    "controller_server",
    "smoother_server",
    "planner_server",
    "behavior_server",
    "bt_navigator",
    "waypoint_follower",
    "velocity_smoother",
]


def robot_nav2(name, robot_class, params_path):
    params = ParameterFile(
        RewrittenYaml(
            source_file=ReplaceString(
                source_file=params_path,
                replacements={"<robot>": name, "<footprint>": FOOTPRINTS[robot_class]},
            ),
            root_key=name,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    def nav2_node(package, executable, remappings=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            namespace=name,
            output="screen",
            parameters=[params],
            remappings=list(remappings),
        )

    return [
        # controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel (the bridge)
        nav2_node("nav2_controller", "controller_server", [("cmd_vel", "cmd_vel_nav")]),
        nav2_node("nav2_smoother", "smoother_server"),
        nav2_node("nav2_planner", "planner_server"),
        nav2_node("nav2_behaviors", "behavior_server"),
        nav2_node("nav2_bt_navigator", "bt_navigator"),
        nav2_node("nav2_waypoint_follower", "waypoint_follower"),
        nav2_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=name,
            output="screen",
            parameters=[
                {"use_sim_time": True, "autostart": True, "node_names": LIFECYCLE_NODES}
            ],
        ),
    ]


def launch_setup(context):
    params_path = os.path.join(
        get_package_share_directory("swarm_bringup"), "config", "nav2_params.yaml"
    )
    classes = {name: robot_class for name, robot_class, *_ in FLEET}
    wanted = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",")]
    actions = []
    for name in filter(None, wanted):
        if name not in classes:
            raise RuntimeError(f"unknown robot '{name}'; FLEET has {list(classes)}")
        actions += robot_nav2(name, classes[name], params_path)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots",
            default_value=",".join(name for name, *_ in FLEET),
            description="Comma-separated robot names to start Nav2 for",
        ),
        OpaqueFunction(function=launch_setup),
    ])
