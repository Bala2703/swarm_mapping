"""
Starts Nav2 for every robot in the swarm, each in its own namespace, so all
robots can navigate at the same time.

Why not nav2_bringup/navigation_launch.py: it remaps /tf -> tf inside the
namespace (one TF tree per robot with plain frame names). This project keeps
a single global /tf with prefixed frames (scout_1/base_link, ...), so that
remap would leave Nav2 blind. Same Nav2 nodes, just without the /tf remap.

By default each robot's Nav2 nodes run composed in ONE container process
(Nav2's own "use_composition" mode). With one process per node (24 for three
robots) Fast DDS discovery was slow enough at startup that a lifecycle
service reply got dropped ("client will not receive response") and that
robot's Nav2 hung half-configured. use_composition:=false restores one
process per node, which is handy for debugging a single server.

Per robot, config/nav2_params.yaml gets <robot> and <footprint> filled in and
is nested under the robot's namespace. Each robot plans on its own SLAM map
(/<robot>/map), so Nav2 itself only needs slam_swarm.launch.py; goals may be
given in any frame on the TF tree, e.g. `world`. Close the teleop UI while
Nav2 drives: it publishes cmd_vel at 10 Hz and would fight Nav2 for the
selected robot.

Usage:
    ros2 launch swarm_bringup nav2_swarm.launch.py                 # all robots
    ros2 launch swarm_bringup nav2_swarm.launch.py robots:=scout_1
    ros2 launch swarm_bringup nav2_swarm.launch.py use_composition:=false
Send a goal: RViz "2D Goal Pose" with its topic set to /<robot>/goal_pose.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import ReplaceString, RewrittenYaml

from swarm_bringup.fleet_config import FLEET

# Footprint polygons around base_link (the wheel axle), from the
# rover.urdf.xacro dimensions including chassis_offset_x, plus ~1 cm margin.
FOOTPRINTS = {
    "scout": "[[0.14, 0.12], [0.14, -0.12], [-0.08, -0.12], [-0.08, 0.12]]",
    "mapper": "[[0.25, 0.195], [0.25, -0.195], [-0.13, -0.195], [-0.13, 0.195]]",
}

# (package, executable, component plugin, remappings), in lifecycle order.
# controller -> cmd_vel_nav -> velocity_smoother -> cmd_vel (the bridge)
NAV2_SERVERS = [
    ("nav2_controller", "controller_server", "nav2_controller::ControllerServer",
     [("cmd_vel", "cmd_vel_nav")]),
    ("nav2_smoother", "smoother_server", "nav2_smoother::SmootherServer", []),
    ("nav2_planner", "planner_server", "nav2_planner::PlannerServer", []),
    ("nav2_behaviors", "behavior_server", "behavior_server::BehaviorServer", []),
    ("nav2_bt_navigator", "bt_navigator", "nav2_bt_navigator::BtNavigator", []),
    ("nav2_waypoint_follower", "waypoint_follower", "nav2_waypoint_follower::WaypointFollower", []),
    ("nav2_velocity_smoother", "velocity_smoother", "nav2_velocity_smoother::VelocitySmoother",
     [("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel")]),
]

LIFECYCLE_PARAMS = {
    "use_sim_time": True,
    "autostart": True,
    "node_names": [executable for _, executable, _, _ in NAV2_SERVERS],
    # Default 4 s: with 3 robots' Nav2 + SLAM + Gazebo starting at once, a
    # healthy but slow server missed it and that robot's whole bringup
    # aborted (bt_navigator never activated, so its goals were ignored).
    "bond_timeout": 20.0,
}


def robot_nav2(name, robot_class, params_path, use_composition):
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

    if use_composition:
        components = [
            ComposableNode(
                package=package,
                plugin=plugin,
                name=executable,
                namespace=name,
                parameters=[params],
                remappings=remappings,
            )
            for package, executable, plugin, remappings in NAV2_SERVERS
        ]
        components.append(
            ComposableNode(
                package="nav2_lifecycle_manager",
                plugin="nav2_lifecycle_manager::LifecycleManager",
                name="lifecycle_manager_navigation",
                namespace=name,
                parameters=[LIFECYCLE_PARAMS],
            )
        )
        # The params file also goes on the container itself, as nav2_bringup
        # does: nodes the servers create internally (local_costmap,
        # global_costmap) take their parameters from the process arguments.
        return [
            ComposableNodeContainer(
                name="nav2_container",
                namespace=name,
                package="rclcpp_components",
                executable="component_container_isolated",
                output="screen",
                parameters=[params, {"use_sim_time": True}],
                composable_node_descriptions=components,
            )
        ]

    nodes = [
        Node(
            package=package,
            executable=executable,
            name=executable,
            namespace=name,
            output="screen",
            parameters=[params],
            remappings=remappings,
        )
        for package, executable, _, remappings in NAV2_SERVERS
    ]
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=name,
            output="screen",
            parameters=[LIFECYCLE_PARAMS],
        )
    )
    return nodes


def launch_setup(context):
    params_path = os.path.join(
        get_package_share_directory("swarm_bringup"), "config", "nav2_params.yaml"
    )
    use_composition = LaunchConfiguration("use_composition").perform(context).lower() in (
        "true", "1", "yes"
    )
    classes = {name: robot_class for name, robot_class, *_ in FLEET}
    wanted = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",")]
    actions = []
    for name in filter(None, wanted):
        if name not in classes:
            raise RuntimeError(f"unknown robot '{name}'; FLEET has {list(classes)}")
        actions += robot_nav2(name, classes[name], params_path, use_composition)
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots",
            default_value=",".join(name for name, *_ in FLEET),
            description="Comma-separated robot names to start Nav2 for",
        ),
        DeclareLaunchArgument(
            "use_composition",
            default_value="true",
            description="Run each robot's Nav2 nodes in one component container",
        ),
        OpaqueFunction(function=launch_setup),
    ])
