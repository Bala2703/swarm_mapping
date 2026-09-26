"""
Autonomous exploration: one explore_lite node per robot.

Each explorer reads the shared /map_merged (frame `world`), finds frontiers
(boundaries between known free space and unknown space), picks the cheapest
one for its robot (closer and bigger is better) and sends it as a goal to
that robot's Nav2 (/<robot>/navigate_to_pose). It repeats until no frontiers
are left, then drives the robot back to where it started.

The robots do NOT coordinate: each picks its own best frontier, and in
practice they all pick the same one (the biggest), because explore_lite aims
every robot at that frontier's centroid. This is the baseline that a
coordinator has to beat.

Why not explore_lite's own launch file: like nav2_bringup, it remaps
/tf -> tf inside the namespace (one TF tree per robot). This project uses a
single global /tf with prefixed frames, so that remap is left out here.

Needs: slam_swarm.launch.py (SLAM, `world` frame, /map_merged) and
nav2_swarm.launch.py (every robot's bt_navigator active). Close the teleop
UI first: it would fight Nav2 for cmd_vel.

Usage:
    ros2 launch swarm_bringup explore_swarm.launch.py                 # all robots
    ros2 launch swarm_bringup explore_swarm.launch.py robots:=scout_1
Pause / resume one robot:
    ros2 topic pub --once /scout_1/explore/resume std_msgs/msg/Bool "{data: false}"
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from swarm_bringup.fleet_config import FLEET


def explorer(name):
    return Node(
        package="explore_lite",
        executable="explore",
        name="explore_node",
        namespace=name,
        output="screen",
        parameters=[{
            "use_sim_time": True,
            # Frames are plain strings, so the robot prefix is explicit. The
            # goal frame comes from the map's own header (`world`).
            "robot_base_frame": f"{name}/base_link",
            "costmap_topic": "/map_merged",
            "costmap_updates_topic": "/map_merged_updates",  # not published; full maps only
            "visualize": True,              # /<robot>/explore/frontiers markers
            "planner_frequency": 0.15,      # Hz; re-pick frontier while driving (upstream default)
            "progress_timeout": 30.0,       # s without progress -> blacklist that frontier
            # explore_lite's frontier cost is
            #   potential_scale * distance[m] * resolution - gain_scale * size[cells] * resolution
            # The distance is already in metres, so the extra *resolution
            # (0.05) makes it 20x too weak: at the upstream 3.0, 1 m of
            # frontier outweighed 6.7 m of travel and every robot picked the
            # same biggest frontier. At 30, 1 m of travel costs as much as
            # 1.5 m of frontier. That makes distance count, but it did not stop
            # robots sharing goals: with a few big frontiers, all still pick
            # the biggest.
            "potential_scale": 30.0,        # weight on distance to the frontier
            "orientation_scale": 0.0,
            "gain_scale": 1.0,              # weight on frontier size
            "transform_tolerance": 0.3,
            "min_frontier_size": 0.5,       # m; ignore frontiers smaller than this
            "return_to_init": True,         # drive back to the start when done
        }],
    )


def launch_setup(context):
    names = [name for name, *_ in FLEET]
    wanted = [r.strip() for r in LaunchConfiguration("robots").perform(context).split(",")]
    actions = []
    for name in filter(None, wanted):
        if name not in names:
            raise RuntimeError(f"unknown robot '{name}'; FLEET has {names}")
        actions.append(explorer(name))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "robots",
            default_value=",".join(name for name, *_ in FLEET),
            description="Comma-separated robot names to run an explorer for",
        ),
        OpaqueFunction(function=launch_setup),
    ])
