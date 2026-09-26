"""
Spawns a heterogeneous ground-rover swarm into Gazebo Fortress.

Edit the FLEET list in swarm_bringup/fleet_config.py to change how many
robots you spawn and what class each one is — both this launch file and
slam_swarm.launch.py read from that one place, so scaling from 3 robots
to 30 is a one-line change there.

Usage:
    ros2 launch swarm_bringup spawn_swarm.launch.py
"""

import os
import xml.etree.ElementTree as ET

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from swarm_bringup.fleet_config import FLEET, WORLD_NAME


def generate_launch_description():
    pkg_share = get_package_share_directory("swarm_bringup")
    world_path = os.path.join(pkg_share, "worlds", f"{WORLD_NAME}.sdf")
    xacro_path = os.path.join(pkg_share, "urdf", "rover.urdf.xacro")

    # --- bring up Gazebo Fortress itself, loading the test world ---
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": f"-r {world_path}"}.items(),
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"],
    )

    nodes = [gz_sim, clock_bridge]

    for name, robot_class, x, y, yaw in FLEET:
        # --- render this robot's URDF from the shared xacro macro ---
        robot_description = xacro.process_file(
            xacro_path,
            mappings={"robot_name": name, "robot_class": robot_class},
        ).toxml()

        # --- publish its TF tree, namespaced and prefixed so N robots
        #     don't clash on /tf ---
        nodes.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=name,
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "frame_prefix": f"{name}/",
                        "use_sim_time": True,
                    }
                ],
            )
        )

        # --- spawn the model into the running Gazebo world ---
        nodes.append(
            Node(
                package="ros_gz_sim",
                executable="create",
                namespace=name,
                output="screen",
                arguments=[
                    "-name",
                    name,
                    "-topic",
                    "robot_description",
                    "-x",
                    str(x),
                    "-y",
                    str(y),
                    "-z",
                    "0.1",
                    "-Y",
                    str(yaw),
                ],
            )
        )

        # --- wheel frames: static transforms read from the URDF ---
        # The wheel joints are continuous, so robot_state_publisher only
        # publishes them when joint states arrive. Gazebo's
        # JointStatePublisher sent those at 1 kHz (see the xacro), and a
        # Python joint_state_publisher on sim time still burned ~45% CPU
        # just receiving the 1 kHz /clock. The wheels are only drawn, never
        # used by SLAM or Nav2, so publish each one once, at rest.
        # (ign_ros2_control would bring real joint states back.)
        for joint in ET.fromstring(robot_description).iter("joint"):
            if joint.get("type") != "continuous":
                continue
            origin = joint.find("origin")
            jx, jy, jz = origin.get("xyz", "0 0 0").split()
            jroll, jpitch, jyaw = origin.get("rpy", "0 0 0").split()
            nodes.append(
                Node(
                    package="tf2_ros",
                    executable="static_transform_publisher",
                    namespace=name,
                    name=f"{joint.get('name')}_static_tf",
                    arguments=[
                        "--x", jx,
                        "--y", jy,
                        "--z", jz,
                        "--roll", jroll,
                        "--pitch", jpitch,
                        "--yaw", jyaw,
                        "--frame-id", f"{name}/{joint.find('parent').get('link')}",
                        "--child-frame-id", f"{name}/{joint.find('child').get('link')}",
                    ],
                )
            )

        # --- bridge this robot's Gazebo topics to ROS 2 ---
        # Format: <gz_topic>@<ros_type><dir><gz_type>, where <dir> is
        # "[" for Gazebo -> ROS and "]" for ROS -> Gazebo. ("@" would be
        # both ways, which pushed all of /tf back into Gazebo.) Fortress
        # uses the "ignition.msgs.*" namespace for gz_type; on
        # Harmonic/Jetty swap that prefix for "gz.msgs.*".
        bridge_args = [
            f"/model/{name}/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            f"/model/{name}/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry",
            f"/model/{name}/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU",
            f"/world/{WORLD_NAME}/model/{name}/lidar"
            f"@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            f"/model/{name}/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V",
        ]

        nodes.append(
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                namespace=name,
                name="gz_bridge",
                output="screen",
                arguments=bridge_args + ["--ros-args", "-p", "use_sim_time:=true"],
                remappings=[
                    (f"/model/{name}/cmd_vel", "cmd_vel"),
                    (f"/model/{name}/odometry", "odom"),
                    (f"/model/{name}/imu", "imu"),
                    (f"/world/{WORLD_NAME}/model/{name}/lidar", "scan"),
                    (f"/model/{name}/tf", "/tf"),
                ],
            )
        )
        # No lidar static TF here: the xacro's <ignition_frame_id> stamps
        # scans with <name>/lidar_link, which robot_state_publisher already
        # publishes from the URDF.

    return LaunchDescription(nodes)
