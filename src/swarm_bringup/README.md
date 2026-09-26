# swarm_bringup

Spawns a heterogeneous ground-rover swarm (scout + mapper classes) into
Gazebo Fortress on ROS 2 Humble, for multi-robot exploration/mapping work.

## What's in here

```
swarm_bringup/
├── launch/
│   ├── spawn_swarm.launch.py      # bring-up: world + N robots + bridges
│   └── slam_swarm.launch.py       # spawn_swarm + one slam_toolbox per robot
├── swarm_bringup/fleet_config.py  # FLEET roster — the one place to edit
├── config/slam_toolbox_params.yaml
├── urdf/rover.urdf.xacro          # one macro, two classes: scout / mapper
├── worlds/exploration_test.sdf    # multi-room test world
└── package.xml / setup.py         # ament_python package plumbing
```

Every file has been checked for well-formedness (`xacro` processes both
the scout and mapper variants cleanly, the SDF and YAML are valid,
every launch file compiles, `fleet_config` imports cleanly) but **has
not been run against a live Gazebo Fortress instance** — I don't have
that installed in this environment. Treat this as a solid,
spec-accurate starting point, not a guaranteed first-try success. See
"If something doesn't come up" below.

## Prerequisites

- ROS 2 Humble, sourced (`source /opt/ros/humble/setup.bash`)
- Gazebo Fortress + the ROS integration packages:
  ```bash
  sudo apt install ros-humble-ros-gz ros-humble-ros-gz-sim \
                    ros-humble-ros-gz-bridge ros-humble-robot-state-publisher \
                    ros-humble-xacro ros-humble-slam-toolbox
  ```

## Build

```bash
mkdir -p ~/swarm_ws/src
cp -r swarm_bringup ~/swarm_ws/src/
cd ~/swarm_ws
colcon build --packages-select swarm_bringup
source install/setup.bash
```

## Run

```bash
# base swarm only (no mapping)
ros2 launch swarm_bringup spawn_swarm.launch.py

# swarm + per-robot SLAM
ros2 launch swarm_bringup slam_swarm.launch.py
```

`slam_swarm.launch.py` includes `spawn_swarm.launch.py` and adds one
`async_slam_toolbox_node` per robot on top — you don't run both, the
SLAM launch brings up everything the base one does. Edit the `FLEET`
list in `swarm_bringup/fleet_config.py` to change the roster — each
entry is just `(name, class, x, y, yaw)` — both launch files read from
this one place.

## Verifying SLAM is working

```bash
ros2 topic list | grep map        # expect /scout_1/map, /scout_2/map, /mapper_1/map
ros2 run tf2_tools view_frames     # per robot, or run inside each namespace

# drive a rover around manually and watch its map fill in
ros2 topic pub /scout_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.3}}" --rate 10
ros2 run rviz2 rviz2   # add a Map display on topic /scout_1/map,
                        # fixed frame scout_1/map
```

Each robot maps independently right now — there's no merged map yet.
If `/scout_1/map` never appears: check `ros2 topic echo /scout_1/scan
--once` first (no scan data means slam_toolbox has nothing to work
with, and the fix is upstream in the bridge, not in SLAM config).

## Verifying the base swarm worked

```bash
# Gazebo's own view of things
ign topic -l | grep scout_1        # or `gz topic -l` if it resolves the same

# ROS's view, per robot
ros2 topic list | grep scout_1
ros2 topic echo /scout_1/scan --once
ros2 topic echo /scout_1/odom --once

# drive one rover manually
ros2 topic pub /scout_1/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}, angular: {z: 0.0}}" --rate 10
```

If `/scout_1/scan` or `/scout_1/odom` don't show up, the most likely
culprits, in order of likelihood:

1. **Sensor topic path mismatch.** The lidar's Gazebo-side topic is
   `/world/exploration_test/model/<name>/lidar` — this depends on
   Gazebo's own topic-scoping rules and is the one string in this repo
   I'd least bet my life on being byte-exact on the first try. Run
   `ign topic -l` after spawning and grep for `lidar`; if the actual
   topic differs, update it in both `rover.urdf.xacro` (the `<topic>`
   tag under the lidar sensor) and `spawn_swarm.launch.py`
   (`bridge_args`) to match.
2. **Bridge didn't start** — check the terminal output for the
   `gz_bridge` node under each robot's namespace for connection errors.
3. **Sensors plugin not loaded** — confirm `exploration_test.sdf` is
   the world actually being loaded (not some other default world).

## Next steps (per the roadmap)

- Merge the per-robot maps into one shared map. Since we know every
  robot's spawn pose (it's right there in `fleet_config.py`), this is
  simpler than the general "unknown initial position" map-merge
  problem — no feature-matching required, just overlay each robot's
  occupancy grid at its known fixed offset. Worth writing this as a
  small custom node rather than pulling in the community
  `m-explore-ros2` package's `multirobot_map_merge`, which needs to be
  built from source and whose slam_toolbox support (vs. the older
  gmapping) currently lives on an experimental branch.
- Add a frontier-detection + claim-assignment node per robot so scouts
  don't converge on the same target once the merged map exists.
- Swap the native `DiffDrive` plugin for `ign_ros2_control` +
  `diff_drive_controller` once you're ready to match your real
  rovers' `ros2_control` hardware-interface pattern exactly.
