# Swarm SLAM and Map Merging Execution Guide

## Overview
To successfully visualize the swarm, all individual robot TF trees must be connected to a single unified `world` frame. Without this, RViz can only display one robot at a time, resulting in `queue is full` warnings from unconnected TF trees.

**⚠️ RViz Humble Bug Warning:** `MapDisplay::fixedFrameChanged()` does not update its TF filter. If you change the Fixed Frame dropdown in RViz while it is running, you **must** toggle the map display off and on for it to update correctly.

---

## Run Instructions

Open a new terminal for each of the following steps. Ensure you have sourced your ROS 2 workspace in every terminal.

### 1. Launch the SLAM Swarm
Start the main SLAM bringup for the fleet:
```bash
ros2 launch swarm_bringup slam_swarm.launch.py
```

### 2. Publish Static Transforms (The "World" Frame)
Currently, we must manually link each robot's map to the `world` frame using their spawn poses from `fleet_config`. Run these three commands in separate terminals (or send them to the background):
```bash
# Terminal 2: Scout 1
ros2 run tf2_ros static_transform_publisher --x -3 --y 3 --frame-id world --child-frame-id scout_1/map

# Terminal 3: Scout 2
ros2 run tf2_ros static_transform_publisher --x -3 --y -3 --frame-id world --child-frame-id scout_2/map

# Terminal 4: Mapper 1
ros2 run tf2_ros static_transform_publisher --frame-id world --child-frame-id mapper_1/map
```
*(Note: Automating this step is the next immediate development goal—see the Development Tasks section).*

### 3. Start the Map Merge Node
Run the merge node, explicitly using sim time to fix wall-clock timestamp issues:
```bash
ros2 run swarm_bringup map_merge_node --ros-args -p use_sim_time:=true
```

### 4. Launch RViz
Open RViz using the provided configuration file (also using sim time):
```bash
rviz2 -d ~/Documents/rmcl-robotics-1/swarm_ws/src/swarm_bringup/rviz/swarm_mapping.rviz --ros-args -p use_sim_time:=true
```

---

## How to Read RViz Output (Diagnostics)

Use these visual cues to diagnose system performance:

*   **Ghost or Double Walls:**
    *   *In the merged map ONLY:* The merge node is the problem (see Known Issues below).
    *   *In a robot's individual map:* The robot's SLAM or the simulation physics is drifting.
*   **Laser Scans:**
    *   Points should land exactly on walls.
    *   *Rotated/Offset scans:* Indicates that the specific robot's SLAM has drifted.
    *   *Short arc of points on the floor:* The robot is physically tilting/pitching backward.
*   **Pose Graph:**
    *   A smooth, continuous chain indicates healthy SLAM.
    *   A long edge cutting across the map or a sudden, sharp jump indicates a bad loop closure.
*   **Measure Tool:**
    *   True SDF walls are exactly **0.2 m thick**.
    *   If walls in the merged map measure thicker than 0.2 m, it is due to error accumulation.

---

## Known Issues & Current Limitations

### 1. Map Merge Accumulation (High Priority)
*   **Symptom:** The occupied cells grow over time (up to 2.2× true thickness), leaving hundreds of ghost wall cells behind after loop closures.
*   **Cause:** The merge node never clears `_merged_data`.
*   **Required Fix:** Update the code to keep the latest map from each robot and completely rebuild the merged grid from scratch on every publish cycle.

### 2. Per-Robot SLAM Drift
*   **Symptom:** Individual maps (e.g., `scout_1`) can drift significantly (e.g., 2m off, expanding the map to 275 cells when it should be max 236).
*   **Suspected Causes:**
    1.  **Robot Tipping:** The rover balances on its wheel axle with only a front caster. It tips backward easily, tilting the lidar ~13° into the floor.
    2.  **Manual Driving Collisions:** `cmd_vel` never times out and wheel torque is unlimited, causing aggressive collisions when driven manually.
    3.  **Loose Loop-Closure:** Current `slam_toolbox` settings may be accepting bad match candidates too easily.

### 3. Yaw-Related Holes (Dormant)
*   **Note:** `col*res` is correct for `slam_toolbox` (which centers its cells). Do not add `+0.5`. 
*   **Issue:** Holes in the map only appear when a robot's spawn yaw is not exactly `0`. At a 0.3 rad yaw, hole occurrence is about 6.4%. Since spawn yaws are currently 0, this is dormant.

---

## Upcoming Development Tasks

1.  **Automate Static Transforms:** Add a loop in `spawn_swarm.launch.py` to iterate over `FLEET` and automatically publish the `world -> <name>/map` transforms (mimicking the existing lidar static-TF loop).
2.  **Install RViz Config via Setup.py:** To ensure symlink-install picks up the RViz folder, update `setup.py`:
    *   Add this to `data_files`: `(os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz'))`
    *   Rebuild the workspace.