# Swarm Mapping — Project Guide

*Status as of 26 Sep 2026. Written for someone who knows basic ROS 2
(nodes, topics, launch files, TF) but is new to this project.*

> Older docs predate the fixes described here and are partly out of date:
> `how_to_run_swarm_mapping.md` (repo root), and `README.md` / `HANDOFF.md`
> in `src/swarm_bringup/`. When they disagree with this guide, trust this one.

---

## 1. What this project is

Three small ground rovers explore a building **together** and build **one
shared map**, in simulation. It runs on **ROS 2 Humble** with **Gazebo
Fortress**. That pairing was chosen on purpose: the real target is 2 physical
rovers that run Humble, so the sim uses the same ROS version.

Each robot:

1. drives around and **builds its own map** with SLAM (`slam_toolbox`);
2. contributes that map to a **merged map** of the whole building;
3. can be sent to any point and **drives there by itself** with Nav2,
   avoiding walls and obstacles.

The end goal is **autonomous swarm exploration**. The robots decide where to
go next, split the work between them, and stop when the building is fully
mapped. That last part is not built yet (see [section 10](#10-whats-next)).

---

## 2. Where we are right now

| Area | Status |
| --- | --- |
| Simulation: world + 3 robots spawn and drive | ✅ Working |
| Each robot builds its own map (SLAM) | ✅ Working |
| Shared `world` frame joining all robots | ✅ Working |
| Merged map of all robots (`/map_merged`) | ✅ Working |
| RViz view of everything in one window | ✅ Working |
| Nav2 on all 3 robots at the same time (click a goal, robot drives) | ✅ Working |
| A saved map of the whole world (`maps/merged.pgm`) | ✅ 98% of the floor mapped |
| **Autonomous exploration (robots pick their own goals)** | ⏭️ **Next step** |
| Coordination (robots split the work, don't chase the same area) | ⬜ Not started |
| Same motor-control interface as the real rovers (`ros2_control`) | ⬜ Not started |
| Running on the 2 real rovers | ⬜ Not started |

**Measured health (latest runs):**

- **Tilt:** robots stay level while driving (0.0° pitch).
- **SLAM corrections:** usually 1–3 cm, sometimes 10–20 cm on the scouts.
- **Nav2:** three goals sent at once to three robots all succeeded.
- **CPU:** load about 5 on a 16-core machine (it started around 50).

---

## 3. The world and the robots

**World** (`worlds/exploration_test.sdf`): a closed 12 × 12 m room. The outer
walls are at x = ±6 m and y = ±6 m, and all walls are 1 m tall. Two interior
walls split it into areas connected by gaps:

- a horizontal wall at y = 2, from x = −4 to x = 2
- a vertical wall at x = 3, from y = −4 to y = 1

**Robots** (listed in `swarm_bringup/fleet_config.py`, built from one xacro
with two "classes"):

| Name | Class | Spawn (x, y, yaw) | Lidar | Max speed | Chassis |
| --- | --- | --- | --- | --- | --- |
| `scout_1` | scout | (−3, 3, 0) | 6 m range, 360 beams, 10 Hz | 1.2 m/s, 2.0 rad/s | 20 × 16 cm, 1.2 kg |
| `scout_2` | scout | (−3, −3, 0) | 6 m range, 360 beams, 10 Hz | 1.2 m/s, 2.0 rad/s | 20 × 16 cm, 1.2 kg |
| `mapper_1` | mapper | (0, 0, 0) | 12 m range, 720 beams, 10 Hz | 0.6 m/s, 1.0 rad/s | 36 × 28 cm, 4.5 kg |

Scouts are small and fast with a short-range lidar. The mapper is bigger and
slower with a long-range lidar. All robots are differential drive: two
wheels plus one front caster. They also carry an IMU, which is bridged to ROS
but not used yet. The dimensions are placeholders until the real rovers are
measured.

To change the team, edit the `FLEET` list in `fleet_config.py`. Every launch
file and the merge node read from it.

---

## 4. How the pieces fit together

```
                         ┌──────────────────── Gazebo Fortress ────────────────────┐
                         │ physics · lidar · IMU · DiffDrive (drives the wheels,   │
                         │ computes wheel odometry)                                │
                         └───────────▲──────────────────────────────┬──────────────┘
                                     │ /model/<r>/cmd_vel           │ lidar, odometry, IMU, TF
                               ros_gz_bridge (one per robot, one-way per topic)
                                     │                              ▼
   RViz "2D Goal Pose"     /<r>/cmd_vel                 /<r>/scan  /<r>/odom  /tf
           │                    ▲                            │
           ▼                    │                            ▼
   /<r>/goal_pose ──►  Nav2 (one per robot)  ◄── /<r>/map ── slam_toolbox (one per robot)
                        plans on its own map                 builds /<r>/map and the
                        avoids obstacles with                <r>/map → <r>/odom transform
                        its own live scan                           │
                                                                    ▼
                                                     map_merge_node (one, shared)
                                                     all /<r>/map ──► /map_merged (frame: world)
```

`<r>` stands for a robot name (`scout_1`, `scout_2`, `mapper_1`). Everything
per-robot lives in that robot's **namespace**, so each robot has its own
`/scout_1/scan`, `/scout_1/map` and so on.

### What each part does

- **Gazebo** simulates the physics and sensors. Its DiffDrive plugin turns
  `cmd_vel` into wheel motion and reports wheel odometry.
- **ros_gz_bridge** copies topics between Gazebo and ROS. Each topic goes one
  way only: sensors, odometry and TF go Gazebo → ROS; `cmd_vel` goes
  ROS → Gazebo.
- **slam_toolbox**, one per robot, turns that robot's lidar scans plus its
  odometry into a map (`/<r>/map`). It also keeps correcting where the robot
  is. Wheel odometry slowly drifts, and SLAM fixes it by matching scans.
- **map_merge_node**, which is our own code, overlays all robots' maps into
  one grid, `/map_merged`, in the shared `world` frame.
- **Nav2**, one stack per robot, takes a goal and plans a path on the robot's
  own map. It follows the path while dodging obstacles seen in the robot's
  live scan, and outputs `cmd_vel`.

---

## 5. Frames (TF): the one concept to get right

There is **one** `/tf` topic for all robots. Frame names are **prefixed**
with the robot name. That's because ROS namespaces only rename *topics*; they
don't rename frame IDs inside messages, so the prefix has to be set
explicitly everywhere.

```
world                                   ← shared frame (origin = middle of the room)
├── scout_1/map                         ← world→map: static, = scout_1's spawn pose
│   └── scout_1/odom                    ← map→odom: published by slam_toolbox (drift correction)
│       └── scout_1/base_link           ← odom→base: Gazebo DiffDrive (wheel odometry), via the bridge
│           ├── scout_1/chassis_link    ← fixed joints: robot_state_publisher, from the URDF
│           │   ├── scout_1/lidar_link  ← the lidar; scans are stamped with this frame
│           │   └── scout_1/imu_link
│           ├── scout_1/caster_link
│           ├── scout_1/left_wheel_link   ← wheels: static transforms from spawn_swarm.launch.py
│           └── scout_1/right_wheel_link
├── scout_2/map  → … same shape …
└── mapper_1/map → … same shape …
```

**Why the `world` frame exists.** Without it, each robot's tree is a separate
island. Nothing can relate `scout_1`'s position to `scout_2`'s, and RViz can
show only one robot at a time. Each robot's map frame starts exactly where
the robot spawned. We know those spawn poses (they're in `fleet_config.py`),
so `world → <r>/map` is a fixed transform. It's published in
`slam_swarm.launch.py`.

**Why positions can look odd in logs.** Nav2 reports a robot's position in
that robot's own map frame, where the spawn point is (0, 0). Goals sent from
RViz are in `world`. For example, `scout_2` at (0, 0) in its own frame is at
(−3, −3) in `world`.

---

## 6. Repository layout

```
swarm_ws/                        ← git repo = ROS 2 workspace
├── PROJECT_GUIDE.md             ← this file
├── src/swarm_bringup/           ← the only package (ament_python)
│   ├── swarm_bringup/
│   │   ├── fleet_config.py      ← the robot list (name, class, spawn pose)
│   │   ├── map_merge_node.py    ← our merged-map node
│   │   └── swarm_teleop.py      ← keyboard driving UI (PyQt)
│   ├── launch/
│   │   ├── spawn_swarm.launch.py  ← Gazebo, robots, bridges, wheel frames
│   │   ├── slam_swarm.launch.py   ← includes spawn_swarm, adds slam_toolbox per robot + world frame
│   │   └── nav2_swarm.launch.py   ← Nav2 for every robot
│   ├── config/
│   │   ├── slam_toolbox_params.yaml
│   │   └── nav2_params.yaml       ← shared by all robots; <robot> and <footprint> filled in per robot
│   ├── urdf/rover.urdf.xacro      ← one robot model, two classes
│   ├── worlds/exploration_test.sdf
│   ├── rviz/swarm_mapping.rviz    ← ready-made RViz layout
│   ├── maps/merged.pgm, merged.yaml  ← saved baseline map
│   ├── setup.py, package.xml
│   └── README.md, HANDOFF.md      ← older notes (partly outdated)
└── build/ install/ log/           ← colcon output (git-ignored)
```

---

## 7. How to run it

**Once, and after adding new files:**

```bash
cd ~/Documents/rmcl-robotics-1/swarm_ws
colcon build --symlink-install --packages-select swarm_bringup
```

The build uses `--symlink-install`, so edits to *existing* files take effect
without rebuilding. *New* files need one rebuild.

**Then run each step in its own terminal**, after
`source install/setup.bash`:

| # | Command | What it starts |
| --- | --- | --- |
| 1 | `ros2 launch swarm_bringup slam_swarm.launch.py` | Gazebo, 3 robots, bridges, SLAM ×3, `world` frame |
| 2 | `ros2 run swarm_bringup map_merge_node --ros-args -p use_sim_time:=true` | the merged map |
| 3 | `rviz2 -d ~/Documents/rmcl-robotics-1/swarm_ws/src/swarm_bringup/rviz/swarm_mapping.rviz --ros-args -p use_sim_time:=true` | the RViz view |
| 4 | `ros2 launch swarm_bringup nav2_swarm.launch.py` | Nav2 for all 3 robots (start it after the robots have appeared) |

**Check that Nav2 is ready.** Your robots only accept goals once this
prints `active [3]` for each one:

```bash
for r in scout_1 scout_2 mapper_1; do ros2 lifecycle get /$r/bt_navigator; done
```

**Drive the robots, one of two ways:**

- **With Nav2 (recommended).** The RViz toolbar has three *2D Goal Pose*
  buttons. In order, they send goals to `scout_1`, `scout_2` and `mapper_1`.
  Click one, then click and drag on the map. You can give all three robots
  goals at the same time.
- **By keyboard.** Run `ros2 run swarm_bringup swarm_teleop`, pick a robot,
  click the drive pad, and use `i`/`,`/`j`/`l`. Press `k` to stop.
  - **Close the teleop before using Nav2.** It sends commands 10 times a
    second and would fight Nav2.

**Save the merged map:**

```bash
ros2 run nav2_map_server map_saver_cli -t /map_merged -f <name> --ros-args -p use_sim_time:=true
```

**What you see in RViz:**

- **Fixed frame:** `world`.
- **Merged map:** grey is free, black is walls.
- **Each robot's group:**
  - lidar scan, drawn in the robot's colour (orange, blue or green)
  - robot model
  - SLAM pose graph
  - Nav2 plan
  - footprint
  - local costmap
  - two displays that start off and can be ticked on: its own SLAM map and
    its global costmap

---

## 8. Design decisions, and the problems that led to them

These are not arbitrary. Each one fixed a real, measured problem. Please
don't undo them without reading the reason.

| # | Symptom we saw | Cause | Fix (where) |
| --- | --- | --- | --- |
| 1 | Merged map got worse over time: thick and doubled walls, ghost trails | The merge node painted every new map *on top of* the old result and never cleared anything. SLAM redraws its whole map as it corrects itself, so every old wall position stayed. | Keep only each robot's **latest** map and rebuild the merged grid **from scratch** every second (`map_merge_node.py`) |
| 2 | Maps full of phantom lines; SLAM jumping 20–44 cm | The rovers **tipped backwards** onto their tail (15° scouts, 12.9° mapper) because the center of mass sat almost exactly over the wheel axle. The tilted lidar hit the floor about 0.5 m behind the robot and drew it as a wall. | Chassis moved 3 cm (scouts) or 6 cm (mapper) ahead of the axle; frictionless caster; acceleration limits in both directions (`rover.urdf.xacro`) |
| 3 | Only one robot visible in RViz at a time | Each robot's TF tree was a separate island | Static `world → <r>/map` transforms from the spawn poses (`slam_swarm.launch.py`) |
| 4 | Lidar frame needed a hand-typed transform that broke when the robot changed | Fortress ignores the SDF `<frame_id>` tag | `<ignition_frame_id>` in the xacro, so scans use `<r>/lidar_link` from the URDF |
| 5 | CPU overloaded (load ~50) | Bridges were two-way, echoing all TF back into Gazebo. Gazebo also published wheel joint states 1000 times a second, flooding `/tf`. | One-way bridges; wheel frames published once as static transforms (`spawn_swarm.launch.py`) |
| 6 | Nav2 couldn't see the robots | Nav2's standard launch file assumes a separate TF tree per robot, which clashes with our shared `/tf` | Our own `nav2_swarm.launch.py` without that assumption |
| 7 | Nav2 refused to plan: "Starting point in lethal space" | Robots see each other with their lidars, so the merged map sometimes contained a robot's own body as an obstacle | Nav2 plans on each robot's **own** map, where it never appears; `/map_merged` is kept for choosing where to explore |
| 8 | Early goals failed: "no valid path found" | A young SLAM map is small, so goals just outside it weren't in Nav2's costmap | Global costmap is a 20 m window around the robot; unmapped cells count as "unknown", which the planner may cross |
| 9 | Occasional planner failure with the default planner (NavFn) | A known path-tracing weakness in NavFn | Switched to the **Smac 2D** planner (`nav2_params.yaml`) |
| 10 | One robot randomly never accepted goals | With 24 separate Nav2 processes starting at once, the middleware (Fast DDS) dropped a startup message, so that robot's Nav2 hung half-started | Each robot's Nav2 runs as **one process** (composition); startup timeout raised to 20 s (`nav2_swarm.launch.py`) |

**The map merge works from known start poses.** It doesn't try to discover
where the robots are relative to each other, because we already know. That
keeps the merge simple and exact in sim. On real robots, the start poses will
have to be measured carefully, or the merge will need a correction step.

---

## 9. Known limitations

- **Scout SLAM still wobbles a bit.** Corrections of 10–20 cm happen
  sometimes, mostly from wheel-odometry heading drift in turns and the
  scouts' short 6 m lidar in open areas. Planned fixes: separate SLAM
  settings for scouts and mapper, and later fusing the IMU into odometry.
- **Robots appear in each other's maps.** The scouts' lidars see the mapper
  and each other's lidar tops. The mapper's lidar passes over the scouts, so
  **the mapper cannot see the scouts** and could bump into them.
- **The merge node uses about 45% of a CPU core.** It is plain Python looping
  over every cell, and it receives the 1000 Hz sim clock. A NumPy rewrite
  would make it much cheaper.
- **Wheels don't turn in RViz.** They're drawn at rest, which is intentional.
  See decision 5.
- **The test world is small and symmetric.** The robots see about 80% of it
  from where they spawn. A bigger world would be a better exploration test.
- **The RViz config and saved map aren't installed** by `setup.py`. Load them
  from `src/`, as shown above.

---

## 10. What's next

1. **Autonomous exploration with `explore_lite`**, the standard ROS 2 Humble
   frontier explorer.
   - Each robot looks at `/map_merged` and finds the nearest "frontier",
     meaning the edge between known and unknown space.
   - It sends that frontier to its own Nav2 as a goal, and repeats until
     nothing unknown is left.
   - Robots won't coordinate yet, so two may chase the same frontier. Their
     time to map 95% of the floor becomes the **baseline** to beat.
2. **Coordination:** the actual swarm part. Options include:
   - robots "claiming" frontiers so others skip them
   - scoring frontiers by distance and expected new area
   - optimal robot-to-frontier assignment (the Hungarian method)

   Each must beat the baseline.
3. **Better SLAM and merge:**
   - per-class SLAM settings
   - a ground-truth map score, so changes are measured
   - erasing robots from the merged map using their known positions
   - a NumPy merge node
4. **`ros2_control`:** drive the sim through the same controller interface
   the real rovers use.
5. **The real rovers:**
   - real dimensions in the xacro
   - sensor noise in the sim
   - a procedure for measuring start poses
   - networking and time sync between machines

---

## 11. Troubleshooting

| Problem | Likely cause and fix |
| --- | --- |
| A robot ignores goals from RViz | Its Nav2 isn't active. Run the `ros2 lifecycle get` check above. If it isn't `active [3]`, restart `nav2_swarm.launch.py` after the sim is fully up. |
| RViz says "Frame [world] does not exist" | `slam_swarm.launch.py` isn't running; it publishes the `world` frame. |
| No merged map in RViz | `map_merge_node` isn't running, or was started without `use_sim_time:=true`. |
| A robot won't drive straight or jerks | The teleop UI is still open and fighting Nav2. Close it. |
| Robots, topics or TF missing entirely | Check the middleware settings: `env \| grep -i -E 'rmw\|zenoh'`. All terminals must use the same one. This project uses Humble's default, Fast DDS. |
| Everything is slow and Nav2 warns about "missed its desired rate" | CPU overload. Close other heavy programs and check with `top`. The Gazebo GUI alone costs about one core. |
| Nav2 says "Starting point in lethal space" | The robot's own map shows an obstacle right where it stands. Nudge it with the teleop, close the teleop, and send the goal again. |

---

## 12. Glossary

- **SLAM:** Simultaneous Localization And Mapping. The robot builds a map
  while working out where it is on that map.
- **Odometry:** position estimated from wheel rotation. Good short-term, but
  drifts over time.
- **Occupancy grid:** a map made of small cells (5 cm here). Each cell is
  free, occupied or unknown.
- **Map merging:** combining several robots' maps into one.
- **Costmap:** Nav2's version of the map, with walls "inflated" so the robot
  keeps a safe distance. Each robot has a **local** costmap (3 m around it,
  from live scans) and a **global** costmap (20 m, from its SLAM map).
- **Frontier:** the border between explored free space and unexplored space.
  Exploration means "go to a frontier, repeat".
- **Lifecycle node:** a Nav2 node that must be configured, then activated,
  before it works. A lifecycle manager does this at startup.
- **Composition:** running several ROS nodes inside one process. It's faster
  to start and uses less CPU than one process per node.
- **Sim time:** the simulator's own clock, published on `/clock`. Every node
  in the sim must use it (`use_sim_time:=true`), otherwise timestamps don't
  match.
