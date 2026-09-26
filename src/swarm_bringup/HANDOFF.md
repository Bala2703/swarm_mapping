 # Project handoff notes

Context for picking this project back up in Claude Code. This isn't
user-facing documentation (see README.md for that) — it's a summary of
decisions made and why, so they don't get silently re-litigated or
reversed by accident.

## Goal

Heterogeneous ground-rover swarm sim for multi-robot exploration/
mapping, with an explicit sim-to-real target: 2 real rovers running
ROS 2 Humble. That real-hardware constraint is *why* the stack below
was chosen over newer alternatives — matching the real robots' distro
mattered more than using the newest Gazebo.

## Stack, and why

- **ROS 2 Humble + Gazebo Fortress.** Humble is EOL May 2027; Fortress
  (Humble's official binary-supported Gazebo pairing) reached its own
  EOL in September 2026. Neither is going anywhere in the short term,
  but don't expect new fixes/features from Fortress going forward —
  it's frozen. The pairing was chosen specifically to match the real
  rovers' ROS 2 distro for sim-to-real transfer, overriding what would
  otherwise be the better-supported default (Jazzy + Harmonic).
- Package naming follows Fortress/Ignition-era conventions:
  `libignition-gazebo-*.so` plugin filenames, `ignition.msgs.*` in
  bridge type strings, `ign gazebo` as the launch command. On
  Harmonic/Jetty these become `libgz-sim-*`, `gz.msgs.*`, `gz sim` —
  nothing else about the approach changes if this project ever moves
  off Fortress.

## Known environment gotcha

If Zenoh transport (`GZ_TRANSPORT_IMPLEMENTATION=zenoh` or similar) is
set anywhere in the shell environment — e.g. left over from a real
robot's own transport setup — it silently breaks Gazebo/ROS discovery
in the sim (robots fail to spawn, or spawn but never bridge to ROS,
with no obvious error pointing at the cause). Already hit this once.
Worth being the first thing checked if nodes/entities mysteriously
don't show up again: `env | grep -i zenoh` / `env | grep -i
GZ_TRANSPORT` before debugging anything else.

## Current package: swarm_bringup

- `fleet_config.py` — single source of truth for the robot roster
  (name, class, spawn pose). Both launch files import from here;
  never duplicate the list.
- `rover.urdf.xacro` — one macro, two classes (`scout`/`mapper`)
  differentiated by chassis size, lidar range/resolution, speed caps.
  Placeholder dimensions — swap in the real rovers' actual
  measurements before trusting any sim-to-real comparison.
- Drivetrain is the native Gazebo `DiffDrive` system plugin, not
  `ign_ros2_control`. Deliberate simplification to get something
  driving end-to-end fast. **Still pending:** swap to
  `ign_ros2_control` + `diff_drive_controller` so the sim rovers use
  the same controller_manager interface the real rovers do — this
  matters more here than in a pure-sim project, given the sim-to-real
  goal.
- `spawn_swarm.launch.py` — world + robots + per-robot topic bridges
  (cmd_vel, odometry, tf, scan, imu, joint_states). Bridge topic
  strings were deliberately set explicitly on both the xacro sensor
  side and the launch-file bridge side (rather than relying on
  Gazebo's default auto-scoped topic names) so they're guaranteed to
  match — verify with `ign topic -l` after first spawn if anything's
  silent.
- `slam_swarm.launch.py` — includes spawn_swarm + one
  `async_slam_toolbox_node` per robot, each with its own namespaced
  odom/base_link/map frames. Frame_id strings needed explicit
  per-robot values in the launch file loop — ROS namespacing doesn't
  auto-apply to TF frame_id strings the way it does to topic names.
  **Not yet done:** merging the per-robot maps into one shared map.

## Map-merge: deliberate decision, don't default back to the obvious package

The community package for this (`m-explore-ros2`'s
`multirobot_map_merge`) was evaluated and **intentionally not used**:
it requires building from source (no apt binary), and its
slam_toolbox support (vs. the older gmapping) currently lives on an
experimental branch rather than main.

Since every robot's spawn pose is already known (`fleet_config.py`),
our merge problem is strictly easier than what that package solves —
it does feature-matching to *estimate* unknown relative robot
positions, which is unnecessary overhead here. The plan is a small
custom node that overlays each robot's occupancy grid at its known
fixed pose offset directly. Re-evaluate this decision only if the
robots' relative poses stop being known/trustworthy (e.g. moved to
unknown-start-position experiments).

## Immediate next steps, in order

1. Custom known-pose map-merge node (see above).
2. Frontier detection + a claim/assignment mechanism so scouts don't
   converge on the same frontier once the merged map exists.
3. `ign_ros2_control` swap for the drivetrain.
4. Exploration/mapping metrics + RViz2 multi-robot visualization setup.
