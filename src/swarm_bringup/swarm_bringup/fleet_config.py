"""
Single source of truth for the swarm roster. Both spawn_swarm.launch.py
and slam_swarm.launch.py import FLEET/WORLD_NAME from here so they can
never drift out of sync with each other.
"""

# name, robot_class ("scout" | "mapper"), spawn x, y, yaw (radians)
FLEET = [
    ("scout_1",  "scout",  -3.0,  3.0, 0.0),
    ("scout_2",  "scout",  -3.0, -3.0, 0.0),
    ("mapper_1", "mapper",  0.0,  0.0, 0.0),
]

WORLD_NAME = "exploration_test"  # must match the <world name="..."> in the .sdf
