"""
Merges /scout_1/map, /scout_2/map, /mapper_1/map into one /map_merged grid
using each robot's known spawn pose from fleet_config.py rather than feature
matching, since every robot's `map` frame origin coincides with its spawn
pose (verified: scout_1/map -> scout_1/base_link is identity at t=0, and the
fleet_pose . info.origin . cell-offset composition below was checked cell by
cell against exploration_test.sdf's known wall positions before this loop
was written).

Before publishing, narrow unknown gaps between free cells (the space between
far-apart lidar rays) are filled in; see fill_ray_gaps.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from swarm_bringup.fleet_config import FLEET

# name -> (x, y, yaw): the fixed transform from that robot's own `map` frame
# into the shared frame, taken directly from the spawn poses already in
# fleet_config.py rather than measured.
FLEET_POSES = {name: (x, y, yaw) for name, _cls, x, y, yaw in FLEET}

# Output grid: covers the whole exploration_test.sdf world (outer walls at
# +/-6m) with margin, at the same resolution slam_toolbox uses.
MERGED_RESOLUTION = 0.05
MERGED_WIDTH = 300
MERGED_HEIGHT = 300
MERGED_ORIGIN_X = -7.5
MERGED_ORIGIN_Y = -7.5

OCCUPIED_THRESHOLD = 65


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def transform_point(x, y, origin_x, origin_y, yaw):
    cos_t = math.cos(yaw)
    sin_t = math.sin(yaw)
    return (
        origin_x + x * cos_t - y * sin_t,
        origin_y + x * sin_t + y * cos_t,
    )


def cell_to_shared_frame(msg, row, col, robot_name):
    """Where cell (row, col) of this robot's occupancy grid lands in the
    shared frame: grid-local offset -> this robot's map frame (via
    info.origin) -> shared frame (via that robot's fleet_config pose)."""
    res = msg.info.resolution
    local_x = col * res
    local_y = row * res

    grid_origin = msg.info.origin
    grid_yaw = yaw_from_quaternion(grid_origin.orientation)
    map_x, map_y = transform_point(
        local_x, local_y, grid_origin.position.x, grid_origin.position.y, grid_yaw
    )

    robot_x, robot_y, robot_yaw = FLEET_POSES[robot_name]
    return transform_point(map_x, map_y, robot_x, robot_y, robot_yaw)


def merge_cell(existing, new):
    """Combine one output cell's current value with a newly-observed one.
    Unknown loses to anything known. Occupied wins over free -- missing a
    wall is worse than wrongly marking free space as blocked. Otherwise,
    keep whichever value was already there (first robot to see it wins)."""
    if new == -1:
        return existing
    if existing == -1:
        return new
    if new >= OCCUPIED_THRESHOLD or existing >= OCCUPIED_THRESHOLD:
        return max(existing, new)
    return existing


def fill_ray_gaps(grid):
    """Return the merged grid with narrow unknown gaps between free cells
    marked free: a 3x3 morphological closing of the free cells, applied to
    unknown cells only. It fills gaps up to 2 cells (10 cm) wide.

    Why: far from a lidar its rays are more than one cell apart (1 deg rays
    are 5 cm apart at 2.9 m), so the cells between them stay unknown.
    explore_lite counts unknown cells next to free space as frontier, so these
    specks become frontiers that robots chase. Occupied cells are never
    changed, and it cannot reach past a wall into unexplored space: a cell is
    filled only if everything around it is within one cell of free space."""
    g = np.array(grid, dtype=np.int8).reshape(MERGED_HEIGHT, MERGED_WIDTH)
    closed = _erode3(_dilate3(g == 0))
    g[closed & (g == -1)] = 0
    return g.ravel().tolist()


def _dilate3(mask):
    """True where any cell of the 3x3 neighbourhood is True."""
    h, w = mask.shape
    padded = np.pad(mask, 1, constant_values=False)
    out = np.zeros_like(mask)
    for dy in range(3):
        for dx in range(3):
            out |= padded[dy:dy + h, dx:dx + w]
    return out


def _erode3(mask):
    """True where every cell of the 3x3 neighbourhood is True (cells past
    the grid edge count as False)."""
    h, w = mask.shape
    padded = np.pad(mask, 1, constant_values=False)
    out = np.ones_like(mask)
    for dy in range(3):
        for dx in range(3):
            out &= padded[dy:dy + h, dx:dx + w]
    return out


class MapMergeNode(Node):
    def __init__(self):
        super().__init__('map_merge_node')
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        # Latest full map per robot. slam_toolbox re-rasterises its whole map
        # from the optimised pose graph on every publish, so only the newest
        # message from each robot is valid -- older ones are superseded.
        self._latest_data = {name: None for name in FLEET_POSES}
        self.map_subscribers = {
            name: self.create_subscription(
                OccupancyGrid,
                f'/{name}/map',
                lambda msg, name=name: self.map_callback(msg, name),
                map_qos,
            )
            for name in FLEET_POSES
        }
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map_merged', map_qos)
        self.create_timer(1.0, self._rebuild_merged_map)

    def map_callback(self, msg, robot_name):
        self._latest_data[robot_name] = msg

    def _rebuild_merged_map(self):
        """Merge every robot's latest map into a fresh grid, so anything a
        robot's SLAM no longer believes drops out of the merged map too."""
        maps = {name: msg for name, msg in self._latest_data.items() if msg is not None}
        if not maps:
            return
        grid = [-1] * (MERGED_WIDTH * MERGED_HEIGHT)
        for robot_name, msg in maps.items():
            self._merge_into_output(grid, msg, robot_name)
        self._publish_merged(fill_ray_gaps(grid))

    def _merge_into_output(self, grid, msg, robot_name):
        width = msg.info.width
        for i, value in enumerate(msg.data):
            if value == -1:
                continue
            row, col = divmod(i, width)
            shared_x, shared_y = cell_to_shared_frame(msg, row, col, robot_name)
            # floor, not int(): int() truncates toward zero, which would fold
            # points just past the -x/-y edge into column/row 0.
            out_col = math.floor((shared_x - MERGED_ORIGIN_X) / MERGED_RESOLUTION)
            out_row = math.floor((shared_y - MERGED_ORIGIN_Y) / MERGED_RESOLUTION)
            if not (0 <= out_row < MERGED_HEIGHT and 0 <= out_col < MERGED_WIDTH):
                continue
            out_index = out_row * MERGED_WIDTH + out_col
            grid[out_index] = merge_cell(grid[out_index], value)

    def _publish_merged(self, grid):
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        # Shared frame: world -> <name>/map static transforms, built from the
        # same fleet_config poses used above, connect it to every robot's TF tree.
        out.header.frame_id = 'world'
        out.info.resolution = MERGED_RESOLUTION
        out.info.width = MERGED_WIDTH
        out.info.height = MERGED_HEIGHT
        out.info.origin.position.x = MERGED_ORIGIN_X
        out.info.origin.position.y = MERGED_ORIGIN_Y
        out.info.origin.orientation.w = 1.0
        out.data = grid
        self.map_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
