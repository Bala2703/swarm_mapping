"""
Merges /scout_1/map, /scout_2/map, /mapper_1/map into one /map_merged grid
using each robot's known spawn pose from fleet_config.py rather than feature
matching, since every robot's `map` frame origin coincides with its spawn
pose (verified: scout_1/map -> scout_1/base_link is identity at t=0, and the
fleet_pose . info.origin . cell-offset composition below was checked cell by
cell against exploration_test.sdf's known wall positions before this loop
was written).
"""
import math

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


class MapMergeNode(Node):
    def __init__(self):
        super().__init__('map_merge_node')
        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1,
        )
        self._merged_data = [-1] * (MERGED_WIDTH * MERGED_HEIGHT)
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

    def map_callback(self, msg, robot_name):
        self._merge_into_output(msg, robot_name)
        self._publish_merged()

    def _merge_into_output(self, msg, robot_name):
        width = msg.info.width
        for i, value in enumerate(msg.data):
            if value == -1:
                continue
            row, col = divmod(i, width)
            shared_x, shared_y = cell_to_shared_frame(msg, row, col, robot_name)
            out_col = int((shared_x - MERGED_ORIGIN_X) / MERGED_RESOLUTION)
            out_row = int((shared_y - MERGED_ORIGIN_Y) / MERGED_RESOLUTION)
            if not (0 <= out_row < MERGED_HEIGHT and 0 <= out_col < MERGED_WIDTH):
                continue
            out_index = out_row * MERGED_WIDTH + out_col
            self._merged_data[out_index] = merge_cell(self._merged_data[out_index], value)

    def _publish_merged(self):
        out = OccupancyGrid()
        out.header.stamp = self.get_clock().now().to_msg()
        # mapper_1 spawns at (0, 0, yaw=0) in fleet_config.py, so its own
        # `map` frame IS the shared frame this node computes into -- no new
        # frame or static transform needed for the merged output.
        out.header.frame_id = 'mapper_1/map'
        out.info.resolution = MERGED_RESOLUTION
        out.info.width = MERGED_WIDTH
        out.info.height = MERGED_HEIGHT
        out.info.origin.position.x = MERGED_ORIGIN_X
        out.info.origin.position.y = MERGED_ORIGIN_Y
        out.info.origin.orientation.w = 1.0
        out.data = list(self._merged_data)
        self.map_publisher.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = MapMergeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
