"""
Frontier coordinator: one node that explores with all robots as a team.

It replaces explore_lite (explore_swarm.launch.py). There, every robot ran
its own explorer on the same merged map, so they all chose the same frontier
and drove to the same point (PROJECT_GUIDE.md, section 9). This node plans
for every robot at once. Once a second it:

  1. reads /map_merged and finds the frontier: known-free cells that touch
     unknown ones;
  2. cuts the frontier into pieces about piece_size long, each with one goal
     point that has room around it for the robot;
  3. measures every robot's path length to every piece, through known free
     space, so walls count;
  4. hands out pieces cheapest first, one per robot, and claims the area
     around each one: no other robot is sent within claim_radius (by path)
     of a claimed piece;
  5. sends each goal to that robot's own Nav2 (/<robot>/navigate_to_pose).

A robot gets a new piece when it reaches its goal, when Nav2 gives up on
it, when it stops making progress, or when its frontier has been mapped
already (then it doesn't need to get there). Goals are never closer than
min_goal_distance, so Nav2 can't report one reached before the robot moves.
When no piece is left, every robot is sent back to its spawn pose and the
node exits. A new goal simply replaces the old one in Nav2; nothing is
cancelled first, so there is no race between a cancel and a new goal.

Time comes from the stamps on /map_merged (sim time). That avoids
subscribing to the 1 kHz /clock, which costs a Python node ~45% of a core.
"""
import math

import numpy as np
from scipy import ndimage
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

from swarm_bringup.fleet_config import FLEET

OCCUPIED_THRESHOLD = 65   # same as map_merge_node
ROBOT_COLORS = {'scout_1': (1.0, 0.55, 0.0), 'scout_2': (0.3, 0.75, 1.0), 'mapper_1': (0.3, 0.9, 0.3)}


class Piece:
    """One stretch of frontier and the point a robot is sent to for it."""
    __slots__ = ('x', 'y', 'row', 'col', 'size', 'points')

    def __init__(self, x, y, row, col, size, points):
        self.x, self.y, self.row, self.col = x, y, row, col
        self.size = size        # frontier cells in this piece
        self.points = points    # (N, 2) world coordinates of those cells, for display


def find_pieces(grid, res, origin_x, origin_y, piece_size, min_segment_cells,
                min_piece_cells, clearance_m, goal_clearance_m):
    """Frontier pieces of an occupancy grid (int8, rows = y), plus the mask
    of cells a robot can drive through (free, clearance_m from obstacles)."""
    free = grid == 0
    unknown = grid == -1
    occupied = grid >= OCCUPIED_THRESHOLD
    if occupied.any():
        clearance = ndimage.distance_transform_edt(~occupied) * res   # m to nearest obstacle
    else:
        clearance = np.full(grid.shape, np.inf)
    touches_unknown = np.zeros_like(unknown)
    touches_unknown[1:, :] |= unknown[:-1, :]
    touches_unknown[:-1, :] |= unknown[1:, :]
    touches_unknown[:, 1:] |= unknown[:, :-1]
    touches_unknown[:, :-1] |= unknown[:, 1:]
    frontier = free & touches_unknown
    traversable = free & (clearance >= clearance_m)

    # Connected stretches of frontier; tiny ones are specks, not frontiers.
    labels, _ = ndimage.label(frontier, structure=np.ones((3, 3), bool))
    rows, cols = np.nonzero(frontier)
    seg = labels[rows, cols]
    keep = np.bincount(labels.ravel())[seg] >= min_segment_cells
    rows, cols, seg = rows[keep], cols[keep], seg[keep]
    if not len(rows):
        return [], traversable
    xs = origin_x + (cols + 0.5) * res
    ys = origin_y + (rows + 0.5) * res
    # Cut every stretch at the edges of a piece_size grid of square tiles.
    tx = np.floor(xs / piece_size).astype(np.int64) + 10000
    ty = np.floor(ys / piece_size).astype(np.int64) + 10000
    _, piece_of = np.unique((seg.astype(np.int64) * 20000 + tx) * 20000 + ty, return_inverse=True)
    piece_of = piece_of.ravel()
    goal_ok = clearance[rows, cols] >= goal_clearance_m
    pieces = []
    for k in range(piece_of.max() + 1):
        member = piece_of == k
        size = int(member.sum())
        if size < min_piece_cells:
            continue
        candidates = np.nonzero(member & goal_ok)[0]
        if not len(candidates):
            continue            # no spot on this piece with room for a robot
        cx, cy = xs[member].mean(), ys[member].mean()
        best = candidates[np.argmin((xs[candidates] - cx) ** 2 + (ys[candidates] - cy) ** 2)]
        pieces.append(Piece(float(xs[best]), float(ys[best]), int(rows[best]), int(cols[best]),
                            size, np.column_stack((xs[member], ys[member]))))
    return pieces, traversable


class PathGraph:
    """The traversable cells as an 8-connected graph, for path lengths."""

    def __init__(self, traversable, res):
        self.shape = traversable.shape
        h, w = self.shape
        self.rr, self.cc = np.nonzero(traversable)
        self.node = np.full((h, w), -1, np.int64)
        self.node[self.rr, self.cc] = np.arange(len(self.rr))
        src, dst, length = [], [], []
        for dy, dx, step in ((0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2.0)), (1, -1, math.sqrt(2.0))):
            c0, c1 = max(0, -dx), w - max(0, dx)
            a = self.node[0:h - dy, c0:c1]
            b = self.node[dy:h, c0 + dx:c1 + dx]
            both = (a >= 0) & (b >= 0)
            src.append(a[both])
            dst.append(b[both])
            length.append(np.full(int(both.sum()), step * res))
        n = len(self.rr)
        self.graph = csr_matrix((np.concatenate(length), (np.concatenate(src), np.concatenate(dst))),
                                shape=(n, n))

    def lengths_from(self, cells):
        """Path length in metres from each traversable (row, col) in cells to
        every cell; inf where unreachable. Shape (len(cells), H, W)."""
        out = np.full((len(cells),) + self.shape, np.inf)
        if len(self.rr) and cells:
            dist = dijkstra(self.graph, directed=False, indices=[int(self.node[r, c]) for r, c in cells])
            out[:, self.rr, self.cc] = dist
        return out


def nearest_cell(mask, row, col, max_cells):
    """The True cell of mask closest to (row, col), within max_cells; or None."""
    r0, c0 = max(0, row - max_cells), max(0, col - max_cells)
    sub = mask[r0:row + max_cells + 1, c0:col + max_cells + 1]
    rr, cc = np.nonzero(sub)
    if not len(rr):
        return None
    i = int(np.argmin((rr + r0 - row) ** 2 + (cc + c0 - col) ** 2))
    return int(rr[i] + r0), int(cc[i] + c0)


def assign_pieces(pieces, poses, from_robot, claims, graph, res,
                  min_goal_distance, claim_radius, size_weight):
    """Greedy assignment: give the cheapest (robot, piece) pair first, claim
    the area around that piece, repeat. Cost is the path length minus
    size_weight times the piece's length, so a bigger piece is worth a
    slightly longer drive. from_robot and claims hold path-length grids.
    Returns {robot: (piece, path length)}."""
    assigned = {}
    claims = list(claims)
    candidates = list(from_robot)
    while candidates:
        best = None
        for name in candidates:
            rx, ry = poses[name]
            for pc in pieces:
                if math.hypot(pc.x - rx, pc.y - ry) < min_goal_distance:
                    continue
                if any(cl[pc.row, pc.col] < claim_radius for cl in claims):
                    continue
                d = from_robot[name][pc.row, pc.col]
                if not math.isfinite(d):
                    continue
                cost = d - size_weight * pc.size * res
                if best is None or cost < best[0]:
                    best = (cost, name, pc, d)
        if best is None:
            break
        _, name, pc, d = best
        assigned[name] = (pc, d)
        candidates.remove(name)
        claims.append(graph.lengths_from([(pc.row, pc.col)])[0])
    return assigned


class RobotState:
    def __init__(self, spawn):
        self.spawn = spawn          # (x, y, yaw) in world = origin of <robot>/map
        self.phase = 'explore'      # explore -> return -> home | failed
        self.goal = None            # (x, y) in world while a goal is in force
        self.token = 0              # bumped per goal; results of older goals are ignored
        self.best_dist = math.inf
        self.last_progress = 0.0
        self.goals_sent = 0
        self.return_tries = 0


class FrontierCoordinator(Node):
    def __init__(self):
        super().__init__('frontier_coordinator')
        p = {
            'robots': '',                 # comma-separated; empty = every robot in fleet_config
            'piece_size': 1.0,            # m; frontier is cut into tiles this big
            'min_segment_size': 0.5,      # m; shorter frontier stretches are ignored
            'min_piece_size': 0.3,        # m; smaller leftovers of a cut are ignored
            'clearance': 0.2,             # m; paths keep this far from obstacles (mapper half-width)
            'goal_clearance': 0.3,        # m; goals keep this far from obstacles
            'min_goal_distance': 0.6,     # m; never send a goal closer than this (goal tolerance is 0.25)
            'claim_radius': 2.0,          # m by path; no second robot is sent this close to a claimed piece
            'size_weight': 1.0,           # m of travel that 1 m of frontier is worth
            'still_valid_radius': 0.75,   # m; a goal stays while any frontier cell is this close to it
            'progress_timeout': 30.0,     # s (sim) without getting 0.25 m closer -> give up on that goal
            'skip_radius': 0.5,           # m around a failed goal that is skipped ...
            'skip_time': 60.0,            # s (sim) ... for this long
            'done_cycles': 3,             # cycles in a row with nothing to do before exploration is done
            'return_home': True,
        }
        for name, default in p.items():
            self.declare_parameter(name, default)
        self.p = {name: self.get_parameter(name).value for name in p}

        wanted = [r.strip() for r in self.p['robots'].split(',') if r.strip()]
        fleet = {name: (x, y, yaw) for name, _cls, x, y, yaw in FLEET}
        for name in wanted:
            if name not in fleet:
                raise RuntimeError(f"unknown robot '{name}'; fleet_config has {list(fleet)}")
        self.robots = {name: RobotState(fleet[name]) for name in (wanted or fleet)}

        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE)
        self.map = None
        self.map_wall_time = 0.0
        self.create_subscription(OccupancyGrid, '/map_merged', self._on_map, map_qos)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.nav = {name: ActionClient(self, NavigateToPose, f'/{name}/navigate_to_pose')
                    for name in self.robots}
        self.markers = self.create_publisher(MarkerArray, '/coordinator/markers', 1)

        self.now = 0.0              # sim seconds, from the latest map stamp
        self.skip = []              # (x, y, until) spots where a goal failed
        self.started_at = None      # sim time of the first goal
        self.done_count = 0
        self.finished = False
        self.last_report = -math.inf
        self.create_timer(1.0, self._cycle)
        self.get_logger().info(f"Coordinating {', '.join(self.robots)}; waiting for /map_merged and Nav2.")

    # ---------------------------------------------------------------- inputs

    def _on_map(self, msg):
        self.map = msg
        self.map_wall_time = self.get_clock().now().nanoseconds * 1e-9
        self.now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _robot_pose(self, name):
        try:
            t = self.tf_buffer.lookup_transform('world', f'{name}/base_link', rclpy.time.Time()).transform
        except tf2_ros.TransformException:
            return None
        return t.translation.x, t.translation.y

    # ------------------------------------------------------------------ cycle

    def _cycle(self):
        if self.finished:
            return
        if self.map is None:
            self.get_logger().info('Waiting for /map_merged ...', throttle_duration_sec=10.0)
            return
        if self.get_clock().now().nanoseconds * 1e-9 - self.map_wall_time > 5.0:
            self.get_logger().warn('/map_merged is more than 5 s old; is map_merge_node running?',
                                   throttle_duration_sec=10.0)
            return

        info = self.map.info
        res, ox, oy = info.resolution, info.origin.position.x, info.origin.position.y
        grid = np.asarray(self.map.data, dtype=np.int8).reshape(info.height, info.width)
        pieces, traversable = find_pieces(
            grid, res, ox, oy, self.p['piece_size'],
            max(1, round(self.p['min_segment_size'] / res)), max(1, round(self.p['min_piece_size'] / res)),
            self.p['clearance'], self.p['goal_clearance'])
        self.skip = [s for s in self.skip if s[2] > self.now]
        pieces = [pc for pc in pieces if not any(math.hypot(pc.x - sx, pc.y - sy) < self.p['skip_radius']
                                                   for sx, sy, _ in self.skip)]

        def to_cell(x, y):
            return int((y - oy) / res), int((x - ox) / res)

        poses = {name: self._robot_pose(name) for name in self.robots}

        if any(st.phase == 'return' for st in self.robots.values()) or \
                (self.started_at is not None and all(st.phase in ('home', 'failed') for st in self.robots.values())):
            for name, st in self.robots.items():
                if st.phase == 'return' and st.goal is not None and poses[name] is not None \
                        and self._stalled(st, poses[name]):
                    self.get_logger().warn(f"{name}: no progress on the way home for "
                                           f"{self.p['progress_timeout']:.0f} s.")
                    self._return_failed(name)
            self._check_finished()
            self._publish_markers(pieces, poses)
            return

        # Drop goals that are done in all but name: their frontier is mapped
        # already, or the robot has stopped getting closer.
        frontier_xy = np.concatenate([pc.points for pc in pieces]) if pieces else np.empty((0, 2))
        for name, st in self.robots.items():
            if st.goal is None or poses[name] is None:
                continue
            gx, gy = st.goal
            if not len(frontier_xy) or np.hypot(frontier_xy[:, 0] - gx, frontier_xy[:, 1] - gy).min() \
                    >= self.p['still_valid_radius']:
                self.get_logger().info(f'{name}: the frontier at its goal is mapped now; picking a new one.')
                st.goal = None
                continue
            if self._stalled(st, poses[name]):
                self.get_logger().warn(f'{name}: no progress towards ({gx:.2f}, {gy:.2f}) for '
                                       f"{self.p['progress_timeout']:.0f} s; skipping that spot.")
                self.skip.append((gx, gy, self.now + self.p['skip_time']))
                st.goal = None

        free = [name for name, st in self.robots.items()
                if st.goal is None and poses[name] is not None and self.nav[name].server_is_ready()]
        busy = [name for name, st in self.robots.items() if st.goal is not None]

        # Path lengths from every free robot (to choose pieces) and from every
        # claimed goal (to keep other robots away from it).
        starts, owners = [], []
        for name in free:
            r, c = to_cell(*poses[name])
            cell = nearest_cell(traversable, r, c, int(1.0 / res))   # the robot's own body may mark it occupied
            if cell is not None:
                starts.append(cell)
                owners.append(('robot', name))
        for name in busy:
            r, c = to_cell(*self.robots[name].goal)
            cell = nearest_cell(traversable, r, c, int(0.5 / res))
            if cell is not None:
                starts.append(cell)
                owners.append(('claim', name))
        graph = PathGraph(traversable, res) if pieces and free else None
        dist = graph.lengths_from(starts) if graph is not None else None
        from_robot = {who: dist[i] for i, (kind, who) in enumerate(owners) if kind == 'robot'} if dist is not None else {}
        claims = [dist[i] for i, (kind, _) in enumerate(owners) if kind == 'claim'] if dist is not None else []

        assigned = assign_pieces(pieces, {n: poses[n] for n in from_robot}, from_robot, claims, graph, res,
                                 self.p['min_goal_distance'], self.p['claim_radius'], self.p['size_weight'])

        for name, (pc, d) in assigned.items():
            rx, ry = poses[name]
            self._send(name, pc.x, pc.y, math.atan2(pc.y - ry, pc.x - rx), 'explore')
            self.get_logger().info(f'{name} -> frontier at ({pc.x:.2f}, {pc.y:.2f}), {d:.1f} m away by path')

        if free and not assigned and self.started_at is None:
            self.get_logger().info(f'No frontier a robot can reach yet ({len(pieces)} pieces).',
                                   throttle_duration_sec=10.0)

        # Done when every robot is free and none of them has anywhere to go.
        exploring = any(st.goal is not None for st in self.robots.values())
        if self.started_at is not None and free and not assigned and not exploring:
            self.done_count += 1
            if self.done_count >= self.p['done_cycles']:
                self._exploration_done(grid, res, len(pieces))
        else:
            self.done_count = 0

        if self.now - self.last_report >= 10.0 and self.started_at is not None:
            self.last_report = self.now
            goals = ', '.join(f'{n}->({st.goal[0]:.1f}, {st.goal[1]:.1f})' if st.goal else f'{n}: idle'
                              for n, st in self.robots.items())
            self.get_logger().info(f't={self.now - self.started_at:5.1f} s  known {self._known_m2(grid, res):6.1f} m2, '
                                   f'{len(pieces)} frontier pieces | {goals}')
        self._publish_markers(pieces, poses)

    def _stalled(self, st, pose):
        """True once the robot hasn't got 0.25 m closer to its goal for
        progress_timeout seconds."""
        d = math.hypot(st.goal[0] - pose[0], st.goal[1] - pose[1])
        if d < st.best_dist - 0.25:
            st.best_dist, st.last_progress = d, self.now
            return False
        return self.now - st.last_progress > self.p['progress_timeout']

    # ------------------------------------------------------------ goals

    def _send(self, name, x, y, yaw, phase):
        """Send a goal (world frame) to this robot's Nav2, expressed in the
        robot's own map frame. A new goal replaces the old one in Nav2."""
        st = self.robots[name]
        st.token += 1
        st.phase, st.goal = phase, (x, y)
        st.best_dist, st.last_progress = math.inf, self.now
        st.goals_sent += phase == 'explore'
        if self.started_at is None:
            self.started_at = self.now
        sx, sy, syaw = st.spawn
        dx, dy = x - sx, y - sy
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = f'{name}/map'   # stamp stays 0: use the latest transform
        goal.pose.pose.position.x = math.cos(syaw) * dx + math.sin(syaw) * dy
        goal.pose.pose.position.y = -math.sin(syaw) * dx + math.cos(syaw) * dy
        goal.pose.pose.orientation.z = math.sin((yaw - syaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos((yaw - syaw) / 2.0)
        token = st.token
        future = self.nav[name].send_goal_async(goal)
        future.add_done_callback(lambda f, name=name, token=token: self._on_goal_response(name, token, f))

    def _on_goal_response(self, name, token, future):
        st = self.robots[name]
        handle = future.result()
        if token != st.token:
            return                  # superseded before Nav2 answered; the newer goal replaces it
        if not handle.accepted:
            self.get_logger().warn(f'{name}: Nav2 rejected the goal; is its bt_navigator active?')
            st.goal = None
            if st.phase == 'return':
                self._return_failed(name)
            return
        handle.get_result_async().add_done_callback(
            lambda f, name=name, token=token: self._on_result(name, token, f))

    def _on_result(self, name, token, future):
        st = self.robots[name]
        if token != st.token:
            return                  # an older goal that a newer one replaced
        status = future.result().status
        goal, st.goal = st.goal, None
        if st.phase == 'return':
            if status == GoalStatus.STATUS_SUCCEEDED:
                st.phase = 'home'
                self.get_logger().info(f'{name} is home.')
            else:
                self._return_failed(name)
            return
        if status == GoalStatus.STATUS_ABORTED and goal is not None:
            self.get_logger().warn(f'{name}: Nav2 gave up on ({goal[0]:.2f}, {goal[1]:.2f}); '
                                   f"skipping that spot for {self.p['skip_time']:.0f} s.")
            self.skip.append((goal[0], goal[1], self.now + self.p['skip_time']))

    # ------------------------------------------------------------ the end

    def _exploration_done(self, grid, res, leftover):
        took = self.now - self.started_at
        sent = ', '.join(f'{n} {st.goals_sent}' for n, st in self.robots.items())
        self.get_logger().info(f'Exploration done after {took:.1f} s: known {self._known_m2(grid, res):.1f} m2; '
                               f'goals sent: {sent}; {leftover} frontier pieces left that no robot can use.')
        if not self.p['return_home']:
            self.finished = True
            return
        for name, st in self.robots.items():
            if self.nav[name].server_is_ready():
                self._send(name, st.spawn[0], st.spawn[1], st.spawn[2], 'return')
            else:
                st.phase = 'failed'
        self.get_logger().info('Sending every robot back to its spawn pose.')

    def _return_failed(self, name):
        st = self.robots[name]
        if st.return_tries < 2:
            st.return_tries += 1
            self.get_logger().warn(f'{name}: going home failed; trying again ({st.return_tries}/2).')
            self._send(name, st.spawn[0], st.spawn[1], st.spawn[2], 'return')
        else:
            st.phase = 'failed'
            self.get_logger().error(f'{name}: could not get home.')

    def _check_finished(self):
        if all(st.phase in ('home', 'failed') for st in self.robots.values()):
            home = [n for n, st in self.robots.items() if st.phase == 'home']
            self.get_logger().info(f"Finished. Home: {', '.join(home) or 'none'}.")
            self.finished = True

    @staticmethod
    def _known_m2(grid, res):
        return float((grid != -1).sum()) * res * res

    # ------------------------------------------------------------ display

    def _publish_markers(self, pieces, poses):
        arr = MarkerArray()
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        def marker(ns, mid, mtype, scale, rgb, alpha=1.0):
            m = Marker()
            m.header.frame_id = 'world'      # stamp 0: drawn with the latest transform
            m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = scale
            m.color = ColorRGBA(r=rgb[0], g=rgb[1], b=rgb[2], a=alpha)
            return m

        cells = marker('frontier', 0, Marker.POINTS, 0.05, (0.2, 0.4, 1.0))
        for pc in pieces:
            cells.points.extend(Point(x=float(x), y=float(y), z=0.02) for x, y in pc.points)
        arr.markers.append(cells)
        spots = marker('pieces', 0, Marker.SPHERE_LIST, 0.12, (0.2, 1.0, 0.2), 0.6)
        spots.points = [Point(x=pc.x, y=pc.y, z=0.05) for pc in pieces]
        arr.markers.append(spots)
        for i, (name, st) in enumerate(self.robots.items()):
            if st.goal is None:
                continue
            rgb = ROBOT_COLORS.get(name, (1.0, 1.0, 1.0))
            goal = marker('goals', i, Marker.SPHERE, 0.3, rgb, 0.9)
            goal.pose.position = Point(x=st.goal[0], y=st.goal[1], z=0.15)
            arr.markers.append(goal)
            if poses.get(name) is not None:
                line = marker('lines', i, Marker.LINE_LIST, 0.03, rgb, 0.9)
                line.points = [Point(x=poses[name][0], y=poses[name][1], z=0.1),
                               Point(x=st.goal[0], y=st.goal[1], z=0.1)]
                arr.markers.append(line)
            label = marker('labels', i, Marker.TEXT_VIEW_FACING, 0.25, rgb)
            label.pose.position = Point(x=st.goal[0], y=st.goal[1], z=0.5)
            label.text = name if st.phase == 'explore' else f'{name} (home)'
            arr.markers.append(label)
        self.markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierCoordinator()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    rclpy.try_shutdown()
