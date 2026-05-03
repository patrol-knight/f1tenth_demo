#!/usr/bin/env python3
import math
import heapq
from typing import List, Tuple, Optional
from collections import deque

import rclpy
from rclpy.node import Node

from nav_msgs.msg import OccupancyGrid, Path, Odometry
from geometry_msgs.msg import PoseArray, PoseStamped, Quaternion

from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


Grid = List[List[int]]


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    return q


def dist(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


class Planner(Node):
    def __init__(self):
        super().__init__("planner_node")

        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("robot_pose_topic", "/pf/pose/odom")
        self.declare_parameter("waypoints_topic", "/interface/waypoints")
        self.declare_parameter("path_topic", "/planning/path")

        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("inflation_radius_m", 0.45)

        self.declare_parameter("allow_diagonal", True)
        self.declare_parameter("astar_timeout_ms", 300)

        self.declare_parameter("resample_ds", 0.10)
        self.declare_parameter("min_path_points", 2)

        self.declare_parameter("smoothing_enable", True)
        self.declare_parameter("smoothing_alpha", 0.08)
        self.declare_parameter("smoothing_beta", 0.3)
        self.declare_parameter("smoothing_iterations", 60)

        self.map_topic = self.get_parameter("map_topic").value
        self.robot_pose_topic = self.get_parameter("robot_pose_topic").value
        self.waypoints_topic = self.get_parameter("waypoints_topic").value
        self.path_topic = self.get_parameter("path_topic").value

        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.stream_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid, self.map_topic, self._on_map, self.map_qos
        )

        self.robot_sub = self.create_subscription(
            Odometry, self.robot_pose_topic, self._on_robot_odom, self.stream_qos
        )

        self.wp_sub = self.create_subscription(
            PoseArray, self.waypoints_topic, self._on_waypoints, self.stream_qos
        )

        self.path_pub = self.create_publisher(Path, self.path_topic, 10)

        self.robot_odom: Optional[Odometry] = None
        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[Grid] = None
        self.inflated_grid: Optional[Grid] = None

        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = "map"

        self.get_logger().info("A* Planner with smoothing ready.")

    def _on_robot_odom(self, msg: Odometry):
        self.robot_odom = msg

    def _on_map(self, msg: OccupancyGrid):
        self.map_msg = msg
        self.frame_id = msg.header.frame_id or "map"
        self.width = msg.info.width
        self.height = msg.info.height
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y

        self.grid = self._occupancy_to_grid(msg)
        self.inflated_grid = self._inflate_grid(self.grid)

        self.get_logger().info(f"Map received: {self.width}x{self.height}")

    def _on_waypoints(self, msg: PoseArray):
        if self.map_msg is None or self.inflated_grid is None:
            self.get_logger().warn("No map yet.")
            return

        if self.robot_odom is None:
            self.get_logger().warn("No odom yet.")
            return

        if len(msg.poses) < 1:
            self.get_logger().warn("Received empty waypoints.")
            empty_path = Path()
            empty_path.header.frame_id = "map"
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.poses = []
            self.path_pub.publish(empty_path)
            return

        world_points = []

        world_points.append((
            self.robot_odom.pose.pose.position.x,
            self.robot_odom.pose.pose.position.y
        ))

        for pose in msg.poses:
            world_points.append((pose.position.x, pose.position.y))

        way_cells = []
        for (wx, wy) in world_points:
            cell = self.world_to_cell(wx, wy)
            if cell is None:
                self.get_logger().warn("Waypoint outside map.")
                return
            way_cells.append(cell)

        full_cells = []

        for i in range(len(way_cells) - 1):
            start   = self._nearest_free(way_cells[i])
            goal    = self._nearest_free(way_cells[i + 1])

            if start is None or goal is None:
                self.get_logger().warn("Start/Goal blocked.")
                return

            seg = self._astar(start, goal)
            if seg is None:
                self.get_logger().warn("A* failed.")
                return

            if i > 0:
                seg = seg[1:]

            full_cells.extend(seg)

        poly = [self.cell_to_world(cx, cy) for (cx, cy) in full_cells]

        poly = self._resample(poly, self.get_parameter("resample_ds").value)

        if self.get_parameter("smoothing_enable").value:
            poly = self._smooth_path(poly)

        if len(poly) < self.get_parameter("min_path_points").value:
            self.get_logger().warn("Path too short.")
            return

        path_msg = self._polyline_to_path(poly)
        self.path_pub.publish(path_msg)

        self.get_logger().info(f"Published path with {len(path_msg.poses)} poses.")

    def _astar(self, start, goal):
        allow_diag = self.get_parameter("allow_diagonal").value
        timeout_ms = self.get_parameter("astar_timeout_ms").value

        def heuristic(a, b):
            return math.hypot(a[0] - b[0], a[1] - b[1])

        open_set = []
        heapq.heappush(open_set, (0, start))

        came_from = {start: None}
        g_score = {start: 0}

        t0 = self.get_clock().now().nanoseconds

        neighbors = [(-1,0),(1,0),(0,-1),(0,1)]
        if allow_diag:
            neighbors += [(-1,-1),(-1,1),(1,-1),(1,1)]

        while open_set:
            if (self.get_clock().now().nanoseconds - t0)/1e6 > timeout_ms:
                return None

            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current:
                    path.append(current)
                    current = came_from[current]
                return path[::-1]

            for dx, dy in neighbors:
                nx, ny = current[0] + dx, current[1] + dy

                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue
                if self.inflated_grid[ny][nx] == 1:
                    continue

                tentative_g = g_score[current] + math.hypot(dx, dy)
                neighbor = (nx, ny)

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return None

    def _smooth_path(self, pts):
        if len(pts) < 3:
            return pts

        alpha       = self.get_parameter("smoothing_alpha").value
        beta        = self.get_parameter("smoothing_beta").value
        iterations  = self.get_parameter("smoothing_iterations").value

        original = pts[:]
        smooth = [list(p) for p in pts]

        for _ in range(iterations):
            for i in range(1, len(pts) - 1):
                for dim in range(2):
                    smooth[i][dim] += alpha * (original[i][dim] - smooth[i][dim])
                    smooth[i][dim] += beta * (
                        smooth[i - 1][dim] +
                        smooth[i + 1][dim] -
                        2.0 * smooth[i][dim]
                    )

        return [(p[0], p[1]) for p in smooth]

    def _occupancy_to_grid(self, msg):
        thr = self.get_parameter("occupied_threshold").value
        unknown_occ = self.get_parameter("unknown_is_occupied").value

        grid = [[0]*self.width for _ in range(self.height)]

        for y in range(self.height):
            for x in range(self.width):
                v = msg.data[y*self.width + x]
                if v < 0:
                    grid[y][x] = 1 if unknown_occ else 0
                else:
                    grid[y][x] = 1 if v >= thr else 0

        return grid

    def _inflate_grid(self, grid):
        r_m = self.get_parameter("inflation_radius_m").value
        r_cells = int(math.ceil(r_m / self.resolution))

        inflated = [row[:] for row in grid]

        for y in range(self.height):
            for x in range(self.width):
                if grid[y][x] == 1:
                    for dy in range(-r_cells, r_cells+1):
                        for dx in range(-r_cells, r_cells+1):
                            ny = y + dy
                            nx = x + dx
                            if 0 <= nx < self.width and 0 <= ny < self.height:
                                inflated[ny][nx] = 1

        return inflated

    def world_to_cell(self, wx, wy):
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        if 0 <= mx < self.width and 0 <= my < self.height:
            return (mx, my)
        return None

    def cell_to_world(self, cx, cy):
        wx = self.origin_x + (cx + 0.5) * self.resolution
        wy = self.origin_y + (cy + 0.5) * self.resolution
        return (wx, wy)

    def _nearest_free(self, cell):
        x, y = cell

        if not (0 <= x < self.width and 0 <= y < self.height):
            return None

        if self.inflated_grid[y][x] == 0:
            return cell

        visited = set()
        q = deque()
        q.append((x, y))
        visited.add((x, y))

        neighbors = [(-1,0),(1,0),(0,-1),(0,1)]

        while q:
            cx, cy = q.popleft()

            for dx, dy in neighbors:
                nx, ny = cx + dx, cy + dy

                if not (0 <= nx < self.width and 0 <= ny < self.height):
                    continue

                if (nx, ny) in visited:
                    continue

                if self.inflated_grid[ny][nx] == 0:
                    return (nx, ny)

                visited.add((nx, ny))
                q.append((nx, ny))

        return None

    def _resample(self, pts, ds):
        if len(pts) < 2:
            return pts

        res = [pts[0]]
        acc = 0.0
        cur = pts[0]

        for i in range(1, len(pts)):
            nxt = pts[i]
            seg_len = dist(cur, nxt)

            if seg_len < 1e-6:
                continue

            direction = ((nxt[0] - cur[0]) / seg_len,
                         (nxt[1] - cur[1]) / seg_len)

            while acc + seg_len >= ds:
                remain = ds - acc
                newp = (cur[0] + direction[0] * remain,
                        cur[1] + direction[1] * remain)
                res.append(newp)
                cur = newp
                seg_len = dist(cur, nxt)
                acc = 0.0

            acc += seg_len
            cur = nxt

        if dist(res[-1], pts[-1]) > ds * 0.5:
            res.append(pts[-1])

        return res

    def _polyline_to_path(self, pts):
        path = Path()
        path.header.frame_id = self.frame_id
        path.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(pts)):
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = pts[i][0]
            ps.pose.position.y = pts[i][1]

            if i < len(pts)-1:
                dx = pts[i+1][0] - pts[i][0]
                dy = pts[i+1][1] - pts[i][1]
            else:
                dx = pts[i][0] - pts[i-1][0]
                dy = pts[i][1] - pts[i-1][1]

            yaw = math.atan2(dy, dx)
            ps.pose.orientation = yaw_to_quat(yaw)

            path.poses.append(ps)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = Planner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally: 
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
