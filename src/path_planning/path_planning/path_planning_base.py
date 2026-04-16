#!/usr/bin/env python3
"""
Base class for ROS2 path planning node.
  - Subscribe: /pf/cost_map (nav_msgs/OccupancyGrid), /pf/pose/odom (nav_msgs/Odometry),
               /interface/waypoints (geometry_msgs/PoseArray)
  - Publish:   /planning/path (nav_msgs/Path)
  - Common pipeline:
      OccupancyGrid -> numpy grid
      current pose + waypoints -> grid cells
      compute distance matrix (Dijkstra early-stop)
      call algorithm hook to produce TOUR (indices)
      stitch tour legs with A*
      convert to nav_msgs/Path
  - Throttle replanning by min_replan_period_sec
"""

import math
import time
import heapq
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict, Set
from scipy.interpolate import CubicSpline
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
)

from nav_msgs.msg import OccupancyGrid, Odometry, Path
from geometry_msgs.msg import PoseArray, PoseStamped


Cell = Tuple[int, int]  # (row, col)
SQRT2 = math.sqrt(2.0)


# ============================================================
# 8-connected grid + heuristic
# ============================================================
def neighbors8(grid: np.ndarray, r: int, c: int, allow_corner_cutting: bool = True) -> List[Tuple[Cell, float]]:
    out: List[Tuple[Cell, float]] = []
    H, W = grid.shape
    for dr in (-1, 0, 1):
        for dc in (-1, 0, 1):
            if dr == 0 and dc == 0:
                continue
            rr, cc = r + dr, c + dc
            if not (0 <= rr < H and 0 <= cc < W):
                continue
            if grid[rr, cc] != 0:
                continue

            is_diag = (dr != 0 and dc != 0)
            if is_diag and not allow_corner_cutting:
                # Disallow diagonal if BOTH adjacent cardinal cells are blocked
                if grid[r + dr, c] != 0 and grid[r, c + dc] != 0:
                    continue

            cost = SQRT2 if is_diag else 1.0
            out.append(((rr, cc), cost))
    return out


def octile_distance(a: Cell, b: Cell) -> float:
    dx = abs(a[1] - b[1])
    dy = abs(a[0] - b[0])
    dmin = min(dx, dy)
    dmax = max(dx, dy)
    return (dmax - dmin) * 1.0 + dmin * SQRT2


# A* (8-connected)
@dataclass
class AStarResult:
    path: Optional[List[Cell]]
    cost: float
    expanded: int
    time_sec: float


def astar_grid_8(
    grid: np.ndarray,
    start: Cell,
    goal: Cell,
    allow_corner_cutting: bool = True,
) -> AStarResult:
    t0 = time.perf_counter()

    if start == goal:
        return AStarResult(path=[start], cost=0.0, expanded=0, time_sec=time.perf_counter() - t0)

    if grid[start[0], start[1]] != 0 or grid[goal[0], goal[1]] != 0:
        return AStarResult(path=None, cost=1e30, expanded=0, time_sec=time.perf_counter() - t0)

    open_pq: List[Tuple[float, float, Cell]] = []
    heapq.heappush(open_pq, (octile_distance(start, goal), 0.0, start))  # (f, g, cell)

    g_score: Dict[Cell, float] = {start: 0.0}
    came_from: Dict[Cell, Optional[Cell]] = {start: None}

    expanded = 0
    while open_pq:
        f, g, cur = heapq.heappop(open_pq)
        if g != g_score.get(cur, 1e30):
            continue

        expanded += 1
        if cur == goal:
            path: List[Cell] = []
            node: Optional[Cell] = goal
            while node is not None:
                path.append(node)
                node = came_from[node]
            path.reverse()
            return AStarResult(path=path, cost=g, expanded=expanded, time_sec=time.perf_counter() - t0)

        r, c = cur
        for nb, step_cost in neighbors8(grid, r, c, allow_corner_cutting=allow_corner_cutting):
            ng = g + step_cost
            if ng < g_score.get(nb, 1e30):
                g_score[nb] = ng
                came_from[nb] = cur
                nf = ng + octile_distance(nb, goal)
                heapq.heappush(open_pq, (nf, ng, nb))

    return AStarResult(path=None, cost=1e30, expanded=expanded, time_sec=time.perf_counter() - t0)


# Multi-target Dijkstra (early stop) for distance matrix
def dijkstra_to_targets(
    grid: np.ndarray,
    start: Cell,
    targets: List[Cell],
    allow_corner_cutting: bool = True,
) -> Dict[Cell, float]:
    if not targets:
        return {}

    H, W = grid.shape
    INF = 1e30
    dist = np.full((H, W), INF, dtype=np.float64)

    sr, sc = start
    if grid[sr, sc] != 0:
        return {t: math.inf for t in targets}

    target_set: Set[Cell] = set(targets)
    found: Dict[Cell, float] = {}

    dist[sr, sc] = 0.0
    pq = [(0.0, sr, sc)]

    while pq and len(found) < len(target_set):
        d, r, c = heapq.heappop(pq)
        if d != dist[r, c]:
            continue

        cur = (r, c)
        if cur in target_set and cur not in found:
            found[cur] = d
            if len(found) == len(target_set):
                break

        for (rr, cc), step_cost in neighbors8(grid, r, c, allow_corner_cutting=allow_corner_cutting):
            nd = d + step_cost
            if nd < dist[rr, cc]:
                dist[rr, cc] = nd
                heapq.heappush(pq, (nd, rr, cc))

    for t in targets:
        if t not in found:
            found[t] = math.inf
    return found


def compute_distance_matrix_dijkstra(
    grid: np.ndarray,
    points: List[Cell],
    allow_corner_cutting: bool = True,
) -> np.ndarray:
    n = len(points)
    dist_mat = np.full((n, n), np.inf, dtype=float)

    for i in range(n):
        src = points[i]
        targets = [points[j] for j in range(n) if j != i]
        dmap = dijkstra_to_targets(grid, src, targets, allow_corner_cutting=allow_corner_cutting)
        dist_mat[i, i] = 0.0
        for j in range(n):
            if i != j:
                dist_mat[i, j] = float(dmap.get(points[j], math.inf))

    return dist_mat


def validate_dist_matrix(dist_mat: np.ndarray):
    n = dist_mat.shape[0]
    for i in range(n):
        for j in range(n):
            if i != j and not np.isfinite(dist_mat[i, j]):
                raise ValueError(f"Unreachable pair in distance matrix: {i} -> {j}.")


# input the multi-point tour, use A* to find shortest path between waypoints, output full path
def stitch_with_astar(
    grid: np.ndarray,
    points: List[Cell],
    tour: List[int],
    allow_corner_cutting: bool = True,
) -> Optional[List[Cell]]:
    full_path: List[Cell] = []
    for i in range(len(tour) - 1):
        a_idx, b_idx = tour[i], tour[i + 1]
        start, goal = points[a_idx], points[b_idx]
        res = astar_grid_8(grid, start, goal, allow_corner_cutting=allow_corner_cutting)
        if res.path is None:
            return None
        if not full_path:
            full_path.extend(res.path)
        else:
            full_path.extend(res.path[1:])
    return full_path


# ROS helpers: OccupancyGrid <-> numpy grid, world <-> grid
@dataclass
class MapInfo:
    frame_id: str
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int


def occgrid_to_numpy(
    msg: OccupancyGrid,
    occ_threshold: int = 50,
    treat_unknown_as_obstacle: bool = True,
) -> Tuple[np.ndarray, MapInfo]:
    info = msg.info
    width = info.width
    height = info.height

    data = np.array(msg.data, dtype=np.int16).reshape((height, width))
    grid = np.zeros((height, width), dtype=np.uint8)

    if treat_unknown_as_obstacle:
        grid[data < 0] = 1
    grid[data >= occ_threshold] = 1

    mi = MapInfo(
        frame_id=msg.header.frame_id if msg.header.frame_id else "map",
        resolution=float(info.resolution),
        origin_x=float(info.origin.position.x),
        origin_y=float(info.origin.position.y),
        width=int(width),
        height=int(height),
    )
    return grid, mi


def world_to_grid(mi: MapInfo, x: float, y: float) -> Cell:
    col = int((x - mi.origin_x) / mi.resolution)
    row = int((y - mi.origin_y) / mi.resolution)
    return (row, col)


def grid_to_world(mi: MapInfo, cell: Cell) -> Tuple[float, float]:
    row, col = cell
    x = mi.origin_x + (col + 0.5) * mi.resolution
    y = mi.origin_y + (row + 0.5) * mi.resolution
    return x, y


# Base ROS2 Node
class PathPlanningBase(Node, ABC):
    """
    Base class: handles ROS2 subscriptions/publishing and the shared planning pipeline.

    Derived classes only implement:
      - compute_tour(dist_mat, close_loop) -> list of indices into points
      - optionally: declare_algorithm_parameters()
    """

    def __init__(self, node_name: str):
        super().__init__(node_name)

        # ---------------- Common Parameters ----------------
        self.declare_parameter("cost_map_topic", "/cost_map")
        self.declare_parameter("odom_topic", "/pf/pose/odom")
        self.declare_parameter("waypoints_topic", "/interface/waypoints")
        self.declare_parameter("path_topic", "/planning/raw_path")

        self.declare_parameter("occ_threshold", 50)
        self.declare_parameter("treat_unknown_as_obstacle", True)
        self.declare_parameter("allow_corner_cutting", True)

        # If True: return to start (classic TSP cycle).
        # If False: open tour (start -> ... -> last), algorithm must respect it.
        self.declare_parameter("close_loop", False)

        # Replan throttle (sec)
        self.declare_parameter("min_replan_period_sec", 0.5)
        
        # Distance (in meters) between waypoints for Pure Pursuit
        self.declare_parameter("path_spacing", 0.1)

        # If True: use the incoming waypoint order as the visit sequence
        # (i.e. do NOT run the algorithm's tour computation). Useful for
        # running A* sequentially from waypoint i -> i+1.
        self.declare_parameter("use_waypoint_order", False)
        # Let derived class add its own params (2-opt, exact TSP limits, etc.)
        self.declare_algorithm_parameters()

        # ---------------- QoS ----------------
        map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        # default_qos = 10

        # ---------------- ROS I/O ----------------
        self._cost_map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("cost_map_topic").value,
            self._on_cost_map,
            map_qos,
        )
        self._odom_sub = self.create_subscription(
            Odometry,
            self.get_parameter("odom_topic").value,
            self._on_odom,
            10,
        )
        self._waypoints_sub = self.create_subscription(
            PoseArray,
            self.get_parameter("waypoints_topic").value,
            self._on_waypoints,
            10,
        )

        self._path_pub = self.create_publisher(
            Path,
            self.get_parameter("path_topic").value,
            10,
        )

        # ---------------- State ----------------
        self._grid: Optional[np.ndarray] = None
        self._map_info: Optional[MapInfo] = None
        self._have_map = False

        self._current_pose_xy: Optional[Tuple[float, float]] = None
        self._waypoints_xy: Optional[List[Tuple[float, float]]] = None

        self._last_plan_time = 0.0

        self.get_logger().info(f"{node_name} started (base).")

    # ---------- hooks for derived classes ----------
    def declare_algorithm_parameters(self) -> None:
        """Derived classes may declare extra ROS params."""
        return

    @abstractmethod
    def compute_tour(self, dist_mat: np.ndarray, close_loop: bool) -> List[int]:
        """
        Return a list of indices into `points` that defines the visiting order.
        Must include start at index 0 as the first element.
        If close_loop=True: typically end with 0.
        If close_loop=False: typically do NOT end with 0.
        """
        raise NotImplementedError

    # Callback function definitions
    def _on_cost_map(self, msg: OccupancyGrid):
        treat_unknown = bool(self.get_parameter("treat_unknown_as_obstacle").value)

        info = msg.info
        width = int(info.width)
        height = int(info.height)

        data = np.array(msg.data, dtype=np.int16).reshape((height, width))

        # Planner grid convention: 0=free, 1=obstacle
        grid = np.zeros((height, width), dtype=np.uint8)

        # Unknown handling (-1) if it appears
        if treat_unknown:
            grid[data < 0] = 1
        else:
            grid[data < 0] = 0

        # Explicit: 100 is obstacle, everything else treated as free
        grid[data >= 100] = 1   # (or grid[data == 100] = 1)

        mi = MapInfo(
            frame_id=msg.header.frame_id if msg.header.frame_id else "map",
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            width=width,
            height=height,
        )

        self._grid = grid
        self._map_info = mi
        self._have_map = True
        # self._maybe_replan("cost_map")

    def _on_odom(self, msg: Odometry):
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        self._current_pose_xy = (x, y)
        # self._maybe_replan("odom")

    def _on_waypoints(self, msg: PoseArray):
        self.get_logger().info("Received waypoints.")
        wps: List[Tuple[float, float]] = []
        for p in msg.poses:
            wps.append((float(p.position.x), float(p.position.y)))
        self._waypoints_xy = wps
        if len(wps) == 0:
            self.get_logger().warn("Received empty waypoints.")
            empty_path = Path()
            empty_path.header.frame_id = self._map_info.frame_id if self._map_info else "map"
            empty_path.header.stamp = self.get_clock().now().to_msg()
            empty_path.poses = []
            self._path_pub.publish(empty_path)
            return
        self._maybe_replan("waypoints")

    def _smooth_path_geometry(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smoothens a list of (x, y) coordinates using Cubic Spline Interpolation.
        """
        if len(waypoints) < 3:
            return waypoints  # Not enough points to interpolate a spline

        pts = np.array(waypoints)
        spacing = self.get_parameter("path_spacing").value

        # 1. Remove duplicate points (zero-distance steps)
        diff = np.diff(pts, axis=0)
        dist = np.sqrt((diff**2).sum(axis=1))
        cumulative_dist = np.concatenate(([0], np.cumsum(dist)))
        
        # Filter for unique points based on cumulative distance
        unique_mask = np.concatenate(([True], dist > 1e-5))
        pts = pts[unique_mask]
        cumulative_dist = cumulative_dist[unique_mask]

        if len(pts) < 3:
            return waypoints

        # 2. Fit Splines
        cs_x = CubicSpline(cumulative_dist, pts[:, 0])
        cs_y = CubicSpline(cumulative_dist, pts[:, 1])

        # 3. Resample at even intervals
        total_dist = cumulative_dist[-1]
        num_points = max(int(total_dist / spacing), 2)
        s_new = np.linspace(0, total_dist, num_points)

        smoothed_pts = np.vstack((cs_x(s_new), cs_y(s_new))).T
        return [tuple(p) for p in smoothed_pts]

    # ---------------- Planning driver ----------------
    def _maybe_replan(self, reason: str):
        if not self._have_map or self._grid is None or self._map_info is None:
            return
        if self._current_pose_xy is None or self._waypoints_xy is None:
            return

        min_period = float(self.get_parameter("min_replan_period_sec").value)
        now = time.perf_counter()
        if (now - self._last_plan_time) < min_period:
            return
        self._last_plan_time = now

        try:
            path_msg = self._plan_path()
            if path_msg is not None:
                self._path_pub.publish(path_msg)
                self.get_logger().info(
                    f"Published path with {len(path_msg.poses)} poses (trigger={reason})."
                )
        except Exception as e:
            self.get_logger().error(f"Planning failed (trigger={reason}): {e}")

    def _plan_path(self) -> Optional[Path]:
        assert self._grid is not None and self._map_info is not None
        assert self._current_pose_xy is not None and self._waypoints_xy is not None

        allow_corner_cutting = bool(self.get_parameter("allow_corner_cutting").value)
        close_loop = bool(self.get_parameter("close_loop").value)

        # 1) Build planning points: start + valid waypoint cells
        start_cell = world_to_grid(self._map_info, self._current_pose_xy[0], self._current_pose_xy[1])
        start_cell = self._clamp_cell(start_cell)

        if self._grid[start_cell[0], start_cell[1]] != 0:
            self.get_logger().warn("Start cell is occupied; cannot plan.")
            return None

        waypoint_cells: List[Cell] = []
        for (x, y) in self._waypoints_xy:
            cell = world_to_grid(self._map_info, x, y)
            cell = self._clamp_cell(cell)
            if self._grid[cell[0], cell[1]] != 0:
                self.get_logger().warn(f"Waypoint ({x:.2f},{y:.2f}) -> occupied cell {cell}; skipping.")
                continue
            waypoint_cells.append(cell)

        if len(waypoint_cells) == 0:
            self.get_logger().warn("No valid waypoints (all invalid/occupied).")
            return None

        points: List[Cell] = [start_cell] + waypoint_cells

        # 2) Either compute distance matrix + tour, or use provided waypoint order
        use_given = bool(self.get_parameter("use_waypoint_order").value)
        if not use_given:
            # Distance matrix via Dijkstra (early-stop)
            self.get_logger().info("Using computed waypoint order.")
            t0 = time.perf_counter()
            dist_mat = compute_distance_matrix_dijkstra(self._grid, points, allow_corner_cutting=allow_corner_cutting)
            validate_dist_matrix(dist_mat)
            t1 = time.perf_counter()

            # Algorithm-specific tour
            t2a = time.perf_counter()
            tour = self.compute_tour(dist_mat, close_loop=close_loop)
            t2b = time.perf_counter()
        else:
            # Bypass tour computation: use the incoming waypoint order
            # Points already defined as [start] + waypoint_cells, so visit
            # in that order. Respect close_loop by appending start index.
            self.get_logger().info("Using given waypoint order.")
            t0 = t1 = t2a = t2b = time.perf_counter()
            n = len(points)
            if close_loop:
                tour = list(range(n)) + [0]
            else:
                tour = list(range(n))

        self._validate_tour(tour, n=len(points), close_loop=close_loop)

        # 4) Stitch with A* for each leg
        t3a = time.perf_counter()
        full_cells = stitch_with_astar(self._grid, points, tour, allow_corner_cutting=allow_corner_cutting)
        t3b = time.perf_counter()

        if full_cells is None or len(full_cells) == 0:
            self.get_logger().warn("A* stitching failed for the chosen tour.")
            return None

        # --- SMOOTHING LOGIC INSERTED HERE ---
        # A) Convert grid cells to world coordinates
        # raw_world_points = [grid_to_world(self._map_info, c) for c in full_cells]
        
        # # B) Apply the smoothing
        # smoothed_world_points = self._smooth_path_geometry(raw_world_points)
        
        # # 5) Convert to nav_msgs/Path
        # # path_msg = self._cells_to_path(full_cells)
        # path_msg = self._world_points_to_path(smoothed_world_points)    ## smoothed path
        # 5) Convert A* grid cells directly to nav_msgs/Path (no smoothing)
        path_msg = self._cells_to_path(full_cells)


        self.get_logger().info(
            f"Plan stats: points={len(points)} "
            f"dijkstra={(t1 - t0):.3f}s, tour={(t2b - t2a):.3f}s, stitch={(t3b - t3a):.3f}s, "
            f"path_poses={len(path_msg.poses)}"
        )
        return path_msg

    # ---------------- helpers ----------------
    def _clamp_cell(self, cell: Cell) -> Cell:
        assert self._map_info is not None
        r, c = cell
        r = max(0, min(self._map_info.height - 1, r))
        c = max(0, min(self._map_info.width - 1, c))
        return (r, c)

    def _cells_to_path(self, cells: List[Cell]) -> Path:
        assert self._map_info is not None

        path_msg = Path()
        path_msg.header.frame_id = self._map_info.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for cell in cells:
            x, y = grid_to_world(self._map_info, cell)
            ps = PoseStamped()
            ps.header.frame_id = path_msg.header.frame_id
            ps.header.stamp = path_msg.header.stamp

            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0

            ps.pose.orientation.x = 0.0
            ps.pose.orientation.y = 0.0
            ps.pose.orientation.z = 0.0
            ps.pose.orientation.w = 1.0

            path_msg.poses.append(ps)

        return path_msg

    def _validate_tour(self, tour: List[int], n: int, close_loop: bool) -> None:
        if not tour:
            raise ValueError("Tour is empty.")
        if tour[0] != 0:
            raise ValueError(f"Tour must start with 0, got {tour[0]}")

        if close_loop:
            if tour[-1] != 0:
                raise ValueError("close_loop=True but tour does not end with 0.")
            core = tour[:-1]
        else:
            if tour[-1] == 0 and len(tour) > 1:
                raise ValueError("close_loop=False but tour ends with 0.")
            core = tour

        if len(core) != n:
            raise ValueError(f"Tour must include each node exactly once (core len {len(core)} vs n {n}).")

        if set(core) != set(range(n)):
            raise ValueError("Tour core must be a permutation of 0..n-1.")
            
    def _world_points_to_path(self, points: List[Tuple[float, float]]) -> Path:
        """Helper to convert world (x,y) directly to Path msg."""
        assert self._map_info is not None
        path_msg = Path()
        path_msg.header.frame_id = self._map_info.frame_id
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y in points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)
        return path_msg
