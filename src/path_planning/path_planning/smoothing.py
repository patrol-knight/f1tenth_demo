#!/usr/bin/env python3
"""
testing command:
general:
ros2 topic pub --once /planning/raw_path nav_msgs/msg/Path "{header: {frame_id: 'map'}, poses: [{pose: {position: {x: 0.0, y: 0.0}}}, {pose: {position: {x: 2.0, y: 0.0}}}, {pose: {position: {x: 2.0, y: 2.0}}}]}"

empty:
ros2 topic pub --once /planning/raw_path nav_msgs/msg/Path "{header: {frame_id: 'map'}, poses: []}"


"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from scipy.interpolate import splprep, splev

from typing import List, Tuple
from dataclasses import dataclass

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped

# --- Helper Classes & Functions for Map ---
@dataclass
class MapInfo:
    frame_id: str
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

def world_to_grid(mi: MapInfo, x: float, y: float) -> Tuple[int, int]:
    col = int((x - mi.origin_x) / mi.resolution)
    row = int((y - mi.origin_y) / mi.resolution)
    return (row, col)

class SmoothingNode(Node):
    def __init__(self):
        super().__init__('smoothing_node')
        
        # Parameters
        self.declare_parameter("path_spacing", 0.1) # Distance between points in smoothed path
        self.declare_parameter("raw_path_topic", "/planning/raw_path") # Topic for raw paths
        self.declare_parameter("inflation_map_topic", "/cost_map") # Topic for collision checking
        self.declare_parameter("occ_threshold", 50) # Threshold to consider cell as obstacle
        
        # Standard QoS for Map and Paths
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # Map Subscriber
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.get_parameter("inflation_map_topic").value,
            self.map_callback,
            self.map_qos
        )

        # Raw Path Subscriber
        self.path_sub = self.create_subscription(
            Path,
            self.get_parameter("raw_path_topic").value,
            self.path_callback,
            10
        )

        # Smoothed Path Publisher
        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )
        
        # Internal State for Collision Checking
        self._grid = None
        self._map_info = None

        self.get_logger().info("Smoothing Node started.")

    def map_callback(self, msg: OccupancyGrid):
        """Processes the occupancy grid for collision checking."""
        info = msg.info
        width = int(info.width)
        height = int(info.height)
        
        data = np.array(msg.data, dtype=np.int16).reshape((height, width))
        grid = np.zeros((height, width), dtype=np.uint8)
        
        # Treat unknown (-1) as obstacle for safety, and apply threshold
        grid[data < 0] = 1
        threshold = self.get_parameter("occ_threshold").value
        grid[data >= threshold] = 1

        self._map_info = MapInfo(
            frame_id=msg.header.frame_id if msg.header.frame_id else "map",
            resolution=float(info.resolution),
            origin_x=float(info.origin.position.x),
            origin_y=float(info.origin.position.y),
            width=width,
            height=height
        )
        self._grid = grid

    def path_callback(self, msg: Path):
        # Edge Case: Received empty path. Publish empty path to Pure Pursuit immediately.
        if len(msg.poses) == 0:
            self.get_logger().warn("Received empty raw_path. Publishing empty path.")
            self.path_pub.publish(msg)
            return

        # Path is too short to mathematically smooth
        if len(msg.poses) < 3:
            self.get_logger().info("Path too short to smooth. Passing through.")
            self.path_pub.publish(msg)
            return

        # 1. Extract (x, y) coordinates from the incoming Path message
        raw_waypoints = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # 2. Apply smoothing and perform collision checking
        smoothed_points = self._smooth_path_geometry(raw_waypoints)
        
        # 3. Convert back to a Path message
        smoothed_msg = Path()
        smoothed_msg.header = msg.header # Keep the original frame_id and stamp
        
        for x, y in smoothed_points:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0 # Default neutral orientation
            
            smoothed_msg.poses.append(pose)
            
        # 4. Publish the smoothed path
        self.path_pub.publish(smoothed_msg)
        self.get_logger().info(f"Published smoothed path with {len(smoothed_msg.poses)} poses.")

    def _smooth_path_geometry(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        """
        Smoothens a list of (x, y) coordinates using B-Spline Approximation and checks for collisions.
        """
        pts = np.array(waypoints)
        spacing = self.get_parameter("path_spacing").value

        # 1. Remove duplicate spots
        diff = np.diff(pts, axis=0)
        dist = np.sqrt((diff**2).sum(axis=1))
        unique_mask = np.concatenate(([True], dist > 1e-5))
        pts = pts[unique_mask]

        if len(pts) < 3:
            return waypoints

        # 2. BSpline Fitting
        smoothing_factor = 0.5
        try:
            tck, u = splprep([pts[:, 0], pts[:, 1]], s=smoothing_factor, k=min(3, len(pts)-1))
        except ValueError as e:
            self.get_logger().warn(f"Spline fitting failed: {e}. Falling back to raw path.")
            return waypoints

        path_length = np.sum(dist)
        num_points = max(int(path_length / spacing), 2)
        
        # 3. Generate new poses
        u_new = np.linspace(0, 1.0, num_points)
        x_new, y_new = splev(u_new, tck)
        smoothed_pts = np.vstack((x_new, y_new)).T
        
        # 4. Collision Checking
        final_path = []
        if self._grid is not None and self._map_info is not None:
            for x, y in smoothed_pts:
                r, c = world_to_grid(self._map_info, x, y)
                
                # Clamp coordinates to grid boundaries just in case
                r = max(0, min(self._map_info.height - 1, r))
                c = max(0, min(self._map_info.width - 1, c))
                
                if self._grid[r, c] != 0:
                    self.get_logger().warn("Smoothed path collides with obstacle! Falling back to raw path.")
                    dists_sq = np.sum((pts - np.array([x, y]))**2, axis=1)
                    closest_idx = np.argmin(dists_sq)
                    safe_pt = tuple(pts[closest_idx])
                    
                    # Only append if it's not a duplicate of the last point added
                    if not final_path or safe_pt != final_path[-1]:
                        final_path.append(safe_pt)
                else:
                    # Point is safe, add as is
                    final_path.append((x, y))

        return [tuple(p) for p in final_path]

def main(args=None):
    rclpy.init(args=args)
    node = SmoothingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()