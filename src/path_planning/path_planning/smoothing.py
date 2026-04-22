#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import numpy as np
from scipy.interpolate import splprep, splev
from typing import List, Tuple

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class SmoothingNode(Node):
    def __init__(self):
        super().__init__('smoothing_node')
        
        # Parameters
        self.declare_parameter("path_spacing", 0.1) # Distance between points in smoothed path
        self.declare_parameter("raw_path", "/planning/raw_path") 
        
        self.path_sub = self.create_subscription(
            Path,
            '/planning/raw_path',
            self.path_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            '/planning/path',
            10
        )
        
        self.get_logger().info("Smoothing Node started.")

    def calculate_smoothness(self, points: np.ndarray) -> float:
        """
        Calculates the average turning angle along the path as per the formula:
        Smoothness(p) = (1/n) * sum(Angle[pi, pi+1, pi+2])
        """
        n_points = len(points)
        if n_points < 3:
            return 0.0

        total_angle = 0.0
        count = 0

        for i in range(n_points - 2):
            p1 = points[i]
            p2 = points[i+1]
            p3 = points[i+2]

            # Vector p1 -> p2 and p2 -> p3
            v1 = p2 - p1
            v2 = p3 - p2

            # Euclidean distances (dis)
            d1 = np.linalg.norm(v1)
            d2 = np.linalg.norm(v2)

            # Avoid division by zero for overlapping points
            if d1 < 1e-6 or d2 < 1e-6:
                continue

            # Dot product calculation for the cos^-1 term
            dot_product = np.dot(v1, v2)
            cosine_val = np.clip(dot_product / (d1 * d2), -1.0, 1.0)
            
            # Angle [p1, p2, p3] = pi - cos^-1(...)
            angle = np.pi - np.arccos(cosine_val)
            total_angle += angle
            count += 1

        if count == 0:
            return 0.0
            
        # Return average angle in degrees for better readability in logs
        return np.degrees(total_angle / count)

    def path_callback(self, msg: Path):
        if len(msg.poses) < 3:
            self.get_logger().warn("Path too short to smooth.")
            self.path_pub.publish(msg)
            return

        raw_waypoints = np.array([(p.pose.position.x, p.pose.position.y) for p in msg.poses])
        
        # Calculate smoothness of raw path
        raw_smoothness = self.calculate_smoothness(raw_waypoints)
        
        # Apply smoothing
        smoothed_points_list = self._smooth_path_geometry(raw_waypoints.tolist())
        smoothed_points_np = np.array(smoothed_points_list)
        
        # Calculate smoothness of processed path
        new_smoothness = self.calculate_smoothness(smoothed_points_np)
        
        # Convert back to a Path message
        smoothed_msg = Path()
        smoothed_msg.header = msg.header
        
        for x, y in smoothed_points_list:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            smoothed_msg.poses.append(pose)
            
        self.path_pub.publish(smoothed_msg)
        
        # Enhanced Log Output
        self.get_logger().info(
            f"Path Processed: Raw Smoothness: {raw_smoothness:.2f} | "
            f"Smoothed Smoothness: {new_smoothness:.2f}° | "
            f"Poses: {len(smoothed_msg.poses)} |"
            f"Average smoothness: {len(smoothed_msg.poses)/new_smoothness+ 5}"
        )

    def _smooth_path_geometry(self, waypoints: List[Tuple[float, float]]) -> List[Tuple[float, float]]:
        pts = np.array(waypoints)
        spacing = self.get_parameter("path_spacing").value

        # Remove duplicate spots
        diff = np.diff(pts, axis=0)
        dist = np.sqrt((diff**2).sum(axis=1))
        unique_mask = np.concatenate(([True], dist > 1e-5))
        pts = pts[unique_mask]

        if len(pts) < 3:
            return waypoints

        smoothing_factor = 0.5
        try:
            tck, u = splprep([pts[:, 0], pts[:, 1]], s=smoothing_factor, k=min(3, len(pts)-1))
        except Exception as e:
            self.get_logger().warn(f"Spline fitting failed: {e}")
            return waypoints

        path_length = np.sum(dist)
        num_points = max(int(path_length / spacing), 2)
        
        u_new = np.linspace(0, 1.0, num_points)
        x_new, y_new = splev(u_new, tck)

        smoothed_pts = np.vstack((x_new, y_new)).T
        return [tuple(p) for p in smoothed_pts]

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
