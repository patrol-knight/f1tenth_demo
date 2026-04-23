# MIT License
#
# Copyright (c) 2026 Hongyi "Sam" Dong
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the 'Software'), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

#!/usr/bin/env python3
"""
Pure Pursuit controller utilities + ROS2 node.

Your original pure_pursuit() returns a curvature kappa.
This node converts curvature -> steering angle (degrees) and publishes it.
"""

from typing import Sequence, Tuple, Optional, List

import numpy as np

# ---- ROS2 imports ----
import rclpy
from rclpy.node import Node
import tf_transformations

from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker

def _wrap_angle(a: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return (a + np.pi) % (2 * np.pi) - np.pi


def pure_pursuit(
    path: Sequence[Sequence[float]],
    state: Sequence[float],
    lookahead: float
) -> Tuple[np.ndarray, float]:
    """
    Compute lookahead point and curvature using the Pure Pursuit algorithm.

    Args:
        path: sequence of (x, y) coordinates (Nx2) describing the path.
        state: current vehicle state as (x, y, theta) where theta is heading radians.
        lookahead: look-ahead distance (scalar > 0).

    Returns:
        (lookahead_point, curvature)
    """
    if lookahead <= 0:
        raise ValueError("lookahead must be positive")

    path_arr = np.asarray(path, dtype=float)
    if path_arr.ndim != 2 or path_arr.shape[1] != 2:
        raise ValueError("path must be Nx2 array-like of (x,y), currently has shape " + str(path_arr.shape))

    x, y, theta = float(state[0]), float(state[1]), float(state[2])

    path_arr = np.asarray(path)

    deltas_all = path_arr - np.array([x, y])
    dists_all = np.hypot(deltas_all[:, 0], deltas_all[:, 1])
    closest_idx = np.argmin(dists_all)

    relevant_path = path_arr[closest_idx:]
    relevant_deltas = relevant_path - np.array([x, y])
    relevant_dists = np.hypot(relevant_deltas[:, 0], relevant_deltas[:, 1])

    idx = np.where(relevant_dists >= lookahead)[0]
    
    if idx.size > 0:
        la_pt = relevant_path[idx[0]]
    else:
        la_pt = path_arr[-1]

    dx = la_pt[0] - x
    dy = la_pt[1] - y
    angle_to_point = np.arctan2(dy, dx)
    alpha = _wrap_angle(angle_to_point - theta)

    curvature = 2.0 * np.sin(alpha) / float(lookahead)

    return la_pt.copy(), float(curvature)


def compute_cross_track_error(
    path: Sequence[Sequence[float]],
    curr_x: float, curr_y:float
) -> float:
    """
    Compute cross-track error as the minimum Euclidean distance from a point
    to the piecewise-linear path.
    """
    path_arr = np.asarray(path, dtype=float)
    point_arr = np.array([curr_x, curr_y])
    if path_arr.ndim != 2 or path_arr.shape[1] != 2:
        raise ValueError("path must be Nx2 array-like of (x,y), currently has shape " + str(path_arr.shape))

    if len(path_arr) == 0:
        raise ValueError("path must not be empty")


    
    dist_sq = np.sum((path_arr-point_arr)**2, axis = 1)
    closest_idx = np.argmin(dist_sq)

    candidate_segment = []
    if closest_idx > 0:
        candidate_segment.append((closest_idx - 1, closest_idx))
    if closest_idx < len(path_arr) - 1:
        candidate_segment.append((closest_idx, closest_idx + 1))


    min_dist = float('inf')
    for start_idx, end_idx in candidate_segment:
        A = path_arr[start_idx]
        B = path_arr[end_idx]
        ab = B-A
        ap = point_arr - A
        seg_len_sq = np.dot(ab,ab)

        if seg_len_sq == 0 :
            dist = np.linalg.norm(point_arr - A)
        else:
            t = np.dot(ap,ab)/seg_len_sq
            t = np.clip(t, 0.0, 1.0)
            closest_on_seg = A + t*ab
            dist = np.linalg.norm(point_arr - closest_on_seg)

        if dist < min_dist:
            min_dist = dist
        
    return float(min_dist)


class PurePursuitNode(Node):
    """
    Subscribes:
      - /planning/path (nav_msgs/msg/Path): The sequence of waypoints the robot is trying to follow.
      - /pf/pose/odom (nav_msgs/msg/Odometry): The current estimated state (position and orientation) of the robot from the Localization (Particle Filter) unit.

    Publishes:
      - /drive (ackermann_msgs/msg/AckermannDriveStamped): The combined steering and velocity command sent to the motor/servo controllers.
    """

    def __init__(self) -> None:
        super().__init__("pure_pursuit_node")

        # ---- Parameters ----
        self.declare_parameter("lookahead", 0.6)       # meters (or your grid units)
        self.declare_parameter("wheelbase", 0.30)      # meters
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("max_steer_deg", 25.0)  # clamp output

        self.lookahead: float = float(self.get_parameter("lookahead").value)
        self.wheelbase: float = float(self.get_parameter("wheelbase").value)
        self.control_rate_hz: float = float(self.get_parameter("control_rate_hz").value)
        self.max_steer_deg: float = float(self.get_parameter("max_steer_deg").value)

        # ---- Internal state ----
        self._path_xy: Optional[np.ndarray] = None 
        self._state: Optional[Odometry] = None
        self._cte_m: Optional[float] = None
        self._max_abs_cte_m: float = 0.0

        # ---- ROS interfaces ----
        self._path_sub = self.create_subscription(Path, "/planning/path", self._on_path, 10)
        self._state_sub = self.create_subscription(Odometry, "/pf/pose/odom", self._on_odom, 10)
        self._drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)
        self._marker_pub = self.create_publisher(Marker, "/viz/lookahead_point", 10)

        self.get_logger().info(
            f"PurePursuitNode up. lookahead={self.lookahead}, wheelbase={self.wheelbase}, "
            f"rate={self.control_rate_hz}Hz, max_steer_deg={self.max_steer_deg}"
        )
    
    def publish_lookahead_marker(self, pt: np.ndarray):
        """Helper to publish a green sphere at the lookahead target."""
        marker = Marker()
        marker.header.frame_id = "map"  # Ensure this matches your path frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "lookahead"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = pt[0]
        marker.pose.position.y = pt[1]
        marker.pose.position.z = 0.1 # Slightly elevated so it's not buried in the ground
        
        # Scale (0.2m sphere)
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color (Green)
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0 # Alpha must be non-zero!
        
        self._marker_pub.publish(marker)

    def _on_path(self, msg: Path) -> None:
        # Convert nav_msgs/Path -> Nx2 array
        self.get_logger().info("Received Path")
        pts: List[List[float]] = []
        for ps in msg.poses:
            pts.append([ps.pose.position.x, ps.pose.position.y])

        if len(pts) == 0:
            self._path_xy = None
            self._cte_m = None
            self._max_abs_cte_m = 0.0
            self.get_logger().warn("Received empty path.")
            return

        self._path_xy = np.asarray(pts, dtype=float)
        self._cte_m = None
        self._max_abs_cte_m = 0.0

    def _on_odom(self, msg: Odometry) -> None:
        # Pose2D: x, y, theta (theta expected in radians)
        self._state = msg
            
        # path empty check
        if self._path_xy is None:
            self.get_logger().warn(
                "No path received yet, cannot compute steering.",
                throttle_duration_sec=5.0
            )
            return

        curr_x = self._state.pose.pose.position.x
        curr_y = self._state.pose.pose.position.y
        self._cte_m = compute_cross_track_error(self._path_xy, curr_x, curr_y)
        self._max_abs_cte_m = max(self._max_abs_cte_m, abs(self._cte_m))

        self.get_logger().info(
            f"CTE={self._cte_m * 100.0:.2f} cm, max CTE={self._max_abs_cte_m * 100.0:.2f} cm"
        )

        goal_pt = self._path_xy[-1]

        dist_to_goal = np.sqrt((goal_pt[0] - curr_x)**2 + (goal_pt[1] - curr_y)**2)
        
        # Run pure pursuit -> curvature
        try:
            la_pt, kappa = pure_pursuit(
                self._path_xy,
                (self._state.pose.pose.position.x, self._state.pose.pose.position.y, tf_transformations.euler_from_quaternion([self._state.pose.pose.orientation.x,
                                                                                                     self._state.pose.pose.orientation.y,
                                                                                                     self._state.pose.pose.orientation.z,
                                                                                                     self._state.pose.pose.orientation.w])[2]),
                self.lookahead
            )
            self.publish_lookahead_marker(la_pt)
        except Exception as e:
            self.get_logger().warn(f"pure_pursuit failed: {e}")
            return

        # Convert curvature -> steering angle (bicycle model)
        # delta = atan(L * kappa)
        delta_rad = float(np.arctan(self.wheelbase * kappa))
        max_steer_rad = np.radians(self.max_steer_deg)
        delta_rad = np.clip(delta_rad, -max_steer_rad, max_steer_rad)
        # Prepare Ackermann message
        delta_rad = np.clip(delta_rad, -max_steer_rad, max_steer_rad)
        delta_rad = np.clip(delta_rad, -max_steer_rad, max_steer_rad)
        # Prepare Ackermann message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "base_link"
        
        drive_msg.drive.steering_angle = delta_rad 
        arrival_threshold = 0.3
        if dist_to_goal < arrival_threshold:
            self.get_logger().info("Goal Reached! Stopping.", throttle_duration_sec=5.0)
            drive_msg.drive.speed = 0.0
            drive_msg.drive.steering_angle = 0.0
        else:
            drive_msg.drive.speed = 0.7
            drive_msg.drive.steering_angle = delta_rad

        self._drive_pub.publish(drive_msg)

def main(args=None):
    rclpy.init(args=args)
    pf = PurePursuitNode()
    rclpy.spin(pf)


if __name__ == "__main__":
    main()
