#!/usr/bin/env python3
from tkinter import Grid

from mpl_toolkits.axes_grid1 import Grid
from typing import Optional

from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
import numpy as np
from scipy.ndimage import binary_dilation

from nav_msgs.msg import OccupancyGrid
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math

Grid = List[List[int]]

class MapInflator(Node):
    def __init__(self):
        super().__init__("map_inflator_node")

        # Parameters
        self.declare_parameter("input_map_topic", "/map")
        self.declare_parameter("output_map_topic", "/cost_map")
        self.declare_parameter("inflation_radius_m", 0.25)
        self.declare_parameter("occupied_threshold", 50)
        self.declare_parameter("unknown_is_occupied", True)

        # QoS Settings for Maps (Must be Transient Local to match Map Server)
        self.map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.sub = self.create_subscription(
            OccupancyGrid, 
            self.get_parameter("input_map_topic").value, 
            self._map_callback, 
            self.map_qos
        )
        
        self.inflated_map_pub = self.create_publisher(
            OccupancyGrid, 
            self.get_parameter("output_map_topic").value, 
            self.map_qos
        )
        
        self.map_msg: Optional[OccupancyGrid] = None
        self.grid: Optional[Grid] = None
        self.inflated_grid: Optional[Grid] = None

        self.width = 0
        self.height = 0
        self.resolution = 0.0
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.frame_id = "map"

        self.get_logger().info("Map Inflator Node initialized.")

    def _map_callback(self, msg: OccupancyGrid):
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

        flat_data = []
        for y in range(self.height):
            for x in range(self.width):
                val = 100 if self.inflated_grid[y][x] == 1 else 0
                flat_data.append(val)

        inflated_msg = OccupancyGrid()
        inflated_msg.header = msg.header  # Keep the same frame_id and timestamp
        inflated_msg.info = msg.info      # Keep same resolution, origin, etc.
        inflated_msg.data = flat_data

        self.inflated_map_pub.publish(inflated_msg)
        self.get_logger().info(f"Published inflated map ({self.width}x{self.height})")

        self.get_logger().info("Publishing inflated map...")
    
    def _occupancy_to_grid(self, msg):
        thr = self.get_parameter("occupied_threshold").value
        unknown_occ = self.get_parameter("unknown_is_occupied").value

        grid = [[0]*self.width for _ in range(self.height)]
        neg_count = 0
        for y in range(self.height):
            for x in range(self.width):
                v = msg.data[y*self.width + x]
                if v < 0:
                    neg_count += 1
                    grid[y][x] = 1 if unknown_occ else 0
                else:
                    grid[y][x] = 1 if v >= thr else 0
        self.get_logger().info(f"Total unknown cells: {neg_count}")
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

def main(args=None):
    rclpy.init(args=args)
    node = MapInflator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()