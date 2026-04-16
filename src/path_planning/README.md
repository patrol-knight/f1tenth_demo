# Path Planning ROS2 Node (`path_planning`) 

## Map Inflator ROS2 Node (map_inflator)
This package implements a Map Inflation node as a ROS 2 component.

It subscribes to a static occupancy grid map, inflates occupied cells by a configurable radius, and publishes a new inflated cost map suitable for planning and collision avoidance.

The node converts the incoming OccupancyGrid into a binary grid, performs inflation in grid space, and republishes the result as another OccupancyGrid.

### Topics

#### Subscribed Topics
- **`/map`** (`nav_msgs/msg/OccupancyGrid`)  
  The input occupancy map (typically from map_server).
  Cells above the occupied threshold (or unknown cells if configured) are treated as obstacles.

#### Published Topics
- **`/cost_map`** (`nav_msgs/msg/OccupancyGrid`)  
  The inflated occupancy grid.
  Occupied cells are expanded outward by the specified inflation radius.

### Parameters
The node supports the following ROS parameters:

- `input_map_topic` (string, default: `/map`)
  Topic name for the incoming occupancy grid.

- `output_map_topic` (string, default: `/cost_map`)
  Topic name for the inflated occupancy grid.

- `inflation_radius_m` (float, default: `0.25`)
  Inflation radius in meters. Converted internally to grid cells using map resolution.

- `occupied_threshold` (int, default: `50`)
  Cells with values ≥ threshold are considered occupied.

- `unknown_is_occupied` (bool, default: `True`)
  Whether unknown cells (-1) should be treated as obstacles.

### Install
colcon:
```bash
sudo apt install -y python3-colcon-common-extensions
```

Numpy & SciPy:
```bash
sudo apt install -y python3-numpy python3-scipy
```

### Build
```bash 
colcon build --packages-select map_inflator
source install/setup.bash
source /opt/ros/humble/setup.bash
```

### Run(Launch)
```bash
ros2 run map_inflator map_inflator_node
```

### Dummy Test (CLI)
You can test the node without a full navigation stack by manually publishing a small occupancy grid.

#### Step 1: Launch the node

Terminal A:
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 run map_inflator map_inflator_node
```
Open Terminal B and source again:
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
```

#### Step 2: Echo the inflated map

Terminal C:
```bash
ros2 topic echo /cost_map
```

#### Step 3: Publish a dummy map

Terminal B:
```bash
ros2 topic pub --once /map nav_msgs/msg/OccupancyGrid "{
  header: {frame_id: 'map'},
  info: {
    resolution: 0.5,
    width: 5,
    height: 5,
    origin: {position: {x: 0.0, y: 0.0, z: 0.0}}
  },
  data: [
    0, 0, 0, 0, 0,
    0, 0, 100, 0, 0,
    0, 0, 100, 0, 0,
    0, 0, 0, 0, 0,
    0, 0, 0, 0, 0
  ]
}"
```
You should see /cost_map publish a grid where the occupied cells are expanded outward.

Expected Behavior:

The node waits for the first /map message (latched via Transient Local QoS).

Converts occupancy values → binary grid.

Inflates obstacles by radius.

Republishes inflated map.



## Nearest Neighbor + 2-opt

This package implements a **waypoint path planner** as a ROS 2 node.

It subscribes to an occupancy grid map, the robot odometry, and a set of waypoints. It then:
- computes an all-pairs waypoint distance matrix with **Dijkstra (early stop)**,
- constructs a visit order using **Nearest Neighbor (NN)**,
- optionally refines the order using **2-opt** (**closed-loop only**),
- stitches the tour legs using **A\*** on an 8-connected grid,
- smooths the resulting world-coordinate path with a **cubic spline**,
- and publishes the final `nav_msgs/msg/Path`.

---

### Topics

#### Subscribed Topics
- **`/pf/cost_map`** (`nav_msgs/msg/OccupancyGrid`)  
  Occupancy grid used for planning. Cells are treated as obstacles if `occ >= occ_threshold` (and optionally if unknown).

- **`/pf/pose/odom`** (`nav_msgs/msg/Odometry`)  
  The robot current position. Only `x,y` are used to form the start cell.

- **`/interface/waypoints`** (`geometry_msgs/msg/PoseArray`)  
  Waypoints to visit. Only waypoint `x,y` are used.

#### Published Topics
- **`/planning/path`** (`nav_msgs/msg/Path`)  
  Output path as a list of `PoseStamped` points in world coordinates (`x,y`). The path is smoothed and resampled using `path_spacing`.

---

### Parameters

#### Common Parameters (from `PathPlanningBase`)
- `cost_map_topic` (string, default: `/pf/cost_map`)  
  Cost map topic name.

- `odom_topic` (string, default: `/pf/pose/odom`)  
  Odometry topic name.

- `waypoints_topic` (string, default: `/interface/waypoints`)  
  Waypoints topic name.

- `path_topic` (string, default: `/planning/path`)  
  Output path topic name.

- `occ_threshold` (int, default: `50`)  
  Occupancy threshold; `occ >= occ_threshold` treated as obstacle.

- `treat_unknown_as_obstacle` (bool, default: `true`)  
  If `true`, unknown cells (`-1`) are treated as obstacles.

- `allow_corner_cutting` (bool, default: `true`)  
  If `true`, allow diagonal motion even near corners.

- `close_loop` (bool, default: `false`)  
  If `true`, the tour returns to start (classic TSP cycle).

- `min_replan_period_sec` (float, default: `0.5`)  
  Minimum time between replans (throttling).

- `path_spacing` (float, default: `0.1`)  
  Path resampling spacing in meters for cubic-spline smoothing.

#### Algorithm Parameters (NN + 2-opt)
- `two_opt_first_improvement` (bool, default: `true`)  
  If `true`, 2-opt stops at the first improving swap per iteration.

- `two_opt_max_iter` (int, default: `5000`)  
  Maximum number of 2-opt outer iterations.

**Note:** 2-opt is applied only when `close_loop:=true` (closed tour). For open tours, the node returns NN order without 2-opt.

---

### Build

```bash
cd ~/ros2_ws
colcon build --packages-select path_planning
source /opt/ros/humble/setup.bash
source install/setup.bash
```
### Dummy Test

#### Step 1 launch the node

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch path_planning nn_2opt_launch.py
```
#### Step 2 subscribe to three other topics

open three separate terminals and run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```
publish a dummy /pf/cost_map:

```bash
ros2 topic pub /pf/cost_map nav_msgs/msg/OccupancyGrid "{header: {frame_id: 'map'}, info: {resolution: 1.0, width: 15, height: 15, origin: {position: {x: 0.0, y: 0.0, z: 0.0}}}, data: [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,100,100,100,100,0,0,0,0,0,0,0,0,0, 0,0,100,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,100,0,0,0,0,0,0,0,0,0,100,100,0, 0,0,100,0,0,0,0,0,0,0,0,0,100,100,0, 0,0,100,0,0,0,100,100,100,0,0,0,0,0,0, 0,0,100,0,0,0,100,100,100,0,0,0,0,0,0, 0,0,100,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,100,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,100,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,100,100,100,100,100,100,100,100,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]}" -1
```
publish a dummy /pf/pose/odom:

```bash
ros2 topic pub /pf/pose/odom nav_msgs/msg/Odometry "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 1.0, y: 1.0, z: 0.0}}}}" -1
```
publish a dummy /interface/waypoints:

```bash
ros2 topic pub /interface/waypoints geometry_msgs/msg/PoseArray "{header: {frame_id: 'map'}, poses: [{position: {x: 5.0, y: 6.0}}, {position: {x: 13.0, y: 13.0}}, {position: {x: 13.0, y: 2.0}}, {position: {x: 7.0, y: 10.0}}, {position: {x: 1.0, y: 10.0}}]}" -1
```


## Smoothing ROS 2 Node (`smoothing_node`)

This package implements a **path smoothing filter** as a standalone ROS 2 node. 

It is designed to sit between a grid-based path planner (like A* or Dijkstra) and a vehicle controller (like Pure Pursuit). It intercepts jagged, discrete waypoints and refines them into continuous, drivable curves using **Cubic Spline Interpolation**.


### ROS 2 API

#### Subscribed Topics
* **`/planning/raw_path`** (`nav_msgs/msg/Path`)  
  The initial, rough path generated by the path planning algorithm. 

#### Published Topics
* **`/planning/path`** (`nav_msgs/msg/Path`)  
  The smoothed output path, resampled at fixed intervals.

#### Parameters
* **`path_spacing`** (float, default: `0.1`)  
  The desired distance (in meters) between consecutive waypoints along the newly smoothed path. Smaller values yield denser points.

---
### Dummy Test
```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /planning/path
```
```
source /opt/ros/humble/setup.bash
ros2 topic pub --once /planning/raw_path nav_msgs/msg/Path "{header: {frame_id: 'map'}, poses: [{pose: {position: {x: 0.0, y: 0.0}}}, {pose: {position: {x: 2.0, y: 0.0}}}, {pose: {position: {x: 2.0, y: 2.0}}}]}"
```


