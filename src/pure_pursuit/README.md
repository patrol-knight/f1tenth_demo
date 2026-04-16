# Pure Pursuit ROS2 Node (`pure_pursuit`)

This package implements a **Pure Pursuit** controller as a ROS 2 node.

It subscribes to a planned path and the robot state, computes the pure pursuit curvature, converts it into a steering angle (degrees), and publishes the steering command.

---

## Topics

### Subscribed Topics
- **`/planning/path`** (`nav_msgs/msg/Path`)  
  The target path as a list of `PoseStamped` points (only `x,y` are used).

- **`/pf/pose/odom`** (`nav_msgs/msg/Odometry`)  
  The robot current pose. Heading is extracted from the quaternion orientation.

### Published Topics
- **`/drive`** (`ackermann_msgs/msg/AckermannDriveStamped`)  
  The combined steering and velocity command sent to the motor/servo controllers..

---

## Parameters

The node supports the following ROS parameters:

- `lookahead` (float, default: `1.0`)  
  Lookahead distance.

- `wheelbase` (float, default: `0.30`)  
  Vehicle wheelbase used to convert curvature → steering angle.

- `control_rate_hz` (float, default: `20.0`)  
  Control loop rate (currently unused if steering is computed on state callback).

- `max_steer_deg` (float, default: `30.0`)  
  Steering output clamp in degrees.

---

## Install
colcon:
```bash
sudo apt install -y python3-colcon-common-extensions
```
tf-transformations:
```bash
sudo apt install -y ros-humble-tf-transformations
```
pip3:
```bash
sudo apt install -y python3-pip
```
transforms3d:
```bash
python3 -m pip install transforms3d
```


## Build

```bash
colcon build --packages-select pure_pursuit
source install/setup.bash
source /opt/ros/humble/setup.bash
```


## Run (Launch)
```bash
ros2 launch pure_pursuit pure_pursuit_launch.py
```

To override the parameters file at launch:
```bash
ros2 launch pure_pursuit pure_pursuit_launch.py params_file:=/path/to/your_params.yaml
```

## Dummy Test (CLI)

You can test the node without any simulator by manually publishing a dummy /path and /state from the command line and then reading /steering.

Step 1: Launch the node

Terminal A:
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
ros2 launch pure_pursuit pure_pursuit_launch.py
```

Open Terminal B (and C) and source again:
```bash
source install/setup.bash
source /opt/ros/humble/setup.bash
```

Important: For --once publishing, the node must already be running before you publish.

Step 2: Read the dummy steering output

Terminal C:
```bash
ros2 topic echo /drive
```

Step 3: Publish a dummy /path once

Terminal B:
```bash
ros2 topic pub -r 10 /pf/pose/odom nav_msgs/msg/Odometry "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 1.0, y: 0.5, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    }
  }
}"
```
Step 4: Publish a dummy /state once

This example publishes a robot pose at (x=0.5, y=0.0) with yaw = 0 rad (quaternion = (0,0,0,1)):

Terminal B:
```bash
ros2 topic pub --once /planning/path nav_msgs/msg/Path "{
  header: {frame_id: 'map'},
  poses: [
    {pose: {position: {x: 0.0, y: 0.0, z: 0.0}}},
    {pose: {position: {x: 2.0, y: 0.0, z: 0.0}}},
    {pose: {position: {x: 5.0, y: 0.0, z: 0.0}}},
    {pose: {position: {x: 10.0, y: 0.0, z: 0.0}}}
  ]
}"
```

Expected output example:
```bash
data: -0.26215294003486633
```
---

Debugging Tips
Check node subscriptions and publishers
```bash
ros2 node info /pure_pursuit_node
```
Check topic info
```bash
ros2 topic info /path
ros2 topic info /state
ros2 topic echo /drive
```

License

MIT License
Copyright (c) 2026 Hongyi "Sam" Dong
