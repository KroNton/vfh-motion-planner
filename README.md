# Motion Planning - VFH Controller

A ROS 2 Nav2 controller plugin implementing the **Vector Field Histogram (VFH)** algorithm for local obstacle avoidance and path tracking.

## Overview

The VFH Controller is a local planner plugin for the [Nav2](https://docs.nav2.org/) navigation stack. It constructs a polar histogram from costmap data to identify obstacle-free directions, then steers the robot toward the goal through open valleys.

### Algorithm Stages

1. **Grid to Polar Histogram** — Scans a local window around the robot in the costmap. For each occupied cell, computes a magnitude based on cost and distance (`m = cost^2 * (window_size - dist)`) and accumulates it in the corresponding angular sector.

2. **Valley Selection** — Identifies consecutive sectors below the obstacle threshold as traversable "valleys". If the goal heading falls within a valley, the robot steers directly toward the goal. Otherwise, it steers toward the closest valley edge.

3. **Velocity Control** — Applies a proportional controller on angular error (`angular_vel = 2.0 * error`). The robot drives at full speed when roughly aligned (error < 0.5 rad), and rotates in place when the heading error is large.

## Dependencies

- ROS 2 (Humble / Iron)
- `nav2_core`, `nav2_costmap_2d`, `nav2_util`
- `geometry_msgs`, `nav_msgs`, `std_msgs`
- `tf2_ros`, `tf2_geometry_msgs`
- `pluginlib`, `angles`

## Building

```bash
cd ~/your_ws
colcon build --packages-select motion_planning
source install/setup.bash
```

## Configuration

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `<plugin_name>.max_linear_vel` | `0.5` | Maximum linear velocity (m/s) |
| `<plugin_name>.max_angular_vel` | `1.0` | Maximum angular velocity (rad/s) |
| `<plugin_name>.active_window_size` | `2.0` | Radius of the local obstacle detection window (m) |
| `<plugin_name>.histogram_sectors` | `72` | Number of angular sectors in the polar histogram (72 = 5 deg/sector) |
| `<plugin_name>.obstacle_threshold` | `50.0` | Histogram magnitude threshold to classify a sector as blocked |

### Nav2 Integration

Add the plugin to your Nav2 parameters file:

```yaml
controller_server:
  ros__parameters:
    controller_plugins: ["VFHController"]
    VFHController:
      plugin: "motion_planning::VFHController"
      max_linear_vel: 0.5
      max_angular_vel: 1.0
      active_window_size: 2.0
      histogram_sectors: 72
      obstacle_threshold: 50.0
```

### Usage with TurtleBot3

This package is designed to work alongside the companion packages in this workspace:

- `turtlebot3_simulations` — Gazebo simulation environment
- `turtlebot3_localization` — AMCL-based localization
- `turtlebot3_navigation` — Nav2 navigation stack configuration

Launch the full navigation stack with VFH as the local controller:

```bash
# 1. Launch simulation
ros2 launch turtlebot3_simulations turtlebot3_world.launch.py

# 2. Launch navigation with VFH controller configured in your nav2 params
ros2 launch turtlebot3_navigation navigation.launch.py
```

## Package Structure

```
motion_planning/
├── CMakeLists.txt
├── package.xml
├── vfh_controller_plugin.xml
├── include/motion_planning/
│   └── vfh_controller.hpp
└── src/
    └── vfh_controller.cpp
```

## How It Works

### Polar Histogram Construction

The controller scans a square window (`active_window_size` radius) around the robot in the costmap. For each occupied cell (cost > 10, excluding unknown space):

```
magnitude = cost^2 * (active_window_size - distance)
angle     = atan2(cell_y - robot_y, cell_x - robot_x)
sector    = angle / (2*pi / histogram_sectors)
histogram[sector] += magnitude
```

This produces a 1D polar histogram where high values indicate obstacle-dense directions.

### Valley Selection Heuristic

Consecutive sectors with histogram values below `obstacle_threshold` form a "valley" (free direction). The algorithm then:

1. If the goal heading lies within a valley — steer directly to goal
2. If the goal is blocked — steer to the nearest valley edge
3. If no valleys exist — turn around (180 deg from current heading)

A safety margin of 4 sectors is applied around obstacles for clearance.

### Velocity Output

```
angular_vel = clamp(2.0 * heading_error, -max_angular_vel, max_angular_vel)
linear_vel  = max_linear_vel   if |heading_error| < 0.5 rad
              0.0              otherwise (rotate in place)
```

## API

The plugin implements the `nav2_core::Controller` interface:

| Method | Description |
|--------|-------------|
| `configure()` | Initializes parameters, costmap reference, and TF buffer |
| `activate()` / `deactivate()` | Lifecycle callbacks |
| `cleanup()` | Resource cleanup |
| `setPlan()` | Receives the global path from the planner |
| `computeVelocityCommands()` | Main loop — builds histogram, selects valley, outputs `TwistStamped` |
| `setSpeedLimit()` | Dynamically adjusts max linear velocity (absolute or percentage) |
