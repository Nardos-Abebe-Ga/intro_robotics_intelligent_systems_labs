# Lab 1: ROS 2 and Custom Robot Simulation

**Program:** BSc in Software Engineering  
**Course:** Introduction to Robotics and Intelligent Systems  
**Academic Year:** 2026

## Overview

This project extends the `diffdrive_bot` differential-drive robot simulation using ROS 2 Humble and Gazebo Harmonic.

The exercise adds a rear depth camera and an IMU sensor, implements obstacle avoidance, introduces custom ROS 2 interfaces, and uses services and actions to control the robot.

## Packages

| Package | Description |
|---|---|
| `diffdrive_bot` | Differential-drive robot model, Gazebo world, sensors, and simulation configuration. |
| `lab_interfaces` | Existing message and action interfaces, including `ObstacleInfo` and `DriveDistance`. |
| `my_robot_interfaces` | Custom `RobotStatus` message, `SetTurnSpeed` service, and `Rotate` action. |
| `my_robot_nodes` | Python nodes for obstacle avoidance, rotation, and square driving. |

## Implemented Features

### 1. Robot Sensors
- Added a rear-facing depth camera.
- Configured depth images, point clouds, and camera information.
- Added an IMU sensor at the center of the robot.
- Configured Gazebo-to-ROS 2 topic bridges.

### 2. Obstacle Avoidance
- Subscribes to `/obstacle`.
- Publishes velocity commands to `/cmd_vel`.
- Moves forward when the path is clear.
- Turns away from detected obstacles.

### 3. Parameters and Custom Status
- `forward_speed`: Default 0.2 m/s.
- `turn_speed`: Default 0.5 rad/s.
- `max_turn_speed`: Default 1.5 rad/s.
- Publishes robot status to `/robot_status`.

### 4. Services
- `/turn_away_enable`: Enables or disables obstacle avoidance.
- `/set_turn_speed`: Changes the turning speed within the allowed maximum.

### 5. Actions
- `/rotate`: Rotates the robot by a requested angle using odometry.
- `/drive_distance`: Uses the existing drive-distance action server.

### 6. Square Driving
The `square_driver` action client drives the robot around a square by combining four drive-distance goals and four 90-degree rotation goals.

The default side length is 0.5 meters.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Harmonic
- ROS-Gazebo integration packages
- Python 3

## Setup and Installation

Clone your fork:

```bash
cd ~/ros2_ws/src
git clone https://github.com/Nardos-Abebe-Ga/intro_robotics_intelligent_systems_labs.git
```

Create a separate working folder and copy the four packages:

```bash
mkdir -p ~/ros2_ws/src/lab1_exercise

cd ~/ros2_ws/src/intro_robotics_intelligent_systems_labs/lab_01

cp -a diffdrive_bot lab_interfaces my_robot_interfaces my_robot_nodes ~/ros2_ws/src/lab1_exercise/
```

Build the workspace:

```bash
cd ~/ros2_ws

source /opt/ros/humble/setup.bash

colcon build --symlink-install

source install/setup.bash
```

## Running the Simulation

Launch the complete square-driving demonstration:

```bash
ros2 launch my_robot_nodes square.launch.py
```

To change the square side length:

```bash
ros2 launch my_robot_nodes square.launch.py side_length:=0.5
```

The robot should complete four sides and report:

```text
Square completed successfully!
```

## Useful ROS 2 Commands

List active nodes:

```bash
ros2 node list
```

List topics:

```bash
ros2 topic list
```

Inspect the robot status:

```bash
ros2 topic echo /robot_status
```

Inspect available actions:

```bash
ros2 action list
```

## Screenshots

The `screenshots/` folder contains evidence of the completed exercise, including:

- Robot simulation in Gazebo.
- Depth camera image and point cloud in RViz.
- ROS 2 node communication graph using `rqt_graph`.
- Square-driving demonstration.

## Repository

GitHub: https://github.com/Nardos-Abebe-Ga/intro_robotics_intelligent_systems_labs

## License

This project uses the MIT License. See the repository's `LICENSE` file.
