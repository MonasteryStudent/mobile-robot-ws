# Mobile Robot

A learning project focused on developing a mobile robotic system with ROS 2.

## Current Status

- Custom robot description created using URDF and Xacro.
- Robot model visualized in RViz.
- Robot simulated in Gazebo with differential-drive control and joint-state publishing.

## Project Goals

- Add and simulate LiDAR and IMU sensors.
- Implement action-based robot navigation using ROS 2 actions.
- Extend the system toward localization, SLAM, and autonomous navigation using the Nav2 stack.

## Workspace Structure

```text
mobile-robot-ws/
├── README.md
├── scripts/
└── src/
    ├── mobile_robot_bringup/
    └── mobile_robot_description/
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- RViz
- Xacro
- Gazebo
- ros_gz

## Build

```bash
colcon build
```

Source the workspace:

```bash
source install/setup.bash
```

Alternatively, you can use the provided helper script, which sources both the ROS 2 installation and the workspace:

```bash
source scripts/setup.sh
```

## Run

Visualize the current robot description in RViz:

```bash
ros2 launch mobile_robot_description display.launch.xml
```

Run the current robot simulation in Gazebo:

```bash
ros2 launch mobile_robot_bringup mobile_robot.launch.xml
```