# Mobile Robot

A learning project focused on developing a mobile robotic system with ROS 2.

## Project Goals

- Design and structure a custom robot description using URDF and Xacro.
- Visualize and validate the robot model in RViz.
- Simulate the robot and its LiDAR and IMU sensors in Gazebo.
- Implement action-based robot navigation using ROS 2 actions.
- Extend the system toward localization, SLAM, and autonomous navigation using the Nav2 stack.

## Workspace Structure

```text
src/
└── mobile_robot_description/
    ├── launch/
    ├── rviz/
    └── urdf/
```

The workspace will be extended with additional ROS 2 packages as the project grows.

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- RViz
- Xacro

Additional dependencies such as Gazebo will be added as the project progresses.

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