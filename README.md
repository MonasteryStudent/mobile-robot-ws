# Mobile Robot

A learning project focused on developing a mobile robotic system with ROS 2.

## Current Status

- Custom robot description created using URDF and Xacro.
- Robot model visualized in RViz.
- Robot simulated in Gazebo with differential-drive control and joint-state publishing.
- Action-based robot navigation implemented using ROS 2 actions.
- `ros2_control` integration added with mock hardware, joint-state broadcasting, and differential-drive control.
- Downward-facing camera simulated in Gazebo, with images bridged to ROS 2.
- Basic line following implemented using three virtual sensor regions in the camera image, including steering corrections and stopping when no line is detected.

## Project Goals

- Extend the system toward localization, SLAM, and autonomous navigation using the Nav2 stack.

## Workspace Structure

```text
mobile-robot-ws/
├── docs/
├── scripts/
└── src/
    ├── mobile_robot_bringup/
    ├── mobile_robot_description/
    ├── mobile_robot_interfaces/
    ├── mobile_robot_line_following/
    └── mobile_robot_navigation/
```

## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo Harmonic

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

Run commands from the workspace root. In each terminal, first load the environment:

```bash
source scripts/setup.sh
```

### Robot Visualization

Display the robot model in RViz:

```bash
ros2 launch mobile_robot_description display.launch.xml
```

### Gazebo Simulation

Start the robot simulation:

```bash
ros2 launch mobile_robot_bringup mobile_robot.launch.xml
```

### Action-Based Navigation

Start the Gazebo simulation above, then launch the navigation server and client in a separate terminal:

```bash
ros2 launch mobile_robot_bringup navigation.launch.xml
```

The client sends the target position configured in the launch file.

### ros2_control with Mock Hardware

Start the controllers and RViz using mock hardware:

```bash
ros2 launch mobile_robot_bringup ros2_control_mock.launch.xml
```

### Line Following

Start the line-following simulation:

```bash
ros2 launch mobile_robot_bringup line_following.launch.xml
```

Start the line-following controller in a separate terminal:

```bash
ros2 run mobile_robot_line_following line_follower
```

Optionally, open the camera viewer in another terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```

Select `/line_camera/image`.