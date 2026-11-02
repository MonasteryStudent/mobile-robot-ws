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

- Add and simulate LiDAR and IMU sensors.
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
- RViz
- Xacro
- Gazebo
- ros_gz
- ros2_control
- ros2_controllers
- cv_bridge
- rqt_image_view (optional, for viewing camera images)

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

Visualize the robot description in RViz:

```bash
ros2 launch mobile_robot_description display.launch.xml
```

Run the robot simulation in Gazebo:

```bash
ros2 launch mobile_robot_bringup mobile_robot.launch.xml
```

Run the ros2_control setup with mock hardware in RViz:

```bash
ros2 launch mobile_robot_bringup ros2_control_mock.launch.xml
```

Run the line-following simulation in Gazebo:

```bash
ros2 launch mobile_robot_bringup line_following.launch.xml
```

Start the line-following controller in a separate terminal:

```bash
ros2 run mobile_robot_line_following line_follower
```

Optionally, view the camera image in another terminal:

```bash
ros2 run rqt_image_view rqt_image_view
```