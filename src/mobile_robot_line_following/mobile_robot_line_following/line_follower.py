#!/usr/bin/env python3

from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class LineFollower(Node):
    def __init__(self):
        super().__init__("line_follower")

        self.bridge = CvBridge()
        self.first_image_received = False
        self.left_brightness = None
        self.right_brightness = None
        self.center_brightness = None

        self.sensor_log_timer = self.create_timer(
            1.0,
            self.log_sensor_values,
        )

        self.image_subscription = self.create_subscription(
            Image,
            "/line_camera/image",
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

    def follow_line(self):
        command = Twist()

        if self.left_on_line and self.right_on_line:
            # Both dark: stop.
            pass
        elif self.left_on_line:
            command.linear.x = 0.06
            command.angular.z = 0.1
        elif self.right_on_line:
            command.linear.x = 0.06
            command.angular.z = -0.1
        elif self.center_on_line:
            command.linear.x = 0.10
        # Otherwise all three sensors are light: keep the stop command.

        self.velocity_publisher.publish(command)

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(
            msg,
            desired_encoding="mono8",
        )

        left_region = image[110:130, 70:90]
        right_region = image[110:130, 230:250]
        center_region = image[110:130, 150:170]

        self.left_brightness = float(left_region.mean())
        self.right_brightness = float(right_region.mean())
        self.center_brightness = float(center_region.mean())

        self.left_on_line = self.left_brightness < 108
        self.right_on_line = self.right_brightness < 108
        self.center_on_line = self.center_brightness < 108

        self.follow_line()

        if not self.first_image_received:
            height, width = image.shape
            self.get_logger().info(
                f"First image received: {width} x {height} pixels"
            )
            self.first_image_received = True

    def log_sensor_values(self):
        if (
            self.left_brightness is None
            or self.right_brightness is None
            or self.center_brightness is None
        ):
            return

        left_state = "DARK" if self.left_on_line else "LIGHT"
        right_state = "DARK" if self.right_on_line else "LIGHT"
        center_state = "DARK" if self.center_on_line else "LIGHT"

        self.get_logger().info(
            f"Left: {self.left_brightness:.1f} ({left_state}), "
            f"Right: {self.right_brightness:.1f} ({right_state}), "
            f"Center: {self.center_brightness:.1f} ({center_state})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
