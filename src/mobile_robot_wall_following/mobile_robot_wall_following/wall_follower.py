#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import math
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.first_scan_received = False

        self.sensor_log_timer = self.create_timer(
            1.0,
            self.log_sensor_values,
        )

        self.scan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos_profile_sensor_data,
        )

    def sector_distance(self, msg, min_deg, max_deg):
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)

        min_distance = float('inf')

        for i, distance in enumerate(msg.ranges):
            curr_angle = msg.angle_min + i * msg.angle_increment

            if (min_angle <= curr_angle <= max_angle 
                and msg.range_min <= distance <= msg.range_max
                and distance < min_distance):
                    min_distance = distance

        if min_distance == float('inf'):
            return None

        return min_distance

    def scan_callback(self, msg):
        if not self.first_scan_received:
            num_ranges = len(msg.ranges)
            self.get_logger().info(
                f"First scan received: {num_ranges} ranges"
            )
            self.first_scan_received = True

        min_deg_front = -5.0
        max_deg_front = 5.0
        min_deg_right = -95.0
        max_deg_right = -85.0

        self.distance_front = self.sector_distance(
            msg, min_deg_front, max_deg_front)
        self.distance_right = self.sector_distance(
            msg, min_deg_right, max_deg_right)

    def log_sensor_values(self):
        if self.first_scan_received:
            self.get_logger().info(
                f"Distance front: {self.distance_front}"
            )
            self.get_logger().info(
                f"Distance right: {self.distance_right}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = WallFollower()

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
