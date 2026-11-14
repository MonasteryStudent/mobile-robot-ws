#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.first_scan_received = False
        self.hit = None

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

        self.velocity_publisher = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def follow_wall(self):
        """Publish a velocity command based on the front and right distances."""
        command = Twist()

        target_distance = 0.80
        distance_tolerance = 0.05
        front_turn_distance = 2.0

        straight_speed = 0.10
        turn_speed = 0.06
        turn_rate = 0.10

        wall_infront = self.distance_front is not None
        right_wall = self.distance_right is not None

        if not right_wall:
            # Stop for now.
            pass
        elif wall_infront and (self.distance_front < front_turn_distance):
            # Turn left.
            command.linear.x = turn_speed
            command.angular.z = turn_rate          
        elif self.distance_right < target_distance - distance_tolerance:
            # Turn left.
            command.linear.x = turn_speed
            command.angular.z = turn_rate
        elif self.distance_right > target_distance + distance_tolerance:
            # Turn right.
            command.linear.x = turn_speed
            command.angular.z = -turn_rate
        else:
            command.linear.x = straight_speed

        self.velocity_publisher.publish(command)

    def closest_hit(self, msg):
        """Return the nearest valid hit as (distance, angle in radians), or None."""
        min_distance = float('inf')

        for i, distance in enumerate(msg.ranges):
            if (
                msg.range_min <= distance <= msg.range_max
                and distance < min_distance
            ):
                    min_distance = distance
                    min_distance_index = i

        if min_distance == float('inf'):
            return None

        min_distance_angle = msg.angle_min + min_distance_index * msg.angle_increment

        return min_distance, min_distance_angle

    def turn_toward_wall(self, angle, angle_tolerance, command):
        """Set the angular velocity to turn toward the wall if needed."""
        turn_rate = 0.10

        if angle > angle_tolerance:
            command.angular.z = turn_rate
        elif angle < -angle_tolerance:
            command.angular.z = -turn_rate

    def drive_to_wall(self, distance, command):
        """Set the forward or reverse velocity if outside the target distance band."""
        straight_speed = 0.10
        target_distance = 0.80
        distance_tolerance = 0.03

        if distance > target_distance + distance_tolerance:
            command.linear.x = straight_speed
        elif distance < target_distance - distance_tolerance:
            command.linear.x = -straight_speed

    def sector_distance(self, msg, min_deg, max_deg):
        """Return the shortest valid distance in the given degree range, or None."""
        min_angle = math.radians(min_deg)
        max_angle = math.radians(max_deg)

        min_distance = float('inf')

        for i, distance in enumerate(msg.ranges):
            curr_angle = msg.angle_min + i * msg.angle_increment

            if (
                min_angle <= curr_angle <= max_angle
                and msg.range_min <= distance <= msg.range_max
                and distance < min_distance
            ):
                    min_distance = distance

        if min_distance == float('inf'):
            return None

        return min_distance

    def scan_callback(self, msg):
        """Process the scan and publish one velocity command."""
        if not self.first_scan_received:
            num_ranges = len(msg.ranges)
            self.get_logger().info(
                f"First scan received: {num_ranges} ranges"
            )
            self.first_scan_received = True

        self.hit = self.closest_hit(msg)

        command = Twist()

        if self.hit is not None:
            angle_tolerance = math.radians(3.0)
            distance, angle = self.hit

            if abs(angle) > angle_tolerance:
                self.turn_toward_wall(angle, angle_tolerance, command)
            else:
                self.drive_to_wall(distance, command)
            
        self.velocity_publisher.publish(command)

        # Re-enable sector-based wall following after parallel alignment.

        # min_deg_front = -5.0
        # max_deg_front = 5.0
        # min_deg_right = -95.0
        # max_deg_right = -85.0

        # self.distance_front = self.sector_distance(
        #     msg, min_deg_front, max_deg_front)
        # self.distance_right = self.sector_distance(
        #     msg, min_deg_right, max_deg_right)
        
        # self.follow_wall()

    def log_sensor_values(self):
        """Log the distance and bearing of the latest valid hit."""
        if self.first_scan_received:
            # self.get_logger().info(
            #     f"Distance front: {self.distance_front}"
            # )
            # self.get_logger().info(
            #     f"Distance right: {self.distance_right}"
            #     )
            if self.hit is not None:
                distance, angle = self.hit
                self.get_logger().info(
                    f"Closest hit: {distance:.3f} m at {math.degrees(angle):.1f}° "
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
