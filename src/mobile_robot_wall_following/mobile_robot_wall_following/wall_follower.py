#!/usr/bin/env python3

import math

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class WallFollower(Node):
    def __init__(self):
        super().__init__("wall_follower")

        self.first_scan_received = False
        self.hit = None

        self.previous_position = None
        self.traveled_distance = 0.0

        self.correction_start_distance = None
        self.correction_start_time = None
        self.left_parallel_band = False
        self.experiment_finished = False        

        self.odom_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

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

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y

        if self.previous_position is None:
            self.previous_position = (x, y)
            return
        
        self.traveled_distance += math.hypot(
            x - self.previous_position[0], 
            y - self.previous_position[1]
        )

        self.previous_position = (x, y)

    def follow_wall(self):
        """Return a velocity command from the latest scan distances."""
        command = Twist()

        if self.distance_right is None or self.hit is None:
            return command

        target_distance = 0.80
        distance_tolerance = 0.05
        front_turn_distance = 2.0

        straight_speed = 0.10
        turn_speed = 0.06
        turn_rate = 0.10

        wall_infront = self.distance_front is not None

        _, angle = self.hit

        target_angle = math.radians(-90)
        angle_tolerance = math.radians(10)

        if wall_infront and (self.distance_front < front_turn_distance):
            # Turn left.
            command.linear.x = turn_speed
            command.angular.z = turn_rate          
        elif self.distance_right < target_distance - distance_tolerance:
            # Turn left.
            if target_angle - angle_tolerance < angle:
                command.angular.z = turn_rate / 3.0
            else:
                command.angular.z = turn_rate
            command.linear.x = turn_speed
        elif self.distance_right > target_distance + distance_tolerance:
            # Turn right.
            if target_angle + angle_tolerance < angle:
                command.angular.z = -turn_rate / 3.0
            else:
                command.angular.z = -turn_rate
            command.linear.x = turn_speed
        else:
            command.linear.x = straight_speed

        return command

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

    def update_experiment(self, command, stamp):
        """Measure the distance and time from the first right turn to realignment."""
        if self.hit is None or self.previous_position is None:
            return

        scan_time = stamp.sec + stamp.nanosec / 1_000_000_000
        wall_distance, angle = self.hit
        is_parallel = abs(angle - math.radians(-90)) <= math.radians(3)

        if self.correction_start_distance is None:
            if command.angular.z < 0.0:
                self.correction_start_distance = self.traveled_distance
                self.correction_start_time = scan_time
            return

        if not is_parallel:
            self.left_parallel_band = True
            return

        if self.left_parallel_band:
            correction_distance = (
                self.traveled_distance - self.correction_start_distance
            )
            elapsed_seconds = scan_time - self.correction_start_time

            self.get_logger().info(
                f"Parallel again after {correction_distance:.3f} m "
                f"and {elapsed_seconds:.1f} s; "
                f"wall distance: {wall_distance:.3f} m; "
                f"hit angle: {math.degrees(angle):.1f}°"
            )
            self.experiment_finished = True

    def scan_callback(self, msg):
        """Process the scan and publish one velocity command."""
        if not self.first_scan_received:
            num_ranges = len(msg.ranges)
            self.get_logger().info(
                f"First scan received: {num_ranges} ranges"
            )
            self.first_scan_received = True

        self.hit = self.closest_hit(msg)

        min_deg_front = -5.0
        max_deg_front = 5.0
        min_deg_right = -95.0
        max_deg_right = -85.0

        self.distance_front = self.sector_distance(
            msg, min_deg_front, max_deg_front)
        self.distance_right = self.sector_distance(
            msg, min_deg_right, max_deg_right)

        command = Twist()

        if self.previous_position is not None and not self.experiment_finished:
            command = self.follow_wall()
            self.update_experiment(command, msg.header.stamp)

            if self.experiment_finished:
                command = Twist()
        
        self.velocity_publisher.publish(command)

    def log_sensor_values(self):
        """Log the latest front and right distances."""
        if not self.first_scan_received or self.experiment_finished:
            return

        front = self.distance_front
        right = self.distance_right
        front_text = "no hit" if front is None else f"{front:.2f} m"
        right_text = "no hit" if right is None else f"{right:.2f} m"
        
        self.get_logger().info(
            f"Front: {front_text}, right: {right_text}"
        )

        if self.hit is not None:
            distance, angle = self.hit
            self.get_logger().info(
                f"Hit distance: {distance:.2f} m, hit angle: {math.degrees(angle):.2f}°"
            )

        self.get_logger().info(
            f"Traveled distance: {self.traveled_distance:.2f} m"
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
