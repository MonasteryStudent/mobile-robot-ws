#!/usr/bin/env python3

import math
import time
import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from mobile_robot_interfaces.action import NavigateToPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class NavigateToPositionServerNode(Node):

    def __init__(self):
        super().__init__("navigate_to_position_server")

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Allows action execution and odometry callbacks to run concurrently.
        self.callback_group = ReentrantCallbackGroup()

        self.odom_sub = self.create_subscription(
            Odometry,
            "odom",
            self.odom_callback,
            10,
            callback_group=self.callback_group
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "cmd_vel",
            10
        )
        
        self.navigate_to_position_server = ActionServer(
            self, 
            NavigateToPosition, 
            "navigate_to_position",
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            goal_callback=self.goal_callback,
            callback_group=self.callback_group
        )

        self.get_logger().info("Action server has been started.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Convert the quaternion orientation to the yaw angle used for 2D navigation.
        self.current_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT    

    def goal_callback(self, goal_request):
        self.get_logger().info("Received goal request.")
        return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):
        target_x = goal_handle.request.target_x
        target_y = goal_handle.request.target_y

        angle_tolerance = 0.05
        distance_tolerance = 0.1

        k_angular = 1.0
        k_linear = 0.8

        max_linear_velocity = 0.6

        cmd = Twist()

        while rclpy.ok():

            dx = target_x - self.current_x
            dy = target_y - self.current_y

            distance = math.sqrt(dx ** 2 + dy ** 2)

            feedback = NavigateToPosition.Feedback()
            feedback.distance_remaining = distance
            goal_handle.publish_feedback(feedback)

            # Calculate the desired heading from the current position to the goal.
            target_yaw = math.atan2(dy, dx)

            # Normalize the angular error to the range [-pi, pi].
            angle_error = target_yaw - self.current_yaw
            angle_error = math.atan2(
                math.sin(angle_error),
                math.cos(angle_error)
            )

            if goal_handle.is_cancel_requested:
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

                goal_handle.canceled()

                result = NavigateToPosition.Result()
                result.success = False
                result.final_x = self.current_x
                result.final_y = self.current_y

                return result

            if distance <= distance_tolerance:
                break

            cmd = Twist()

            # Proportional angular control keeps the robot oriented toward the goal.
            cmd.angular.z = k_angular * angle_error

            if abs(angle_error) > angle_tolerance:
                # Avoid driving forward while the robot is poorly aligned.
                cmd.linear.x = 0.0
            else:
                # Slow down as the robot approaches the target position.
                cmd.linear.x = min(
                    k_linear * distance,
                    max_linear_velocity
                )
          
            self.cmd_vel_pub.publish(cmd)

            # Limit the control loop to approximately 10 Hz.
            time.sleep(0.1)

        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

        goal_handle.succeed()

        result = NavigateToPosition.Result()
        result.success = True
        result.final_x = self.current_x
        result.final_y = self.current_y

        return result


def main(args=None):
    rclpy.init(args=args)

    node = NavigateToPositionServerNode()

    # Multiple threads are required so odometry continues to update while
    # execute_callback is running its control loop.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()