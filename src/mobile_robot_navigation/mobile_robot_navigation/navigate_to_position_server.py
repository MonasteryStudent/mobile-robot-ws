#!/usr/bin/env python3

import math

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

from mobile_robot_interfaces.action import NavigateToPosition
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class NavigateToPositionServerNode(Node):

    def __init__(self):
        super().__init__("navigate_to_position_server")

        self.target_x = 0.0
        self.target_y = 0.0

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        self.angle_tolerance = 0.05
        self.distance_tolerance = 0.1

        self.k_angular = 1.0
        self.k_linear = 0.8

        self.max_linear_velocity = 0.6

        # Shared navigation state used across the action, timer,
        # and odometry callbacks.
        self.state = "IDLE"
        self.goal_result = None
        self.active_goal_handle = None

        # Allows callbacks in this group to be processed while the asynchronous
        # execute callback is suspended waiting for the navigation result.
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

        # Runs one navigation control step every 100 ms instead of using
        # a blocking control loop inside the action execute callback.
        self.control_timer = self.create_timer(
            0.1,
            self.control_callback,
            callback_group=self.callback_group
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

    def control_callback(self):
        # No navigation goal is currently active.
        if self.state == "IDLE":
            return

        if self.active_goal_handle.is_cancel_requested:
            # Stop the robot before completing the action as canceled.
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

            self.active_goal_handle.canceled()
            self.goal_result = "CANCELED"
            self.state = "IDLE"

            # Resume the suspended execute callback so it can return the result.
            self.goal_completion_future.set_result(True)
            return

        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y

        distance = math.sqrt(dx ** 2 + dy ** 2)

        feedback = NavigateToPosition.Feedback()
        feedback.distance_remaining = distance
        self.active_goal_handle.publish_feedback(feedback)

        # Calculate the desired heading from the current position to the goal.
        target_yaw = math.atan2(dy, dx)

        angle_error = target_yaw - self.current_yaw

        # Normalize the angular error to [-pi, pi] so the robot takes
        # the shortest rotational direction toward the target.
        angle_error = math.atan2(
            math.sin(angle_error),
            math.cos(angle_error)
        )

        cmd = Twist()

        if self.state == "ROTATING":
            if abs(angle_error) > self.angle_tolerance:
                # Proportional angular control keeps the robot oriented 
                # toward the goal.
                cmd.angular.z = self.k_angular * angle_error
            else:
                cmd.angular.z = 0.0

                # Continue with forward motion on the next timer cycle.
                self.state = "DRIVING"

            self.cmd_vel_pub.publish(cmd)

        elif self.state == "DRIVING":
            if distance <= self.distance_tolerance:
                # Stop the robot once the target position is reached.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

                self.active_goal_handle.succeed()
                self.goal_result = "SUCCEEDED"
                self.state = "IDLE"

                # Signal the execute callback that navigation has finished.
                self.goal_completion_future.set_result(True)
                return

            if abs(angle_error) > self.angle_tolerance:
                # Stop forward motion and switch back to the rotation state.
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

                self.state = "ROTATING"
                return

            # Apply small heading corrections while driving toward the goal.
            cmd.angular.z = self.k_angular * angle_error

            # Reduce linear velocity as the robot approaches the target.
            cmd.linear.x = min(
                self.k_linear * distance,
                self.max_linear_velocity
            )

            self.cmd_vel_pub.publish(cmd)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        # Convert the quaternion orientation to the yaw angle
        # used for 2D navigation.
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

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        self.goal_result = None

        self.target_x = goal_handle.request.target_x
        self.target_y = goal_handle.request.target_y
        self.active_goal_handle = goal_handle

        # The timer-based state machine performs the actual navigation.
        # This future is completed by the control callback once the goal
        # succeeds or is canceled.
        self.goal_completion_future = Future()

        self.state = "ROTATING"

        # Suspend this coroutine without blocking the executor.
        await self.goal_completion_future

        result = NavigateToPosition.Result()

        result.success = self.goal_result == "SUCCEEDED"
        result.final_x = self.current_x
        result.final_y = self.current_y

        return result


def main(args=None):
    rclpy.init(args=args)

    node = NavigateToPositionServerNode()

    # A single-threaded executor is sufficient because the execute callback
    # waits asynchronously instead of blocking in a control loop.
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()   


if __name__ == "__main__":
    main()