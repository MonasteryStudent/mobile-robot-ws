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

from collections import deque


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

        # Shared navigation state used across callbacks.
        self.state = "IDLE"

        # The active goal is executed by the state machine, while additional
        # accepted goals wait in FIFO order.
        self.active_goal_handle = None
        self.active_goal_future = None
        self.goal_queue = deque()

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

    # Process cancellation requests for goals that are still waiting in the queue.
    def process_queued_cancellations(self):
        remaining_goals = deque()

        while self.goal_queue:
            goal_handle, completion_future = self.goal_queue.popleft()

            if goal_handle.is_cancel_requested:
                result = NavigateToPosition.Result()
                result.success = False
                result.final_x = self.current_x
                result.final_y = self.current_y

                goal_handle.canceled()
                completion_future.set_result(result)

                self.get_logger().info("Canceled queued goal.")
            else:
                remaining_goals.append((goal_handle, completion_future))

        self.goal_queue = remaining_goals

    def control_callback(self):
        self.process_queued_cancellations()

        # No navigation goal is currently active.
        if self.state == "IDLE":
            return

        if self.active_goal_handle.is_cancel_requested:
            # Stop the robot before completing the action as canceled.
            cmd = Twist()
            self.cmd_vel_pub.publish(cmd)

            result = NavigateToPosition.Result()
            result.success = False
            result.final_x = self.current_x
            result.final_y = self.current_y

            self.active_goal_handle.canceled()
            self.active_goal_future.set_result(result)

            self.active_goal_handle = None
            self.active_goal_future = None
            self.start_next_goal()
            
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
                cmd.linear.x = 0.0
                cmd.angular.z = 0.0
                self.cmd_vel_pub.publish(cmd)

                result = NavigateToPosition.Result()
                result.success = True
                result.final_x = self.current_x
                result.final_y = self.current_y

                self.active_goal_handle.succeed()
                self.active_goal_future.set_result(result)

                self.active_goal_handle = None
                self.active_goal_future = None
                self.start_next_goal()

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

    def start_next_goal(self):
        if not self.goal_queue:
            self.active_goal_handle = None
            self.active_goal_future = None
            self.state = "IDLE"
            return

        self.active_goal_handle, self.active_goal_future = self.goal_queue.popleft()

        self.target_x = self.active_goal_handle.request.target_x
        self.target_y = self.active_goal_handle.request.target_y

        self.state = "ROTATING"

        self.get_logger().info(
            f"Starting next goal: "
            f"x={self.target_x:.2f}, y={self.target_y:.2f}"
        )

    async def execute_callback(self, goal_handle: ServerGoalHandle):
        completion_future = Future()

        self.goal_queue.append((goal_handle, completion_future))

        if self.active_goal_handle is None:
            self.start_next_goal()

        result = await completion_future

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