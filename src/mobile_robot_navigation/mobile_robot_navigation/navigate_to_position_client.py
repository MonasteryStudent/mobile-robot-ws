#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from mobile_robot_interfaces.action import NavigateToPosition


class NavigateToPositionClientNode(Node):

    def __init__(self):
        super().__init__("navigate_to_position_client")

        self.action_client = ActionClient(
            self,
            NavigateToPosition,
            "navigate_to_position"
        )

    def send_goal(self):
        goal = NavigateToPosition.Goal()
        goal.target_x = 5.0
        goal.target_y = 5.0

        self.action_client.wait_for_server()

        self.get_logger().info("Sending goal.")

        self.goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        self.goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.get_logger().info("Goal accepted.")

        # Cancel after 3 seconds
        self.cancel_timer = self.create_timer(
            3.0,
            self.cancel_goal
        )

    def cancel_goal(self):
        self.cancel_timer.cancel()

        self.get_logger().info("Canceling goal.")

        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

    def cancel_response_callback(self, future):
        response = future.result()

        if len(response.goals_canceling) > 0:
            self.get_logger().info("Goal cancellation accepted.")
        else:
            self.get_logger().info("Goal cancellation rejected.")

    def feedback_callback(self, feedback_msg):
        distance = feedback_msg.feedback.distance_remaining

        self.get_logger().info(
            f"Distance remaining: {distance:.2f}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = NavigateToPositionClientNode()
    node.send_goal()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()