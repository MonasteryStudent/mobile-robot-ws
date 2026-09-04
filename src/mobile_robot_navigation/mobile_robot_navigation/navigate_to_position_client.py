#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from std_srvs.srv import Trigger
from action_msgs.msg import GoalStatus
from mobile_robot_interfaces.action import NavigateToPosition


class NavigateToPositionClientNode(Node):

    def __init__(self):
        super().__init__("navigate_to_position_client")

        # Tracks whether a currently accepted goal is still active.
        self.goal_active = False

        self.declare_parameter("target_x", 1.0)
        self.declare_parameter("target_y", 1.0)
        self.declare_parameter("show_feedback", False)

        self.action_client = ActionClient(
            self,
            NavigateToPosition,
            "navigate_to_position"
        )

        self.cancel_service = self.create_service(
            Trigger,
            "cancel_navigation",
            self.cancel_service_callback
        )

    def cancel_service_callback(self, request, response):
        if not self.goal_active:
            response.success = False
            response.message = "No active goal."
            return response

        # Send the cancel request asynchronously and process the server response
        # once it becomes available.
        cancel_future = self.goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(
            self.cancel_response_callback
        )

        response.success = True
        response.message = "Cancel request sent."
        return response
    
    def cancel_response_callback(self, future):
        response = future.result()

        # goals_canceling contains the goals whose cancellation was accepted
        # for this specific cancel request.
        if len(response.goals_canceling) > 0:
            self.get_logger().info("Goal cancellation accepted.")
        else:
            self.get_logger().info("Goal cancellation rejected.")

    def send_goal(self):
        goal = NavigateToPosition.Goal()
        goal.target_x = self.get_parameter("target_x").value
        goal.target_y = self.get_parameter("target_y").value

        self.action_client.wait_for_server()

        self.get_logger().info("Sending goal.")

        # Register the feedback callback while sending the goal asynchronously.
        self.goal_future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        # Process the server's accept/reject response once it arrives.
        self.goal_future.add_done_callback(
            self.goal_response_callback
        )

    def goal_response_callback(self, future):
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info("Goal rejected.")
            return

        self.goal_active = True
        self.get_logger().info("Goal accepted.")

        # Request the final result without blocking the node.
        self.result_future = self.goal_handle.get_result_async()
        self.result_future.add_done_callback(
            self.result_callback
        )

    def result_callback(self, future):
        self.goal_active = False

        result_response = future.result()

        # The result response contains both the ROS action status and the
        # application-specific NavigateToPosition result.
        status = result_response.status
        result = result_response.result

        if status == GoalStatus.STATUS_SUCCEEDED:
            status_text = "SUCCEEDED"
        elif status == GoalStatus.STATUS_CANCELED:
            status_text = "CANCELED"
        elif status == GoalStatus.STATUS_ABORTED:
            status_text = "ABORTED"
        else:
            status_text = str(status)

        self.get_logger().info(
            f"Status: {status_text}, "
            f"success={result.success}, "
            f"final_x={result.final_x:.2f}, "
            f"final_y={result.final_y:.2f}"
        )

    def feedback_callback(self, feedback_msg):
        # Feedback is always received, but logging can be disabled by parameter.
        if not self.get_parameter("show_feedback").value:
            return        
    
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