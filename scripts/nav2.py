import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped


class NavigateToPoseActionClient(Node):
    def __init__(self):
        super().__init__("nav_to_pose_execute_python")

        self.pose = PoseStamped()
        self.pose.header.stamp = self.get_clock().now().to_msg()
        self.pose.header.frame_id = "map"

        self.pose.pose.position.x = 8.974
        self.pose.pose.position.y = 9.234
        self.pose.pose.position.z = 0.607

        self.pose.pose.orientation.x = 0.003
        self.pose.pose.orientation.y = 0.003
        self.pose.pose.orientation.z = 0.706
        self.pose.pose.orientation.w = 0.707

        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.pose

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.get_logger().info(str(future))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))


def main(args=None):
    rclpy.init(args=args)
    action_client = NavigateToPoseActionClient()
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
