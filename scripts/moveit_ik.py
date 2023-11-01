import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK

import numpy as np

target_angles = None
global_joint_states = None
tf_base_link_P = None


class MinimalClientAsync:
    def __init__(self, node):
        self.logger = node.get_logger()
        self.clock = node.get_clock()
        self.node = node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        self.timer = node.create_timer(1.0, self.on_timer)
        node.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = node.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info("service not available, waiting again...")

    def on_timer(self):
        global tf_base_link_P
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "P", rclpy.time.Time(), rclpy.duration.Duration(seconds=1)
            )

            tf_base_link_P = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]
            self.tf_base_link_P = tf_base_link_P
        except TransformException as ex:
            self.logger.info(f"Could not transform base_link to P: {ex}")
            return

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def moveit_ik(self):
        group_name = "spot_arm"
        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.clock.now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = "base_link"

        self.req.ik_request.pose_stamped.pose.position.x = self.tf_base_link_P[0]
        self.req.ik_request.pose_stamped.pose.position.y = self.tf_base_link_P[1]
        self.req.ik_request.pose_stamped.pose.position.z = self.tf_base_link_P[2] + 0.05

        self.req.ik_request.pose_stamped.pose.orientation.x = self.tf_base_link_P[3]
        self.req.ik_request.pose_stamped.pose.orientation.y = self.tf_base_link_P[4]
        self.req.ik_request.pose_stamped.pose.orientation.z = self.tf_base_link_P[5]
        self.req.ik_request.pose_stamped.pose.orientation.w = self.tf_base_link_P[6]

        self.req.ik_request.timeout.sec = 5

        return self.req.ik_request

    def send_request(self):
        self.req = GetPositionIK.Request()
        self.req.ik_request = self.moveit_ik()

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()


def main():
    rclpy.init()
    node = Node("moveit_ik_python")

    minimal_client = MinimalClientAsync(node)
    while global_joint_states is None or tf_base_link_P is None:
        rclpy.spin_once(node)
    response = minimal_client.send_request()
    print(response.solution.joint_state.name[:7])
    target_angles = list(response.solution.joint_state.position)[:7]

    if not len(target_angles):
        print("No IK solution found")
        node.destroy_node()
        rclpy.shutdown()
        return
    print([round(np.rad2deg(ta)) for ta in target_angles])


if __name__ == "__main__":
    main()
