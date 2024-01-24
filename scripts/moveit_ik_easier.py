import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, TransformStamped, Quaternion

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import numpy as np

target_angles = None
global_joint_states = None
tf_base_link_P = None


def quaternion_from_euler(roll, pitch, yaw):
    ai = roll / 2.0
    aj = pitch / 2.0
    ak = yaw / 2.0
    ci = np.cos(ai)
    si = np.sin(ai)
    cj = np.cos(aj)
    sj = np.sin(aj)
    ck = np.cos(ak)
    sk = np.sin(ak)
    cc = ci * ck
    cs = ci * sk
    sc = si * ck
    ss = si * sk

    q = Quaternion()
    q.x = cj * sc - sj * cs
    q.y = cj * ss + sj * cc
    q.z = cj * cs - sj * sc
    q.w = cj * cc + sj * ss

    return q


class MoveitIKClient(Node):
    def __init__(self):
        super().__init__("moveit_ik_python")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(1.0, self.on_timer)
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def on_timer(self):
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform("base_link", "P", rclpy.time.Time())

            self.tf_base_link_P = Pose()

            self.tf_base_link_P.position.x = t.transform.translation.x
            self.tf_base_link_P.position.y = t.transform.translation.y
            self.tf_base_link_P.position.z = t.transform.translation.z
            self.tf_base_link_P.orientation.x = t.transform.rotation.x
            self.tf_base_link_P.orientation.y = t.transform.rotation.y
            self.tf_base_link_P.orientation.z = t.transform.rotation.z
            self.tf_base_link_P.orientation.w = t.transform.rotation.w

        except TransformException as ex:
            self.get_logger().info(f"Could not transform base_link to P: {ex}")
            return

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def send_request(self):
        request = GetPositionIK.Request()

        group_name = "spot_arm"
        request.ik_request.group_name = group_name
        request.ik_request.robot_state.joint_state = self.joint_state
        request.ik_request.avoid_collisions = True

        request.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        request.ik_request.pose_stamped.header.frame_id = "base_link"

        self.tf_base_link_P.position.z += 0.05

        request.ik_request.timeout.nanosec = 500000000

        joint_angles = []
        pitch_angles = [-0.1, 0.1, -0.2, 0.2, -0.3, 0.3, -0.4, 0.4]
        for pitch_angle in pitch_angles:
            if len(joint_angles) > 0:
                break

            transform = TransformStamped()
            transform.transform.rotation = quaternion_from_euler(0, pitch_angle, 0)
            pose = do_transform_pose(self.tf_base_link_P, transform)

            request.ik_request.pose_stamped.pose = pose

            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            # print(future.result().solution.joint_state.name[:8])
            joint_angles = list(future.result().solution.joint_state.position)[1:8]
            print(pitch_angle, joint_angles)

        return pitch_angle, joint_angles


def main():
    rclpy.init()

    moveit_ik = MoveitIKClient()
    moveit_ik.joint_state = None
    moveit_ik.tf_base_link_P = None
    while moveit_ik.joint_state is None or moveit_ik.tf_base_link_P is None:
        rclpy.spin_once(moveit_ik)
    pitch_angle, target_angles = moveit_ik.send_request()

    if target_angles is None:
        print("No IK solution found")
        moveit_ik.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
