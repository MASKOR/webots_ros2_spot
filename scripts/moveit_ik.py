import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, TransformStamped, Quaternion

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import numpy as np

from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

from copy import deepcopy
import math


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_python")

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self, joint_names, joint_angles):
        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = (
            self.get_clock().now().to_msg()
        )
        self.motion_plan_request.workspace_parameters.header.frame_id = "base_link"
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True

        jc = JointConstraint()
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0

        joints = {name: angle for name, angle in zip(joint_names, joint_angles)}

        constraints = Constraints()
        for joint, angle in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = "move_group"
        self.motion_plan_request.group_name = "spot_arm"
        self.motion_plan_request.num_planning_attempts = 10
        self.motion_plan_request.allowed_planning_time = 5.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = False
        self.planning_options.look_around_attempts = 0
        self.planning_options.max_safe_execution_cost = 0.0
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 10
        self.planning_options.replan_delay = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

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
        # self.get_logger().info(str(future))
        self.goal_done = True
        pass

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))
        pass


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
        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)

        self.cli = self.create_client(GetPositionIK, "/compute_ik")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def send_request(self, group_name):
        self.req = GetPositionIK.Request()

        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.get_clock().now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = "P"

        self.req.ik_request.pose_stamped.pose.position.z = 0.05

        self.req.ik_request.timeout.sec = 10

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    moveit_ik = MoveitIKClient()

    moveit_ik.joint_state = None
    while moveit_ik.joint_state is None:
        rclpy.spin_once(moveit_ik)
    response = moveit_ik.send_request("spot")

    print(response.solution.joint_state.name[:8])
    target_angles = list(response.solution.joint_state.position)[:8]
    print(target_angles)

    if not len(target_angles):
        print("No IK solution found")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
