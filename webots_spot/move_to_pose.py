import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK

from moveit_msgs.msg import (
    MotionPlanRequest,
    JointConstraint,
    Constraints,
    PlanningOptions,
)
from moveit_msgs.action import MoveGroup

import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
import time

target_angles = None
global_joint_states = None
tf_base_link_pipe = None


def skip_extra_rotation(j):
    k = []
    for f in j:
        if f > np.pi:
            f -= 2 * np.pi
        if f < -np.pi:
            f += 2 * np.pi
        k.append(f)
    return k


class MoveGroupActionClient:
    def __init__(self, node, target_angles):
        self.logger = node.get_logger()

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = (
            node.get_clock().now().to_msg()
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
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        j = skip_extra_rotation(target_angles)

        joints = {}
        joints["joint_1"] = j[0]
        joints["joint_2"] = j[1]
        joints["joint_3"] = j[2]
        joints["joint_4"] = j[3]
        joints["joint_5"] = j[4]
        # joints['wrist_3_joint'] = j[5]
        joints["joint_6"] = 0.0  # gripper always horizontal

        constraints = Constraints()
        for joint, angle in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = "move_group"
        self.motion_plan_request.group_name = "manipulator"
        self.motion_plan_request.num_planning_attempts = 4
        self.motion_plan_request.allowed_planning_time = 4.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = True
        self.planning_options.look_around_attempts = 5
        self.planning_options.max_safe_execution_cost = 0.0
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 4
        self.planning_options.replan_delay = 0.1

        self._action_client = ActionClient(node, MoveGroup, "/move_action")

    def send_goal(self):
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
            self.logger.info("Goal rejected :(")
            return

        self.logger.info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.logger.info(str(future))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.logger.info(str(feedback_msg))


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

        self.arm_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/kinova_joint_trajectory_controller/follow_joint_trajectory",
        )
        self.arm_client.wait_for_server()
        self.arm_point = JointTrajectoryPoint()
        self.arm_joint_names = [
            "joint_1",
            "joint_2",
            "joint_3",
            "joint_4",
            "joint_5",
            "joint_6",
        ]

        self.sequence = [
            "linear_board",
            # "front_right",
            # "front_left",
            # "angled_right",
            # "angled_left",
        ]

    def on_timer(self):
        global tf_base_link_pipe

        try:
            t = self.tf_buffer.lookup_transform(
                "base_link", "linear_board", rclpy.time.Time()
            )
            tf_base_link_pipe = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]

            self.tf_base_link_pipe = tf_base_link_pipe
            self.logger.info(
                f"base_link to linear_board transform:\n"
                f"x: {tf_base_link_pipe[0]:.3f}\n"
                f"y: {tf_base_link_pipe[1]:.3f}\n"
                f"z: {tf_base_link_pipe[2]:.3f}\n"
                f"rx: {tf_base_link_pipe[3]:.3f}\n"
                f"ry: {tf_base_link_pipe[4]:.3f}\n"
                f"rz: {tf_base_link_pipe[5]:.3f}\n"
                f"rw: {tf_base_link_pipe[6]:.3f}"
            )
        except TransformException as ex:
            self.logger.info(f"Could not transform base_link to linear_board: {ex}")
            return

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def moveit_ik(self, approach):
        group_name = "manipulator"
        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.clock.now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = "base_link"

        self.req.ik_request.pose_stamped.pose.position.x = tf_base_link_pipe[0]
        self.req.ik_request.pose_stamped.pose.position.y = tf_base_link_pipe[1]
        self.req.ik_request.pose_stamped.pose.position.z = tf_base_link_pipe[2]
        self.req.ik_request.pose_stamped.pose.orientation.x = tf_base_link_pipe[3]
        self.req.ik_request.pose_stamped.pose.orientation.y = tf_base_link_pipe[4]
        self.req.ik_request.pose_stamped.pose.orientation.z = -tf_base_link_pipe[5]
        self.req.ik_request.pose_stamped.pose.orientation.w = -tf_base_link_pipe[6]

        self.req.ik_request.timeout.sec = 5

        return self.req.ik_request

    def send_request(self, approach):
        self.req = GetPositionIK.Request()
        self.req.ik_request = self.moveit_ik(approach)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()

    def send_trajectory(self, target_angles):
        arm_goal_msg = FollowJointTrajectory.Goal()
        arm_goal_msg.trajectory.joint_names = self.arm_joint_names
        arm_point = JointTrajectoryPoint()
        arm_point.positions = target_angles
        arm_point.velocities = [0.0] * 6
        arm_point.time_from_start.sec = 3
        arm_goal_msg.trajectory.points.append(arm_point)
        send_goal_future = self.arm_client.send_goal_async(arm_goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.logger.info("Trajectory goal rejected")
            return
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future)
        self.logger.info("Trajectory completed")


def main():
    global tf_base_link_pipe
    rclpy.init()
    node = Node("moveit_ik_python")
    minimal_client = MinimalClientAsync(node)

    # Wait for initial joint states and at least one transform
    while global_joint_states is None or tf_base_link_pipe is None:
        rclpy.spin_once(node)

    # 1. Collect and store all transforms for the sequence
    transforms = {}
    for frame in minimal_client.sequence:
        try:
            t = minimal_client.tf_buffer.lookup_transform(
                "base_link",
                frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.005),
            )
            transforms[frame] = [
                t.transform.translation.x,
                t.transform.translation.y,
                t.transform.translation.z,
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w,
            ]
            minimal_client.logger.info(
                f"Stored base_link to {frame} transform:\n"
                f"x: {transforms[frame][0]:.3f}\n"
                f"y: {transforms[frame][1]:.3f}\n"
                f"z: {transforms[frame][2]:.3f}\n"
                f"rx: {transforms[frame][3]:.3f}\n"
                f"ry: {transforms[frame][4]:.3f}\n"
                f"rz: {transforms[frame][5]:.3f}\n"
                f"rw: {transforms[frame][6]:.3f}"
            )
        except TransformException as ex:
            minimal_client.logger.info(
                f"Could not transform base_link to {frame}: {ex}"
            )
            transforms[frame] = None  # Mark as unavailable

    # 2. Compute IK and send trajectories using stored transforms
    for frame in minimal_client.sequence:
        tf_data = transforms.get(frame)
        if tf_data is None:
            minimal_client.logger.info(f"Skipping {frame}: no stored transform")
            continue

        # Set the global tf_base_link_pipe to the stored transform for this frame
        tf_base_link_pipe = tf_data
        minimal_client.tf_base_link_pipe = tf_data

        response = minimal_client.send_request(approach=1)
        target_angles = list(response.solution.joint_state.position)[:6]
        print(f"IK solution for {frame}: {target_angles}")
        if not len(target_angles):
            minimal_client.logger.info(f"No IK solution for {frame}")
            continue

        minimal_client.send_trajectory(target_angles)
        minimal_client.logger.info(f"Sent trajectory for {frame}")

        # Wait 3 seconds before next move
        time.sleep(3)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
