import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.clock import Clock

from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

from copy import deepcopy

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import Pose, TransformStamped, Quaternion

from webots_spot_msgs.msg import GaitInput

from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory

from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose

import numpy as np

import time

TIMEOUT_PERIOD = 5  # seconds


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


class GripperActionClient(Node):
    def __init__(self):
        super().__init__("GripperAC")
        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory",
        )

    def send_goal(self, gripper_action):
        self.goal_done = False

        goal_msg = FollowJointTrajectory.Goal()

        # Set your joint trajectory goal here
        goal_msg.trajectory.joint_names = [
            "gripper_left_finger_joint",
            "gripper_right_finger_joint",
        ]
        point = JointTrajectoryPoint()
        if gripper_action == "close":
            point.positions = [0.02, 0.02]
        elif gripper_action == "open":
            point.positions = [0.045, 0.045]  # Replace with desired joint positions
        point.time_from_start.sec = 1  # Duration in seconds

        goal_msg.trajectory.points.append(point)

        self.action_client.wait_for_server()
        self._send_goal_future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            self.goal_done = True
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # self.get_logger().info(str(future))
        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        # self.get_logger().info(str(feedback_msg))
        pass


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
        pitch_angles = [0.0, -0.1, 0.1, -0.2, 0.2, -0.3, 0.3]
        for pitch_angle in pitch_angles:
            transform = TransformStamped()
            transform.transform.rotation = quaternion_from_euler(0, pitch_angle, 0)
            pose = do_transform_pose(self.tf_base_link_P, transform)

            request.ik_request.pose_stamped.pose = pose

            future = self.cli.call_async(request)
            rclpy.spin_until_future_complete(self, future)

            # print(future.result().solution.joint_state.name[:8])
            joint_angles = list(future.result().solution.joint_state.position)[1:8]
            print(pitch_angle, joint_angles)
            if len(joint_angles) > 0:
                break

        if pitch_angle > 0:
            pitch_angle += 0.06
        if pitch_angle < 0:
            pitch_angle -= 0.06
        return pitch_angle, joint_angles


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_python")

        self._action_client = ActionClient(self, MoveGroup, "/move_action")

    def send_goal(self, joint_angles):
        self.goal_done = False

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
        jc.tolerance_above = 0.001
        jc.tolerance_below = 0.001
        jc.weight = 1.0

        joints = {}
        joints["spotarm_1_joint"] = joint_angles[0]
        joints["spotarm_2_joint"] = joint_angles[1]
        joints["spotarm_3_joint"] = joint_angles[2]
        joints["spotarm_4_joint"] = joint_angles[3]
        joints["Slider11"] = joint_angles[4]
        joints["spotarm_5_joint"] = joint_angles[5]
        joints["spotarm_6_joint"] = joint_angles[6]

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
        self.motion_plan_request.max_velocity_scaling_factor = 0.4
        self.motion_plan_request.max_acceleration_scaling_factor = 0.4
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
        self.get_logger().info(str(future))
        self.goal_done = True

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))


class SpotPitch(Node):
    def __init__(self):
        super().__init__("spot_pitch_control")

        self.gait_pub = self.create_publisher(GaitInput, "/Spot/inverse_gait_input", 1)

    def change_pitch(self, pitch_angle):
        msg = GaitInput()
        msg.penetration_depth = 0.003
        msg.swing_period = 0.2
        msg.pitch = pitch_angle

        self.gait_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    moveit_ik = MoveitIKClient()
    action_client = MoveGroupActionClient()
    spot_pitch = SpotPitch()
    gripper = GripperActionClient()

    pitch_angle = 0.0
    spot_pitch.change_pitch(pitch_angle)
    time.sleep(0.5)

    moveit_ik.joint_state = None
    moveit_ik.tf_base_link_P = None
    moveit_ik.tf_buffer.clear()
    while moveit_ik.joint_state is None or moveit_ik.tf_base_link_P is None:
        rclpy.spin_once(moveit_ik)
    pitch_angle, target_angles = moveit_ik.send_request()

    if len(target_angles) == 0:
        print("No IK solution found")
        rclpy.shutdown()
        return

    timeout = (Clock().now().nanoseconds / 1e9) + TIMEOUT_PERIOD

    if pitch_angle != 0:
        target_angles = []

    while timeout > (Clock().now().nanoseconds / 1e9):
        spot_pitch.change_pitch(pitch_angle)
        time.sleep(0.5)

        while timeout > (Clock().now().nanoseconds / 1e9):
            moveit_ik.joint_state = None
            moveit_ik.tf_base_link_P = None
            moveit_ik.tf_buffer.clear()
            while moveit_ik.joint_state is None or moveit_ik.tf_base_link_P is None:
                rclpy.spin_once(moveit_ik)
            pitch_angle, target_angles = moveit_ik.send_request()

            if pitch_angle == 0.0 and len(target_angles) > 0:
                break

        if pitch_angle == 0.0:
            break

    if pitch_angle != 0 or len(target_angles) == 0:
        spot_pitch.change_pitch(pitch_angle)
        time.sleep(0.5)
        rclpy.shutdown()
        return

    action_client.send_goal(target_angles)
    while not action_client.goal_done:
        rclpy.spin_once(action_client)

    gripper.send_goal("close")
    while not gripper.goal_done:
        rclpy.spin_once(gripper)

    action_client.send_goal(
        [0.0, np.radians(179), np.radians(170), 0.0, 0.0, np.radians(11), 0.0]
    )
    while not action_client.goal_done:
        rclpy.spin_once(action_client)
    spot_pitch.change_pitch(0.0)


if __name__ == "__main__":
    main()
