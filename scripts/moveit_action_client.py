import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from sensor_msgs.msg import JointState
from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

from copy import deepcopy

global_joint_states = None


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__("moveit_plan_execute_python")

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
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        joints = {}
        joints["Slider11"] = 0
        joints["spotarm_1_joint"] = 0
        joints["spotarm_2_joint"] = 3.1415
        joints["spotarm_3_joint"] = 3
        joints["spotarm_4_joint"] = 0
        joints["spotarm_5_joint"] = 0.2
        joints["spotarm_6_joint"] = 0

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

        self.create_subscription(JointState, "/joint_states", self.joint_states_cb, 1)
        self._action_client = ActionClient(self, MoveGroup, "/move_action")

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def send_goal(self):
        self.motion_plan_request.start_state.joint_state = self.joint_state

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
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(str(feedback_msg))


def main(args=None):
    rclpy.init(args=args)
    action_client = MoveGroupActionClient()
    # Get info from /joint_states
    while global_joint_states is None:
        rclpy.spin_once(action_client)
    action_client.send_goal()
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
