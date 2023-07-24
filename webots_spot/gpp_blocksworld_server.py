import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from webots_spot_msgs.srv import SpotMotion
from webots_spot_msgs.action import Stack

from sensor_msgs.msg import JointState
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from moveit_msgs.srv import GetPositionIK

from moveit_msgs.msg import MotionPlanRequest
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.msg import PlanningOptions
from moveit_msgs.action import MoveGroup

import numpy as np
from scipy.spatial.transform import Rotation as R
from copy import deepcopy
import time

target_angles = None
global_joint_states = None
tf_base_link_block = None
tf_base_link_location = None


class MoveGroupActionClient():
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        self._action_client = ActionClient(node, MoveGroup, '/move_action')

    def send_goal(self, target_angles):
        j = target_angles

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp = self.node.get_clock().now().to_msg()
        self.motion_plan_request.workspace_parameters.header.frame_id = 'base_link'
        self.motion_plan_request.workspace_parameters.min_corner.x = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.y = -1.0
        self.motion_plan_request.workspace_parameters.min_corner.z = -1.0
        self.motion_plan_request.workspace_parameters.max_corner.x = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.y = 1.0
        self.motion_plan_request.workspace_parameters.max_corner.z = 1.0
        self.motion_plan_request.start_state.is_diff = True

        joints = {}
        joints['spotarm_1_joint'] = j[0]
        joints['spotarm_2_joint'] = j[1]
        joints['spotarm_3_joint'] = j[2]
        joints['spotarm_4_joint'] = j[3]
        joints['Slider11'] = j[4]
        joints['spotarm_5_joint'] = j[5]
        joints['spotarm_6_joint'] = j[6]

        jc = JointConstraint()
        jc.tolerance_above = 0.0001
        jc.tolerance_below = 0.0001
        jc.weight = 1.0

        constraints = Constraints()
        for (joint, angle) in joints.items():
            jc.joint_name = joint
            jc.position = angle
            constraints.joint_constraints.append(deepcopy(jc))

        self.motion_plan_request.goal_constraints.append(constraints)

        self.motion_plan_request.pipeline_id = 'move_group'
        self.motion_plan_request.group_name = 'spot_arm'
        self.motion_plan_request.num_planning_attempts = 4
        self.motion_plan_request.allowed_planning_time = 4.0
        self.motion_plan_request.max_velocity_scaling_factor = 0.1
        self.motion_plan_request.max_acceleration_scaling_factor = 0.1
        self.motion_plan_request.max_cartesian_speed = 0.0

        self.planning_options = PlanningOptions()
        self.planning_options.plan_only = False
        self.planning_options.look_around = True
        self.planning_options.look_around_attempts = 5
        self.planning_options.max_safe_execution_cost = 0.
        self.planning_options.replan = True
        self.planning_options.replan_attempts = 4
        self.planning_options.replan_delay = 0.1

        goal_msg = MoveGroup.Goal()
        goal_msg.request = self.motion_plan_request
        goal_msg.planning_options = self.planning_options

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)


class MinimalClientAsync():
    def __init__(self, node):
        self.logger = node.get_logger()
        self.clock = node.get_clock()
        self.node = node

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)
        node.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)

        self.cli = node.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.logger.info('service not available, waiting again...')

    def on_timer(self, block, location):
        global tf_base_link_block, tf_base_link_location
        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                'base_link',
                block,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1))
            tf_base_link_block = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                  t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            self.tf_base_link_block = tf_base_link_block
            t = self.tf_buffer.lookup_transform(
                'base_link',
                location,
                rclpy.time.Time(),
                rclpy.duration.Duration(seconds=1))
            tf_base_link_location = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                     t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            self.tf_base_link_location = tf_base_link_location
        except TransformException as ex:
            self.logger.info(
                f'Could not transform base_link to can: {ex}')
            return

    def joint_states_cb(self, joint_state):
        global global_joint_states
        global_joint_states = joint_state
        self.joint_state = joint_state

    def moveit_ik(self):
        group_name = 'spot_arm'
        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp = self.clock.now().to_msg()
        self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

        if self.is_block:
            if self.is_above:
                xyz = [self.tf_base_link_block[0],self.tf_base_link_block[1],self.tf_base_link_block[2]]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.1
            else:
                xyz = [self.tf_base_link_block[0],self.tf_base_link_block[1],self.tf_base_link_block[2]]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.05
        else:
            if self.is_above:
                xyz = [self.tf_base_link_location[0],self.tf_base_link_location[1],self.tf_base_link_location[2]]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.14
            else:
                xyz = [self.tf_base_link_location[0],self.tf_base_link_location[1],self.tf_base_link_location[2]]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.09

        self.req.ik_request.timeout.sec = 5

        return self.req.ik_request

    def send_request(self, is_block, is_above):
        self.is_block = is_block
        self.is_above = is_above
        self.req = GetPositionIK.Request()
        self.req.ik_request = self.moveit_ik()

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()


def reset_variables():
    global global_joint_states, tf_base_link_block, tf_base_link_location, target_angles
    target_angles = None
    global_joint_states = None
    tf_base_link_block = None
    tf_base_link_location = None


def begin_stacking(stack_gh):
    global global_joint_states, tf_base_link_block, tf_base_link_location, target_angles

    reset_variables()
    
    block = stack_gh.request.block.upper()
    location = stack_gh.request.location.upper()
    moveit_node._logger.info(block + ':' + location)

    # Open gripper
    future = open_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(moveit_node, future)

    while (global_joint_states is None or tf_base_link_block is None or tf_base_link_location is None) and rclpy.ok():
        service_client.on_timer(block, location)
        rclpy.spin_once(moveit_node)

    # Move above block
    response = service_client.send_request(is_block=True, is_above=True)
    target_angles = list(response.solution.joint_state.position)[:7]    
    if not len(target_angles):
        moveit_node._logger.info('No IK solution found')
        return
    moveit_node._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles)
    
    # Move to block
    response = service_client.send_request(is_block=True, is_above=False)
    target_angles = list(response.solution.joint_state.position)[:7]    
    if not len(target_angles):
        moveit_node._logger.info('No IK solution found')
        return
    moveit_node._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles)
    time.sleep(4)

    # Close gripper
    future = close_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(moveit_node, future)

    reset_variables()

    # Move above location
    target_angles = None
    response = service_client.send_request(is_block=False, is_above=True)
    target_angles = list(response.solution.joint_state.position)[:7]
    if not len(target_angles):
        moveit_node._logger.info('No IK solution found')
        return
    moveit_node._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles)

    # Move to location
    target_angles = None
    response = service_client.send_request(is_block=False, is_above=False)
    target_angles = list(response.solution.joint_state.position)[:7]
    if not len(target_angles):
        moveit_node._logger.info('No IK solution found')
        return
    moveit_node._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles)
    time.sleep(4)

    # Move above location
    target_angles = None
    response = service_client.send_request(is_block=False, is_above=True)
    target_angles = list(response.solution.joint_state.position)[:7]
    if not len(target_angles):
        moveit_node._logger.info('No IK solution found')
        return
    moveit_node._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles)
    
    # Open gripper
    future = open_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(moveit_node, future)

    stack_gh.succeed()
    result = Stack.Result()
    result.stacked = True
    return result

rclpy.init()

moveit_node = Node('moveit_python')
stacker_node = Node('stacker_python')

from rclpy.executors import MultiThreadedExecutor

executor = MultiThreadedExecutor(num_threads=2)
executor.add_node(moveit_node)
executor.add_node(stacker_node)

service_client = MinimalClientAsync(moveit_node)
moveit_action_client = MoveGroupActionClient(moveit_node)
close_gripper_client = moveit_node.create_client(SpotMotion, '/Spot/close_gripper')
open_gripper_client = moveit_node.create_client(SpotMotion, '/Spot/open_gripper')

def main():
    action_server = ActionServer(
                stacker_node,
                Stack,
                'stack',
                begin_stacking)

    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
