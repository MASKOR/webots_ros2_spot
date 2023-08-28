import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.action import ActionClient

from webots_spot_msgs.srv import SpotMotion
from webots_spot_msgs.action import Stack

from rosgraph_msgs.msg import Clock
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

import time
import numpy as np
from copy import deepcopy


class MoveGroupActionClient(Node):
    def __init__(self):
        super().__init__('moveit_plan_execute_python')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')

    def send_goal(self, target_angles, sec, nanosec):
        self.goal_done = False

        j = target_angles

        self.motion_plan_request = MotionPlanRequest()
        self.motion_plan_request.workspace_parameters.header.stamp.sec = sec
        self.motion_plan_request.workspace_parameters.header.stamp.nanosec = nanosec
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
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        self.goal_done = True
        time.sleep(1)


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('service_clients_python')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 1)
        self.create_subscription(Clock, '/clock', self.clock_cb, 1)

        self.block = "A"
        self.location = "T1"

        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
            rclpy.spin_once(self)

        self.tf_base_link_location = {}
        self.tf_base_link_block = None
        self.create_timer(1.0, self.on_timer)

        self.sec = None
        self.nanosec = None

    def clock_cb(self, msg):
        self.sec = msg.clock.sec
        self.nanosec = msg.clock.nanosec

    def on_timer(self):
        if "T1" not in self.tf_base_link_location.keys() \
            or "T2" not in self.tf_base_link_location.keys() \
            or "T3" not in self.tf_base_link_location.keys():
            try:
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    't1',
                    rclpy.time.Time()
                )
                self.tf_base_link_location['T1'] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    't2',
                    rclpy.time.Time()
                )
                self.tf_base_link_location['T2'] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    't3',
                    rclpy.time.Time()
                )
                self.tf_base_link_location['T3'] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform base_link to can: {ex}')
                return
        
        # Try to get t1, t2, t3 before anything else
        if len(self.tf_base_link_location.keys()) < 3:
            return
        
        try:
            if self.location in "ABC":
                t = self.tf_buffer.lookup_transform(
                    'base_link',
                    self.location,
                    rclpy.time.Time()
                )
                self.tf_base_link_location[self.location] = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

            t = self.tf_buffer.lookup_transform(
                'base_link',
                self.block,
                rclpy.time.Time()
            )
            self.tf_base_link_block = [t.transform.translation.x,t.transform.translation.y,t.transform.translation.z,
                                  t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]

        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform base_link to can: {ex}')
            return

    def joint_states_cb(self, joint_state):
        self.joint_state = joint_state

    def moveit_ik(self):
        self.req = GetPositionIK.Request()

        group_name = 'spot_arm'
        self.req.ik_request.group_name = group_name
        self.req.ik_request.robot_state.joint_state = self.joint_state
        self.req.ik_request.avoid_collisions = True

        self.req.ik_request.pose_stamped.header.stamp.sec = self.sec
        self.req.ik_request.pose_stamped.header.stamp.nanosec = self.nanosec
        self.req.ik_request.pose_stamped.header.frame_id = 'base_link'

        if self.is_block:
            if self.is_above:
                xyz = [
                    self.tf_base_link_block[0],
                    self.tf_base_link_block[1],
                    self.tf_base_link_block[2]
                ]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.14
            else:
                xyz = [
                    self.tf_base_link_block[0],
                    self.tf_base_link_block[1],
                    self.tf_base_link_block[2]
                ]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.04
        else:
            if self.is_above:
                xyz = [
                    self.tf_base_link_location[self.location][0],
                    self.tf_base_link_location[self.location][1],
                    self.tf_base_link_location[self.location][2]
                ]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.17
            else:
                xyz = [
                    self.tf_base_link_location[self.location][0],
                    self.tf_base_link_location[self.location][1],
                    self.tf_base_link_location[self.location][2]
                ]
                self.req.ik_request.pose_stamped.pose.position.x = xyz[0]
                self.req.ik_request.pose_stamped.pose.position.y = xyz[1]
                self.req.ik_request.pose_stamped.pose.position.z = xyz[2] + 0.08

        self.req.ik_request.timeout.sec = 5

        return self.req.ik_request

    def send_request(self, block, location, is_block, is_above):
        self.block = block
        self.location = location

        self.is_block = is_block
        self.is_above = is_above

        self.joint_state = None
        self.tf_base_link_block = None
        self.sec = None
        self.nanosec = None

        while self.joint_state is None or self.tf_base_link_block is None or self.sec == None or self.nanosec == None:
            rclpy.spin_once(self)

        self.req.ik_request = self.moveit_ik()

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)

        target_angles = list(self.future.result().solution.joint_state.position)[:7]

        return target_angles if len(target_angles) > 0 else None, self.sec, self.nanosec



def begin_stacking(stack_gh):
    result = Stack.Result()
    result.stacked = False

    block = stack_gh.request.block.upper()
    location = stack_gh.request.location.upper()
    moveit_action_client._logger.info(block + ':' + location)

    # Open gripper
    future = open_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(service_client, future)
    time.sleep(1)

    # Move above block
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=True, is_above=True)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    # Move to block
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=True, is_above=False)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    # Close gripper
    future = close_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(service_client, future)
    time.sleep(1)

    # Move above block
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=True, is_above=True)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    # Move above location
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=False, is_above=True)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    # Move to location
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=False, is_above=False)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    # Open gripper
    future = open_gripper_client.call_async(SpotMotion.Request())
    rclpy.spin_until_future_complete(service_client, future)
    time.sleep(1)

    # Move above location
    target_angles, sec, nanosec = service_client.send_request(block, location, is_block=False, is_above=True)
    if target_angles == None:
        moveit_action_client._logger.info('No IK solution found')
        stack_gh.abort()
        return result
    moveit_action_client._logger.info(str([round(np.rad2deg(ta)) for ta in target_angles]))
    moveit_action_client.send_goal(target_angles, sec, nanosec)
    while not moveit_action_client.goal_done:
        rclpy.spin_once(moveit_action_client)

    for x in "ABC":
        if x in service_client.tf_base_link_location.keys():
            service_client.tf_base_link_location.pop(x)

    stack_gh.succeed()
    result.stacked = True
    return result

from rclpy.executors import MultiThreadedExecutor

rclpy.init()

stacker_node = Node('stacker_python')

service_client = MinimalClientAsync()

close_gripper_client = service_client.create_client(SpotMotion, '/Spot/close_gripper')
open_gripper_client = service_client.create_client(SpotMotion, '/Spot/open_gripper')

moveit_action_client = MoveGroupActionClient()


def main():
    action_server = ActionServer(
                stacker_node,
                Stack,
                'stack',
                begin_stacking)

    executor = MultiThreadedExecutor()
    executor.add_node(stacker_node)
    executor.spin()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
