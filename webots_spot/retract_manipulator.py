import rclpy
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math
import time

def main():
    rclpy.init()
    node = rclpy.create_node('action_client_node')

    # Create an action client for the spotarm_joint_trajectory_controller
    arm_client = ActionClient(node, FollowJointTrajectory, '/spotarm/joint_trajectory_controller/follow_joint_trajectory')

    # Create an action client for the tiago_gripper_joint_trajectory_controller
    gripper_client = ActionClient(node, FollowJointTrajectory, '/spotarm/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory')

    arm_client.wait_for_server()
    time.sleep(1)

    gripper_client.wait_for_server()
    time.sleep(1)

    # Create a goal request to set arm joint positions
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = ["spotarm_1_joint", "spotarm_2_joint", "spotarm_3_joint", "spotarm_4_joint", "Slider11", "spotarm_5_joint", "spotarm_6_joint"]
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [0.0, math.radians(179), math.radians(170), 0.0, 0.0, math.radians(11), 0.0]
    arm_goal_msg.trajectory.points.append(arm_point)

    # Create a goal request to set gripper joint positions
    gripper_goal_msg = FollowJointTrajectory.Goal()
    gripper_goal_msg.trajectory.joint_names = ["gripper_left_finger_joint", "gripper_right_finger_joint"]
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0.045, 0.045]
    gripper_goal_msg.trajectory.points.append(gripper_point)

    # Send the arm goal
    arm_client.send_goal_async(arm_goal_msg)

    # Send the gripper goal
    gripper_client.send_goal_async(gripper_goal_msg)

    node.destroy_node()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
