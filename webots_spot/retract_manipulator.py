import rclpy
from rclpy.action import ActionClient
from rosgraph_msgs.msg import Clock
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
import math


def main():
    rclpy.init()
    node = rclpy.create_node("retract_manipulator_node")

    def increment_count(_):
        nonlocal clock_msg_count
        clock_msg_count += 1

    clock_msg_count = 0

    node.create_subscription(Clock, "/clock", increment_count, 1)

    # Create an action client for the spotarm_joint_trajectory_controller
    arm_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/spotarm_joint_trajectory_controller/follow_joint_trajectory",
    )

    # Create an action client for the tiago_gripper_joint_trajectory_controller
    gripper_client = ActionClient(
        node,
        FollowJointTrajectory,
        "/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory",
    )

    arm_client.wait_for_server()
    gripper_client.wait_for_server()

    # Wait for simulation clock to initiate
    while clock_msg_count < 40:
        rclpy.spin_once(node)

    # Create a goal request to set arm joint positions
    arm_goal_msg = FollowJointTrajectory.Goal()
    arm_goal_msg.trajectory.joint_names = [
        "spotarm_1_joint",
        "spotarm_2_joint",
        "spotarm_3_joint",
        "spotarm_4_joint",
        "Slider11",
        "spotarm_5_joint",
        "spotarm_6_joint",
    ]
    arm_point = JointTrajectoryPoint()
    arm_point.positions = [
        0.0,
        math.radians(179),
        math.radians(170),
        0.0,
        0.0,
        math.radians(11),
        0.0,
    ]
    arm_goal_msg.trajectory.points.append(arm_point)

    # Create a goal request to set gripper joint positions
    gripper_goal_msg = FollowJointTrajectory.Goal()
    gripper_goal_msg.trajectory.joint_names = [
        "gripper_left_finger_joint",
        "gripper_right_finger_joint",
    ]
    gripper_point = JointTrajectoryPoint()
    gripper_point.positions = [0.045, 0.045]
    gripper_goal_msg.trajectory.points.append(gripper_point)

    # Send action goals
    arm_client.send_goal_async(arm_goal_msg)
    gripper_client.send_goal_async(gripper_goal_msg)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
