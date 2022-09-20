#! /usr/bin/env python3
from math import pi
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster


class DynamicBroadcaster(Node):
    def __init__(self):
        super().__init__('dynamic_broadcaster')
        self.tfb_ = TransformBroadcaster(self)

    def handle_pose(self, motor_pos, gps, imu, time_stamp):
        tfs = TransformStamped()

        ## Base_Link to Front Left Leg
        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "base_link"
        tfs._child_frame_id = "solid" # front left leg
        tfs.transform.translation.x = 0.3635
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0118
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "solid" # front left leg
        tfs._child_frame_id = "front left shoulder"
        tfs.transform.translation.x = -0.0528
        tfs.transform.translation.y = 0.0006
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,motor_pos[0]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "front left shoulder"
        tfs._child_frame_id = "front left upperarm"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = -0.00053
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[motor_pos[1],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "front left upperarm"
        tfs._child_frame_id = "front left forearm"
        tfs.transform.translation.x = -0.1122
        tfs.transform.translation.y = -0.319729
        tfs.transform.translation.z = 0.182338
        r = R.from_euler('xyz',[motor_pos[2],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        ## Base_Link to Front Right Leg
        tfs.header.frame_id= "base_link"
        tfs._child_frame_id = "solid_0" # front right leg
        tfs.transform.translation.x = 0.3635
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0118
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "solid_0" # front right leg
        tfs._child_frame_id = "front right shoulder"
        tfs.transform.translation.x = 0.0528
        tfs.transform.translation.y = 0.0006
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,motor_pos[3]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "front right shoulder"
        tfs._child_frame_id = "front right upperarm"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = -0.00053
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[motor_pos[4],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "front right upperarm"
        tfs._child_frame_id = "front right forearm"
        tfs.transform.translation.x = 0.1122
        tfs.transform.translation.y = -0.319729
        tfs.transform.translation.z = 0.182338
        r = R.from_euler('xyz',[motor_pos[5],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        ## Base_Link to Rear Left Leg
        tfs.header.frame_id= "base_link"
        tfs._child_frame_id = "solid_1" # rear left leg
        tfs.transform.translation.x = -0.3084
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0117
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "solid_1" # rear left leg
        tfs._child_frame_id = "rear left shoulder"
        tfs.transform.translation.x = -0.0528
        tfs.transform.translation.y = 0.0006
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,motor_pos[6]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "rear left shoulder"
        tfs._child_frame_id = "rear left upperarm"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = -0.00053
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[motor_pos[7],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "rear left upperarm"
        tfs._child_frame_id = "rear left forearm"
        tfs.transform.translation.x = -0.1122
        tfs.transform.translation.y = -0.319729
        tfs.transform.translation.z = 0.182338
        r = R.from_euler('xyz',[motor_pos[8],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        ## Base_Link to Rear Right Leg
        tfs.header.frame_id= "base_link"
        tfs._child_frame_id = "solid_2" # rear right leg
        tfs.transform.translation.x = -0.3084
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0117
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "solid_2" # rear right leg
        tfs._child_frame_id = "rear right shoulder"
        tfs.transform.translation.x = 0.0528
        tfs.transform.translation.y = 0.0006
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,motor_pos[9]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "rear right shoulder"
        tfs._child_frame_id = "rear right upperarm"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = -0.00053
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[motor_pos[10],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "rear right upperarm"
        tfs._child_frame_id = "rear right forearm"
        tfs.transform.translation.x = 0.1122
        tfs.transform.translation.y = -0.319729
        tfs.transform.translation.z = 0.182338
        r = R.from_euler('xyz',[motor_pos[11],0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        ## Odom To Base_Link
        tfs.header.frame_id= "odom"
        tfs._child_frame_id = "base_link"
        tfs.transform.translation.x = gps[0]
        tfs.transform.translation.y = gps[1]
        tfs.transform.translation.z = gps[2]
        r = R.from_euler('xyz',[imu[0],imu[1],imu[2]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

    def ur3e_handle_pose(self, ur3e_motor_pos, time_stamp):
        tfs = TransformStamped()

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "UR3e"
        tfs._child_frame_id = "shoulder_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.152
        r = R.from_euler('xyz',[0,0,ur3e_motor_pos[0]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "shoulder_link"
        tfs._child_frame_id = "solid_3"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.12
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,pi/2,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "solid_3"
        tfs._child_frame_id = "upper_arm_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,ur3e_motor_pos[1],0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "upper_arm_link"
        tfs._child_frame_id = "forearm_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = -0.093
        tfs.transform.translation.z = 0.244
        r = R.from_euler('xyz',[0,ur3e_motor_pos[2],0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "forearm_link"
        tfs._child_frame_id = "solid_4"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.213
        r = R.from_euler('xyz',[0,pi/2,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "solid_4"
        tfs._child_frame_id = "wrist_1_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,ur3e_motor_pos[3],0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "wrist_1_link"
        tfs._child_frame_id = "wrist_2_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.104
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,ur3e_motor_pos[4]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.frame_id= "wrist_2_link"
        tfs._child_frame_id = "wrist_3_link"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.085
        r = R.from_euler('xyz',[0,ur3e_motor_pos[5],0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

    def gripper_handle_pose(self, gripper_motor_pos, time_stamp):
        tfs = TransformStamped()

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "wrist_3_link"
        tfs._child_frame_id = "ROBOTIQ 3f Gripper"
        tfs.transform.translation.x = 0.
        tfs.transform.translation.y = 0.14
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "ROBOTIQ 3f Gripper"
        tfs._child_frame_id = "finger_1_link_0"
        tfs.transform.translation.x = -0.045
        tfs.transform.translation.y = 0.021
        tfs.transform.translation.z = 0.036
        r = R.from_euler('xyz',[pi,0,pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_1_link_0"
        tfs._child_frame_id = "finger_1_link_1"
        tfs.transform.translation.x = 0.02
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,gripper_motor_pos[0]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_1_link_1"
        tfs._child_frame_id = "finger_1_link_2"
        tfs.transform.translation.x = 0.05
        tfs.transform.translation.y = -0.028
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,-0.52])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_1_link_2"
        tfs._child_frame_id = "finger_1_link_3"
        tfs.transform.translation.x = 0.039
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,0])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "ROBOTIQ 3f Gripper"
        tfs._child_frame_id = "finger_2_link_0"
        tfs.transform.translation.x = -0.045
        tfs.transform.translation.y = 0.021
        tfs.transform.translation.z = -0.036
        r = R.from_euler('xyz',[pi,0,pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_2_link_0"
        tfs._child_frame_id = "finger_2_link_1"
        tfs.transform.translation.x = 0.02
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,gripper_motor_pos[1]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_2_link_1"
        tfs._child_frame_id = "finger_2_link_2"
        tfs.transform.translation.x = 0.05
        tfs.transform.translation.y = -0.028
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,-0.520])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_2_link_2"
        tfs._child_frame_id = "finger_2_link_3"
        tfs.transform.translation.x = 0.039
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,-0.052])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "ROBOTIQ 3f Gripper"
        tfs._child_frame_id = "finger_middle_link_1"
        tfs.transform.translation.x = 0.046
        tfs.transform.translation.y = 0.041
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,1.619+gripper_motor_pos[2]])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_middle_link_1"
        tfs._child_frame_id = "finger_middle_link_2"
        tfs.transform.translation.x = 0.05
        tfs.transform.translation.y = -0.028
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,-0.520])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)

        tfs.header.stamp = time_stamp
        tfs.header.frame_id= "finger_middle_link_2"
        tfs._child_frame_id = "finger_middle_link_3"
        tfs.transform.translation.x = 0.039
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.
        r = R.from_euler('xyz',[0,0,-0.052])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
