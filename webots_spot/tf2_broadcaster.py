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

    def handle_pose(self, motor_pos, gps, imu):
        tfs = TransformStamped()

        ## Base_Link to Front Left Leg
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id= "base_link"
        tfs._child_frame_id = "front left leg"
        tfs.transform.translation.x = 0.3635
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0118
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "front left leg"
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
        tfs._child_frame_id = "front right leg"
        tfs.transform.translation.x = 0.3635
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0118
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "front right leg"
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
        tfs._child_frame_id = "rear left leg"
        tfs.transform.translation.x = -0.3084
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0117
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "rear left leg"
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
        tfs._child_frame_id = "rear right leg"
        tfs.transform.translation.x = -0.3084
        tfs.transform.translation.y = 0.
        tfs.transform.translation.z = 0.0117
        r = R.from_euler('xyz',[pi/2,0,-pi/2])
        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]
        self.tfb_.sendTransform(tfs)
        
        tfs.header.frame_id= "rear right leg"
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

        ## Map To Base_Link
        tfs.header.frame_id= "map"
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
