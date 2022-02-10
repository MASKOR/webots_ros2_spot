from ctypes.wintypes import INT
import imp
from matplotlib.pyplot import step

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from spot_msg_interface.msg import Legs

import sys
import numpy as np
import math
import time
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

NUMBER_OF_JOINTS = 12

class SpotDriver:
    def init(self, webots_node, properties):
        print('Hello Init')
        self.__robot = webots_node.robot
        self.spot_node = self.__robot.getFromDef("Spot")
        self.__robot.timestep = 32
        index = 0

        ### Init motors        
        self.motor_names = [
            "front left shoulder abduction motor",  "front left shoulder rotation motor",  "front left elbow motor",
            "front right shoulder abduction motor", "front right shoulder rotation motor", "front right elbow motor",
            "rear left shoulder abduction motor",   "rear left shoulder rotation motor",   "rear left elbow motor",
            "rear right shoulder abduction motor",  "rear right shoulder rotation motor",  "rear right elbow motor"
        ]

        self.motors = []
        for motor_name in self.motor_names:
            self.motors.append(self.__robot.getDevice(motor_name))

        rclpy.init(args=None)

        self.__node = rclpy.create_node('spot_driver')
        
        ### Topics
        self.__node.create_subscription(Legs, 'Spot/move_decomp', self.__motor_cb, 2)
        self.__node.create_subscription(Bool, 'Spot/give_paw',self.__give_paw_cb, 1)
        self.__node.create_subscription(Twist, 'cmd_vel', self.__gait_cb ,1)
        # TouchSensor
        self.__touch_fl_pub = self.__node.create_publisher(Bool, 'Spot/touch_fl', 1)
        self.__touch_fr_pub = self.__node.create_publisher(Bool, 'Spot/touch_fr', 1)
        self.__touch_rl_pub = self.__node.create_publisher(Bool, 'Spot/touch_rl', 1)
        self.__touch_rr_pub = self.__node.create_publisher(Bool, 'Spot/touch_rr', 1)

        n_devices = self.__robot.getNumberOfDevices()
        self.__node.get_logger().info('Number of Devices'+str(n_devices))
        while index < n_devices:
            self.__node.get_logger().info("index: "+ str(index)+" "+ str(self.__robot.getDeviceByIndex(index)))
            index += 1

        ### Touch Sensors
        self.touch_fl = self.__robot.getDevice("front left touch sensor")
        self.touch_fr = self.__robot.getDevice("front right touch sensor")
        self.touch_rl = self.__robot.getDevice("rear left touch sensor")
        self.touch_rr = self.__robot.getDevice("rear right touch sensor")

        self.touch_fl.enable(self.__robot.timestep)
        self.touch_fr.enable(self.__robot.timestep)
        self.touch_rl.enable(self.__robot.timestep)
        self.touch_rr.enable(self.__robot.timestep)

        ## Spot Control
        self.rate = self.__node.create_rate(100)
        self.time_step = self.__robot.timestep

        self.__node.get_logger().info('timestep: '+str(self.time_step))

        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=0.032)

        # ------------------ Inputs for Bezier Gait control ----------------
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.1
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0
        self.StepLength = 0.1
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 0.8
        self.ClearanceHeight = 0.15
        self.PenetrationDepth = 0.00003
        self.SwingPeriod = 0.3
        self.YawControl = 0.0
        self.YawControlOn = True

        # ------------------ Spot states ----------------
        self.x_inst = 0.
        self.y_inst = 0.
        self.z_inst = 0.
        self.roll_inst = 0.
        self.pitch_inst = 0.
        self.yaw_inst = 0.
        self.search_index = -1

        # ------------------ Outputs of Contact sensors ----------------
        self.front_left_lower_leg_contact = 1
        self.front_right_lower_leg_contact = 1
        self.rear_left_lower_leg_contact = 1
        self.rear_right_lower_leg_contact = 1
        self.chattering_front_left_lower_leg_contact = 0
        self.chattering_front_right_lower_leg_contact = 0
        self.chattering_rear_left_lower_leg_contact = 0
        self.chattering_rear_right_lower_leg_contact = 0
        self.lim_chattering = 4

    def __model_cb(self):
        spot_rot = self.spot_node.getField("rotation")
        spot_rot_val = spot_rot.getSFRotation()
        self.yaw_inst = spot_rot_val[2]

    def yaw_control(self):
        """ Yaw body controller"""
        yaw_target = self.YawControl
        thr = np.pi / 2
        if (yaw_target > thr and self.yaw_inst < -thr) or (self.yaw_inst > thr and yaw_target < -thr):
            residual = (yaw_target - self.yaw_inst) * np.sign(yaw_target - self.yaw_inst) - 2 * np.pi
            yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
        else:
            residual = yaw_target - self.yaw_inst
            yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
        return yawrate_d

    def __gait_cb(self, msg):
        self.xd = msg.linear.x
        self.yd = msg.linear.y
        self.zd = msg.linear.z
        self.rolld = msg.angular.x
        self.pitchd = msg.angular.y
        self.yawd = msg.angular.z

    def __talker(self, motors_target_pos):
        for idx, motor in enumerate(self.motors):
            motor.setPosition(motors_target_pos[idx])

    def spot_inverse_control(self):
        pos = np.array([self.xd, self.yd, self.zd])
        orn = np.array([self.rolld, self.pitchd, self.yawd])

        # yaw controller
        if self.YawControlOn == 1.0:
            YawRate_desired = self.yaw_control()
        else:
            YawRate_desired = self.YawRate

        # Update Swing Period
        self.bzg.Tswing = self.SwingPeriod
        contacts = [self.front_left_lower_leg_contact, self.front_right_lower_leg_contact,
                    self.rear_left_lower_leg_contact,
                    self.rear_right_lower_leg_contact]

        # Get Desired Foot Poses
        T_bf = self.bzg.GenerateTrajectory(self.StepLength, self.LateralFraction, YawRate_desired,
                                           self.StepVelocity, self.T_bf0, self.T_bf,
                                           self.ClearanceHeight, self.PenetrationDepth,
                                           contacts)
        joint_angles = self.spot.IK(orn, pos, T_bf)

        target = [
            joint_angles[0][0], joint_angles[0][1], joint_angles[0][2],
            joint_angles[1][0], joint_angles[1][1], joint_angles[1][2],
            joint_angles[2][0], joint_angles[2][1], joint_angles[2][2],
            joint_angles[3][0], joint_angles[3][1], joint_angles[3][2],
            ]
        self.__talker(target)

    def __motor_cb(self, msg):
        self.__node.get_logger().info("Talker")
        self.movement_decomposition(msg.leg, 1.0)

    def __give_paw_cb(self, msg):

        pos1 = [-0.20, -0.30, 0.05, 0.20, -0.40, -0.19, -0.40, -0.90, 1.18, 0.49, -0.90, 0.80]
        pos2 = [-0.20, -0.40, -0.19, 0.20, -0.40, -0.19, -0.40, -0.90, 1.18, 0.40,  -0.90, 1.18]

        self.movement_decomposition(pos1, 4.0)

        initial_time = self.__robot.getTime()

        while self.__robot.getTime() - initial_time < 8:
            self.motors[4].setPosition( 0.2 * math.sin(2 * self.__robot.getTime() + 0.6))
            self.motors[5].setPosition( 0.4 * math.sin(2 * self.__robot.getTime()))
            self.step()

        self.movement_decomposition(pos2, 4.0)

    def movement_decomposition(self, target, duration):
        """ Send command to actuators of joints

        """
        self.__node.get_logger().info("movement decomposition")
        time_step = self.__robot.getBasicTimeStep()

        self.__node.get_logger().info("timestep: "+ str(time_step))
        
        n_steps_to_achieve_target = duration*1000/time_step
        self.__node.get_logger().info("Steps: "+str(n_steps_to_achieve_target))
        current_pos = []
        step_difference = []
        self.__node.get_logger().info("Target: " + str(target))
        for idx, motor in enumerate(self.motors):
            current_pos.append(motor.getTargetPosition())
            step_difference.append( ((target[idx] - current_pos[idx]) / n_steps_to_achieve_target) )

        for step in range(int(n_steps_to_achieve_target)):
            for idx, motor in enumerate(self.motors):
                current_pos[idx] = step_difference[idx] + current_pos[idx]
                motor.setPosition(current_pos[idx])              
            self.step()

    def callback_front_left_lower_leg_contact(self, data):
        if data == 0:
            self.chattering_front_left_lower_leg_contact += 1
            if self.chattering_front_left_lower_leg_contact > self.lim_chattering:
                self.front_left_lower_leg_contact = 0
        else:
            self.front_left_lower_leg_contact = 1
            self.chattering_front_left_lower_leg_contact = 0


    def callback_front_right_lower_leg_contact(self, data):
        if data == 0:
            self.chattering_front_right_lower_leg_contact += 1
            if self.chattering_front_right_lower_leg_contact > self.lim_chattering:
                self.front_right_lower_leg_contact = 0
        else:
            self.front_right_lower_leg_contact = 1
            self.chattering_front_right_lower_leg_contact = 0

    def callback_rear_left_lower_leg_contact(self, data):
        if data == 0:
            self.chattering_rear_left_lower_leg_contact += 1
            if self.chattering_rear_left_lower_leg_contact > self.lim_chattering:
                self.rear_left_lower_leg_contact = 0
        else:
            self.rear_left_lower_leg_contact = 1
            self.chattering_rear_left_lower_leg_contact = 0

    def callback_rear_right_lower_leg_contact(self, data):
        if data == 0:
            self.chattering_rear_right_lower_leg_contact += 1
            if self.chattering_rear_right_lower_leg_contact > self.lim_chattering:
                self.rear_right_lower_leg_contact = 0
        else:
            self.rear_right_lower_leg_contact = 1
            self.chattering_rear_right_lower_leg_contact = 0

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.callback_front_left_lower_leg_contact(bool(self.touch_fl.getValue()))
        self.callback_front_right_lower_leg_contact(bool(self.touch_fr.getValue()))
        self.callback_rear_left_lower_leg_contact(bool(self.touch_rl.getValue()))
        self.callback_rear_right_lower_leg_contact(bool(self.touch_rr.getValue()))

        self.spot_inverse_control()
        self.__model_cb()
        self.__robot.step(32)