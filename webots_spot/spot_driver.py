import rclpy
from rclpy.node import Node

from spot_msgs.msg import GaitInput

import numpy as np
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

NUMBER_OF_JOINTS = 12

class SpotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.spot_node = self.__robot.getFromDef("Spot")
        self.__robot.timestep = 32

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
        self.__node.get_logger().info('Init SpotDriver')
        ## Topics
        self.__node.create_subscription(GaitInput, '/Spot/inverse_gait_input', self.__gait_cb ,1)
        ## Webots Touch Sensors
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

        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)
        self.bzg = BezierGait(dt=0.032)
        self.motors_initial_pos = []

        # ------------------ Inputs for Bezier Gait control ----------------
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.1
        self.rolld = 0.0
        self.pitchd = 0.0
        self.yawd = 0.0
        self.StepLength = 0.0
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 0.0
        self.ClearanceHeight = 0.0
        self.PenetrationDepth = 0.0
        self.SwingPeriod = 0.0
        self.YawControl = 0.0
        self.YawControlOn = False

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

    def stand_up(self):
        self.__node.get_logger().info('Standup')
        motor_pos = [0.20, 0.7, -1.39 ,-0.20, 0.7, -1.39 ,0.20, 0.7, -1.39 ,-0.20, 0.7, -1.39]
        self.__talker(motor_pos)

    def sit_down(self):
        self.__node.get_logger().info('Sitdown')
        motor_pos = [0.20, 0.7, -1.39 ,-0.20, 0.7, -1.39 ,0.20, 0.7, -1.39 ,-0.20, 0.7, -1.39]
        self.__talker(motor_pos)

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
        self.xd = msg.x
        self.yd = msg.y
        self.zd = msg.z
        self.rolld = msg.roll
        self.pitchd = msg.pitch
        self.yawd = msg.yaw
        self.StepLength = msg.step_length
        self.LateralFraction = msg.lateral_fraction
        self.YawRate = msg.yaw_rate
        self.StepVelocity = msg.step_velocity
        self.ClearanceHeight = msg.clearance_height
        self.PenetrationDepth = msg.penetration_depth
        self.SwingPeriod = msg.swing_period
        self.YawControl = msg.yaw_control
        self.YawControlOn = msg.yaw_control_on

    def __talker(self, motors_target_pos):
        for idx, motor in enumerate(self.motors):
            motor.setPosition(motors_target_pos[idx] - self.motors_initial_pos[idx])

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
        joint_angles = -self.spot.IK(orn, pos, T_bf)

        target = [
            joint_angles[0][0], joint_angles[0][1], joint_angles[0][2],
            joint_angles[1][0], joint_angles[1][1], joint_angles[1][2],
            joint_angles[2][0], joint_angles[2][1], joint_angles[2][2],
            joint_angles[3][0], joint_angles[3][1], joint_angles[3][2],
            ]

        if not self.motors_initial_pos:
            self.motors_initial_pos = target
        self.__talker(target)

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
        #Update Spot state
        self.__model_cb()
