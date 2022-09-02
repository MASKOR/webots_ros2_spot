import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from spot_msgs.msg import GaitInput
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

from scipy.spatial.transform import Rotation as R
import numpy as np
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

from webots_spot.tf2_broadcaster import DynamicBroadcaster

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
        
        ## Positional Sensors
        self.motor_sensor_names = [
            "front left shoulder abduction sensor",  "front left shoulder rotation sensor",  "front left elbow sensor",
            "front right shoulder abduction sensor", "front right shoulder rotation sensor", "front right elbow sensor",
            "rear left shoulder abduction sensor",   "rear left shoulder rotation sensor",   "rear left elbow sensor",
            "rear right shoulder abduction sensor",  "rear right shoulder rotation sensor",  "rear right elbow sensor"
        ]
        self.motor_sensors = []
        self.motors_pos = []
        for idx, sensor_name in enumerate(self.motor_sensor_names):
            self.motor_sensors.append(self.__robot.getDevice(sensor_name))
            self.motor_sensors[idx].enable(self.__robot.timestep)
            self.motors_pos.append(0.)

        ## GPS
        self.gps_sensor = self.__robot.getDevice("gps")
        self.gps_sensor.enable(self.__robot.timestep)

        ## IMU
        self.inertial_unit = self.__robot.getDevice("inertial unit")
        self.inertial_unit.enable(self.__robot.timestep)

        ## Gyro
        self.gyro = self.__robot.getDevice("gyro")
        self.gyro.enable(self.__robot.timestep)

        rclpy.init(args=None)
        self.ros_clock = Clock()

        self.__node = rclpy.create_node('spot_driver')
        self.__node.get_logger().info('Init SpotDriver')

        self.tf2_broadcaster = DynamicBroadcaster()

        ## Topics
        self.__node.create_subscription(GaitInput, '/Spot/inverse_gait_input', self.__gait_cb, 1)
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel, 10)
        self.odom_pub = self.__node.create_publisher(Odometry, '/Spot/odometry', 10)
        
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

    def __cmd_vel(self, msg):
        if not self.__node.count_publishers('/Spot/inverse_gait_input'):
            StepLength = 0.024
            ClearanceHeight = 0.024
            PenetrationDepth = 0.003
            SwingPeriod = 0.2
            YawControl = 0.0
            YawControlOn = 0.0
            
            self.xd = 0.
            self.yd = 0.
            self.zd = 0.1
            self.rolld = 0.
            self.pitchd = 0.
            self.yawd = 0.
            self.StepLength = StepLength
            self.LateralFraction = msg.linear.y
            self.YawRate = msg.angular.z
            self.StepVelocity = msg.linear.x
            if not self.StepVelocity :
                if self.LateralFraction != 0:
                    self.StepVelocity = 0.2
                if self.YawRate != 0:
                    self.StepVelocity = 0.01

            self.ClearanceHeight = ClearanceHeight
            self.PenetrationDepth = PenetrationDepth
            self.SwingPeriod = SwingPeriod
            self.YawControl = YawControl
            self.YawControlOn = YawControlOn

    def __gait_cb(self, msg):
        if self.__node.count_publishers('/Spot/inverse_gait_input') > 0:
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
        self.handle_transforms_and_odometry()

    def handle_transforms_and_odometry(self):
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()

        gps = self.gps_sensor.getValues()
        linear_twist = self.gps_sensor.getSpeedVector()
        imu = self.inertial_unit.getRollPitchYaw()
        gyro = self.gyro.getValues()

        time_stamp = self.ros_clock.now().to_msg()

        self.tf2_broadcaster.handle_pose(self.motors_pos, gps, imu, time_stamp)

        odom = Odometry()
        odom.header.frame_id = 'odom'
        odom.header.stamp = time_stamp
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = gps[0]
        odom.pose.pose.position.y = gps[1]
        odom.pose.pose.position.z = gps[2]
        r = R.from_euler('xyz',[imu[0],imu[1],imu[2]])
        odom.pose.pose.orientation.x = r.as_quat()[0]
        odom.pose.pose.orientation.y = r.as_quat()[1]
        odom.pose.pose.orientation.z = r.as_quat()[2]
        odom.pose.pose.orientation.w = r.as_quat()[3]
        odom.twist.twist.linear.x = linear_twist[0]
        odom.twist.twist.linear.y = linear_twist[1]
        odom.twist.twist.linear.z = linear_twist[2]
        odom.twist.twist.angular.x = gyro[0]
        odom.twist.twist.angular.y = gyro[1]
        odom.twist.twist.angular.z = gyro[2]
        self.odom_pub.publish(odom)

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
        