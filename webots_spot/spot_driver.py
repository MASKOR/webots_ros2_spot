import rclpy
from rclpy.node import Node
from rclpy.clock import Clock

from spot_msgs.msg import GaitInput
from spot_msgs.srv import SpotMotion
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster

from scipy.spatial.transform import Rotation as R
import numpy as np
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

NUMBER_OF_JOINTS = 12

motions = {
    'stand': [-0.1, 0.0, 0.0, 0.1, 0.0, 0.0, -0.1, 0.0, 0.0, 0.1, 0.0, 0.0],
    'sit'  : [-0.20, -0.40, -0.19, 0.20, -0.40, -0.19, -0.40, -0.90, 1.18, 0.40, -0.90, 1.18],
    'lie'  : [-0.40, -0.99, 1.59, 0.40, -0.99, 1.59, -0.40, -0.99, 1.59, 0.40, -0.99, 1.59],
}


class SpotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        self.spot_node = self.__robot.getFromDef("Spot")
        self.__robot.timestep = int(self.__robot.getBasicTimeStep())

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

        # manually calculated offsets of webots' spot wrt vertical axis
        self.motors_initial_pos = [
            0.001, -0.5185, 1.21,
            0.001, -0.5185, 1.21,
            0.001, -0.5185, 1.21,
            0.001, -0.5185, 1.21,
            ]

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

        self.__node = Node('spot_driver')
        self.__node.get_logger().info('Init SpotDriver')

        self.tfb_ = TransformBroadcaster(self.__node)

        ## Topics
        self.__node.create_subscription(GaitInput, '/Spot/inverse_gait_input', self.__gait_cb, 1)
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel, 1)
        self.odom_pub = self.__node.create_publisher(Odometry, '/Spot/odometry', 1)
        self.joint_state_pub = self.__node.create_publisher(JointState, '/joint_states', 1)

        ## Services
        self.__node.create_service(SpotMotion, '/Spot/stand_up', self.__stand_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/sit_down', self.__sit_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/lie_down', self.__lie_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/shake_hand', self.__shakehand_motion_cb)

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
        self.bzg = BezierGait(dt=self.__robot.timestep/1000)

        # ------------------ Inputs for Bezier Gait control ----------------
        self.xd = 0.0
        self.yd = 0.0
        self.zd = 0.0
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

        # Motion command
        self.fixed_motion = False

        self.n_steps_to_achieve_target = 0
        self.step_difference = [0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.,0.]
        self.m_target = []
        self.paw = False
        self.paw2 = False
        self.paw_time = 0.
        self.previous_cmd = False

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
        # Override motion command
        self.fixed_motion = False

        if not self.__node.count_publishers('/Spot/inverse_gait_input'):
            StepLength = 0.08
            ClearanceHeight = 0.01
            PenetrationDepth = 0.003
            SwingPeriod = 0.4
            YawControl = 0.0
            YawControlOn = 0.0
            StepVelocity = 0.2

            self.xd = 0.
            self.yd = 0.
            self.zd = 0.
            self.rolld = 0.
            self.pitchd = 0.
            self.yawd = 0.
            self.StepLength = StepLength * msg.linear.x

            # Rotation along vertical axis
            self.YawRate = msg.angular.z
            if self.YawRate != 0 and self.StepLength == 0:
                self.StepLength = StepLength * 0.1

            # Holonomic motions
            self.LateralFraction = np.arctan2(msg.linear.y, msg.linear.x)
            # forcefully set 0, as output is never 0 if second term in arctan2 is -ve
            if msg.linear.y == 0:
                self.LateralFraction = 0
            if self.LateralFraction != 0:
                if msg.linear.x != 0:
                    # change lateral fraction for 2nd and 4th quadrants
                    if msg.linear.x > 0 and self.LateralFraction < 0:
                        self.LateralFraction += np.pi
                    elif msg.linear.x < 0 and self.LateralFraction > 0:
                        self.LateralFraction -= np.pi
                    self.StepLength = StepLength * msg.linear.y * np.sign(msg.linear.x)
                else:
                    # for sideway motion
                    self.StepLength = StepLength * abs(msg.linear.y)

            self.StepVelocity = StepVelocity
            self.ClearanceHeight = ClearanceHeight
            self.PenetrationDepth = PenetrationDepth
            self.SwingPeriod = SwingPeriod
            self.YawControl = YawControl
            self.YawControlOn = YawControlOn

    def __gait_cb(self, msg):
        # Override motion command
        self.fixed_motion = False

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
        contacts = [
            self.front_left_lower_leg_contact,
            self.front_right_lower_leg_contact,
            self.rear_left_lower_leg_contact,
            self.rear_right_lower_leg_contact
            ]

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

        self.__talker(target)

    def handle_transforms_and_odometry(self):
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()

        gps = self.gps_sensor.getValues()
        linear_twist = self.gps_sensor.getSpeedVector()
        imu = self.inertial_unit.getRollPitchYaw()
        gyro = self.gyro.getValues()

        time_stamp = self.__node.get_clock().now().to_msg()

        ## Odom To Base_Link
        tfs = TransformStamped()
        tfs.header.stamp = time_stamp
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

        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.motor_names)
        joint_state.position = []
        joint_state.position.extend(self.motors_pos)
        qty = len(self.motor_names)
        joint_state.velocity = [0. for _ in range(qty)]
        joint_state.effort = [0. for _ in range(qty)]
        self.joint_state_pub.publish(joint_state)

    def movement_decomposition(self, target, duration):
        """
        Decompose big motion into smaller motions
        """
        self.previous_cmd = True

        self.n_steps_to_achieve_target = duration * 1000 / self.__robot.timestep
        self.step_difference = [(target[i] - self.motors_pos[i]) / self.n_steps_to_achieve_target
                                for i in range(NUMBER_OF_JOINTS)]
        self.m_target = []

    def __stand_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = 'performing previous command, override with bool argument'
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions['stand'], 1)
        response.answer = 'standing up'
        return response

    def __sit_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = 'performing previous command, override with bool argument'
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions['sit'], 1)
        response.answer = 'sitting down'
        return response

    def __lie_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = 'performing previous command, override with bool argument'
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions['lie'], 1)
        response.answer = 'lying down'
        return response

    def __shakehand_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = 'performing previous command, override with bool argument'
            return response

        # Start handshake motion
        self.movement_decomposition([-0.20, -0.30, 0.05,
                                        0.20, -0.40, -0.19,
                                        -0.40, -0.90, 1.18,
                                        0.49, -0.90, 0.80], 1)
        self.paw = True
        response.answer = 'shaking hands'
        return response

    def defined_motions(self):
        if self.n_steps_to_achieve_target > 0:
            if not self.m_target:
                self.m_target = [self.step_difference[i] + self.motors_pos[i] for i in range(NUMBER_OF_JOINTS)]
            else: # if compared to current motors_positions, the final motion is smaller
                self.m_target = [self.step_difference[i] + self.m_target[i] for i in range(NUMBER_OF_JOINTS)]

            # Increment motor positions by step_difference
            for idx, motor in enumerate(self.motors):
                motor.setPosition(self.m_target[idx])

            self.n_steps_to_achieve_target -= 1
        else:
            if self.paw:
                self.paw_time = self.__robot.getTime() + 4
                self.paw = False
                self.paw2 = True # Do the shakehands motion
            else:
                self.previous_cmd = False

            self.m_target = []

        if self.paw2:
            self.previous_cmd = True
            if self.paw_time > self.__robot.getTime():
                self.motors[4].setPosition(0.2 * np.sin(2 * self.__robot.getTime()) + 0.6)
                self.motors[5].setPosition(0.4 * np.sin(2 * self.__robot.getTime()))
            else:
                self.paw2 = False # Sit back again
                self.movement_decomposition([-0.20, -0.40, -0.19,
                                              0.20, -0.40, -0.19,
                                             -0.40, -0.90, 1.18,
                                              0.40, -0.90, 1.18], 1)

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

        if self.fixed_motion:
            self.defined_motions()
        else:
            self.spot_inverse_control()

        self.handle_transforms_and_odometry()

        #Update Spot state
        self.__model_cb()
