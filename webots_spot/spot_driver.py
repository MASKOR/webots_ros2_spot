import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor

from webots_spot_msgs.msg import GaitInput
from webots_spot_msgs.srv import SpotMotion, SpotHeight
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from tf2_ros.transform_broadcaster import TransformBroadcaster

from scipy.spatial.transform import Rotation as R
import numpy as np
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

import os
import random
from ament_index_python.packages import get_package_share_directory

NUMBER_OF_JOINTS = 12

motions = {
    'stand': [-0.1, 0.0, 0.0, 0.1, 0.0, 0.0, -0.1, 0.0, 0.0, 0.1, 0.0, 0.0],
    'sit'  : [-0.20, -0.40, -0.19, 0.20, -0.40, -0.19, -0.40, -0.90, 1.18, 0.40, -0.90, 1.18],
    'lie'  : [-0.40, -0.99, 1.59, 0.40, -0.99, 1.59, -0.40, -0.99, 1.59, 0.40, -0.99, 1.59],
}


def randomise_lane(robot):
    if random.random() < 0.5: # 50% probability for a rightside lane
        l1 = robot.getFromDef("Lane1")
        l1.getField('translation').setSFVec3f([6.6778,1.90426,0.001])
        l1.getField('rotation').setSFRotation([0,0,-1,-1.04721])
        l2 = robot.getFromDef("Lane2")
        l2.getField('translation').setSFVec3f([3.48651,2.88433,0.001])
        l2.getField('rotation').setSFRotation([0,0,-1,-1.57081])
        l3 = robot.getFromDef("Lane3")
        l3.getField('translation').setSFVec3f([2.02652,1.67433,0.001])
        l3.getField('rotation').setSFRotation([0,0,-1,0])


def randomise_imgs(robot):
    img_path = os.path.join(get_package_share_directory('webots_spot'), 'yolo_images/')
    all_imgs = os.listdir(img_path)
    three_imgs = random.sample(all_imgs, 3)
    for idx, img in enumerate(three_imgs):
        robot.getFromDef("Image" + str(idx+1)).getField('url').setMFString(0, img_path + img)


def quat_from_aa(aa):
    return R.from_rotvec(aa[3] * np.array(aa[:3])).as_quat()


def diff_quat(q2, q1):
    return (R.from_quat(q2) * R.from_quat(q1).inv()).as_quat()


def retract_arm(robot):
    # Retracted arm initially
    robot.getDevice("spotarm_2_joint").setPosition(3.1415)
    robot.getDevice("spotarm_3_joint").setPosition(3.0)
    robot.getDevice("spotarm_5_joint").setPosition(np.deg2rad(11))


class SpotDriver:
    def init(self, webots_node, properties):
        self.__robot = webots_node.robot
        randomise_lane(self.__robot)
        randomise_imgs(self.__robot)
        retract_arm(self.__robot)
        self.spot_node = self.__robot.getFromDef("Spot")

        self.spot_translation = self.spot_node.getField('translation')
        self.spot_translation_initial = self.spot_translation.getSFVec3f()
        self.spot_rotation = self.spot_node.getField('rotation')
        self.spot_rotation_initial = self.spot_rotation.getSFRotation()

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
        
        ### Init ur3e motors
        self.ur3e_motors=[]
        self.ur3e_motor_names = [
            'Slider11',
            'spotarm_1_joint',
            'spotarm_2_joint',
            'spotarm_3_joint',
            'spotarm_4_joint',
            'spotarm_5_joint',
            'spotarm_6_joint'
        ]
        for motor_name in self.ur3e_motor_names:
            self.ur3e_motors.append(self.__robot.getDevice(motor_name))
        self.ur3e_sensors = []
        self.ur3e_pos = []
        for idx, sensor_name in enumerate(self.ur3e_motor_names):
            self.ur3e_sensors.append(
                self.__robot.getDevice(sensor_name + '_sensor')
            )
            self.ur3e_sensors[idx].enable(self.__robot.timestep)
            self.ur3e_pos.append(0.)

        ### Init gripper motors
        self.gripper_motors=[]
        self.gripper_sensors = []
        self.gripper_pos = []
        self.gripper_motor_names = [
            'gripper_left_finger_joint',
            'gripper_right_finger_joint'
        ]
        for idx, motor_name in enumerate(self.gripper_motor_names):
            self.gripper_motors.append(self.__robot.getDevice(motor_name))
            self.gripper_sensors.append(self.__robot.getDevice(motor_name + '_sensor'))  
            self.gripper_sensors[idx].enable(self.__robot.timestep)
            self.gripper_pos.append(0.)

        self.remaining_gripper_sensors = []
        self.remaining_gripper_pos = []
        self.remaining_gripper_motor_names = [
        ]
        for idx, motor_name in enumerate(self.remaining_gripper_motor_names):
            self.remaining_gripper_sensors.append(self.__robot.getDevice(motor_name + '_sensor'))  
            self.remaining_gripper_sensors[idx].enable(self.__robot.timestep)
            self.remaining_gripper_pos.append(0.)

        ## Positional Sensors
        self.motor_sensor_names = [name.replace('motor', 'sensor') for name in self.motor_names]
        self.motor_sensors = []
        self.motors_pos = []
        for idx, sensor_name in enumerate(self.motor_sensor_names):
            self.motor_sensors.append(self.__robot.getDevice(sensor_name))
            self.motor_sensors[idx].enable(self.__robot.timestep)
            self.motors_pos.append(0.)

        rclpy.init(args=None)

        self.__node = Node('spot_driver')
        self.__moveit_node = Node('spot_moveit')

        self.tfb_ = TransformBroadcaster(self.__node)        

        self.__node.get_logger().info('Init SpotDriver')

        ## Topics
        self.__node.create_subscription(GaitInput, '/Spot/inverse_gait_input', self.__gait_cb, 1)
        self.__node.create_subscription(Twist, '/cmd_vel', self.__cmd_vel, 1)
        self.joint_state_pub = self.__node.create_publisher(JointState, '/joint_states', 1)

        ## Services
        self.__node.create_service(SpotMotion, '/Spot/stand_up', self.__stand_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/sit_down', self.__sit_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/lie_down', self.__lie_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/shake_hand', self.__shakehand_motion_cb)
        self.__node.create_service(SpotMotion, '/Spot/close_gripper', self.__close_gripper_cb)
        self.__node.create_service(SpotMotion, '/Spot/open_gripper', self.__open_gripper_cb)
        self.__node.create_service(SpotHeight, '/Spot/set_height', self.__spot_height_cb)

        ## ActionServer
        self._action_server = ActionServer(
            self.__moveit_node,
            # self.__node,
            FollowJointTrajectory,
            'ur_joint_trajectory_controller/follow_joint_trajectory',
            self.__moveit_cb)

        # # Set up mulithreading
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(self.__node)
        self.executor.add_node(self.__moveit_node)

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

        # UR3e and Gripper
        self.u3re_n_steps_to_achieve_target = 0
        self.ur3e_targets = []
        self.ur3e_target = []
        self.allow_new_target = False
        self.gripper_close = False

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
            # self.zd = 0.
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

    def handle_transforms_and_odometry(self):
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()

        time_stamp = self.__node.get_clock().now().to_msg()

        for idx, ur3e_sensor in enumerate(self.ur3e_sensors):
            self.ur3e_pos[idx] = ur3e_sensor.getValue()

        for idx, gripper_sensor in enumerate(self.gripper_sensors):
            self.gripper_pos[idx] = gripper_sensor.getValue()

        for idx, gripper_sensor in enumerate(self.remaining_gripper_sensors):
            self.remaining_gripper_pos[idx] = gripper_sensor.getValue()

        ## Odom to "base_link", "A", "B", "C", "P", "Image1", "Image2", "Image3", "PlaceBox"
        tfs = []
        for x in ["Spot", "A", "B", "C", "P", "Image1", "Image2", "Image3", "PlaceBox"]:
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id= "odom"
            tf._child_frame_id = x if x != "Spot" else "base_link"
            
            part = self.__robot.getFromDef(x)
            di = part.getField('translation').getSFVec3f()
            tf.transform.translation.x = -(di[0] - self.spot_translation_initial[0])
            tf.transform.translation.y = -(di[1] - self.spot_translation_initial[1])
            tf.transform.translation.z = di[2] - self.spot_translation_initial[2]
            
            r = diff_quat(quat_from_aa(part.getField('rotation').getSFRotation()), quat_from_aa(self.spot_rotation_initial))
            tf.transform.rotation.x = -r[0]
            tf.transform.rotation.y = -r[1]
            tf.transform.rotation.z = r[2]
            tf.transform.rotation.w = r[3]
            tfs.append(tf)
        self.tfb_.sendTransform(tfs)
        
        unactuated_joints = ['front left piston motor', 'front right piston motor', 'rear left piston motor', 'rear right piston motor']

        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.ur3e_motor_names)
        joint_state.name.extend(self.gripper_motor_names)
        joint_state.name.extend(self.remaining_gripper_motor_names)
        joint_state.name.extend(self.motor_names)
        joint_state.name.extend(unactuated_joints)
        joint_state.position = []
        joint_state.position.extend(self.ur3e_pos)
        joint_state.position.extend(self.gripper_pos)
        joint_state.position.extend(self.remaining_gripper_pos)
        joint_state.position.extend(self.motors_pos)
        joint_state.position.extend([0. for _ in unactuated_joints])
        qty = (
              len(self.ur3e_motor_names)
            + len(self.gripper_motor_names)
            + len(self.remaining_gripper_motor_names)
            + len(self.motor_names)
            + len(unactuated_joints)
            )
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
        self.handle_transforms_and_odometry() # Let the sensor values get updated
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

    def __close_gripper_cb(self, request, response):
        if self.gripper_close:
            response.answer = 'gripper already close'
            return response
        response.answer = 'closing gripper'
        self.gripper_close = True
        return response

    def __open_gripper_cb(self, request, response):
        if not self.gripper_close:
            response.answer = 'gripper already open'
            return response
        response.answer = 'opening gripper'
        self.gripper_close = False
        return response

    def __spot_height_cb(self, request, response):
        if not -0.3 <= request.height <= 0:
            response.answer = 'set height within -0.3 and 0'
            return response
        self.zd = -request.height
        return response

    def __ur3e_defined_motions(self):
        if self.u3re_n_steps_to_achieve_target > 0:
            if not self.ur3e_target:
                self.ur3e_target = [self.ur3e_step_difference[i] + self.ur3e_pos[i] for i in range(len(self.ur3e_motor_names))]
            else: # if compared to current motors_positions, the final motion is smaller
                self.ur3e_target = [self.ur3e_step_difference[i] + self.ur3e_target[i] for i in range(len(self.ur3e_motor_names))]

            # Increment motor positions by step_difference
            for idx, motor in enumerate(self.ur3e_motors):
                motor.setPosition(self.ur3e_target[idx])
            
            self.u3re_n_steps_to_achieve_target -= 1
        elif len(self.ur3e_targets) > 0:
            # Forcefully go to target, moveit2 expect a deviation of max 0.01
            for idx, motor in enumerate(self.ur3e_motors):
                motor.setPosition(self.ur3e_targets[0][idx])
            next_target = True
            for position, target in zip(self.ur3e_pos, self.ur3e_targets[0]):
                if abs(position - target) > 0.01:
                    next_target = False
                    break
            if next_target:
                self.ur3e_targets.pop(0)
        else:
            self.allow_new_target = True

    def __moveit_movement_decomposition(self, duration):
        """
        Decompose big motion into smaller motions
        """
        if len(self.ur3e_targets) > 0 and self.allow_new_target:
            self.allow_new_target = False
            target = self.ur3e_targets[0]
            self.u3re_n_steps_to_achieve_target = duration * 1000 / self.__robot.timestep
            self.ur3e_step_difference = [(target[i] - self.ur3e_pos[i]) / self.u3re_n_steps_to_achieve_target
                                    for i in range(7)]

    def __moveit_cb(self, goal_handle):
        self.__moveit_node.get_logger().info('Executing goal...')
        
        self.ur3e_targets = []
        for p in goal_handle.request.trajectory.points:
            seq = list(p.positions)
            self.ur3e_targets.append(seq)
        
        while len(self.ur3e_targets) > 0:
            if self.allow_new_target and len(self.ur3e_targets) > 0:
                self.__moveit_movement_decomposition(0.4)
            self.__ur3e_defined_motions()

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        return result

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
        self.executor.spin_once(timeout_sec=0)

        self.callback_front_left_lower_leg_contact(bool(self.touch_fl.getValue()))
        self.callback_front_right_lower_leg_contact(bool(self.touch_fr.getValue()))
        self.callback_rear_left_lower_leg_contact(bool(self.touch_rl.getValue()))
        self.callback_rear_right_lower_leg_contact(bool(self.touch_rr.getValue()))

        if self.fixed_motion:
            self.defined_motions()
        else:
            self.spot_inverse_control()

        self.handle_transforms_and_odometry()

        if self.gripper_close:
            for idx, motor in enumerate(self.gripper_motors):
                motor.setPosition([0.018, 0.018][idx])
        else:
            for idx, motor in enumerate(self.gripper_motors):
                motor.setPosition([0.045, 0.045][idx])

        #Update Spot state
        self.__model_cb()
