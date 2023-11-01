import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Time
from webots_spot_msgs.msg import GaitInput
from webots_spot_msgs.srv import SpotMotion, SpotHeight, BlockPose
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from tf2_ros.transform_broadcaster import TransformBroadcaster
from std_srvs.srv import Empty

from scipy.spatial.transform import Rotation as R
import numpy as np
import copy

from webots_spot.SpotKinematics import SpotModel
from webots_spot.Bezier import BezierGait

import os
import random
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import is_wsl

NUMBER_OF_JOINTS = 12
HEIGHT = 0.52  # From spot kinematics

motions = {
    "stand": [-0.1, 0.0, 0.0, 0.1, 0.0, 0.0, -0.1, 0.0, 0.0, 0.1, 0.0, 0.0],
    "sit": [
        -0.20,
        -0.40,
        -0.19,
        0.20,
        -0.40,
        -0.19,
        -0.40,
        -0.90,
        1.18,
        0.40,
        -0.90,
        1.18,
    ],
    "lie": [
        -0.40,
        -0.99,
        1.59,
        0.40,
        -0.99,
        1.59,
        -0.40,
        -0.99,
        1.59,
        0.40,
        -0.99,
        1.59,
    ],
}


def randomise_lane(robot):
    if random.random() < 0.5:  # 50% probability for a rightside lane
        l1 = robot.getFromDef("Lane1")
        l1.getField("translation").setSFVec3f([6.6778, 1.90426, 0.001])
        l1.getField("rotation").setSFRotation([0, 0, -1, -1.04721])
        l2 = robot.getFromDef("Lane2")
        l2.getField("translation").setSFVec3f([3.48651, 2.88433, 0.001])
        l2.getField("rotation").setSFRotation([0, 0, -1, -1.57081])
        l3 = robot.getFromDef("Lane3")
        l3.getField("translation").setSFVec3f([2.02652, 1.67433, 0.001])
        l3.getField("rotation").setSFRotation([0, 0, -1, 0])


def randomise_imgs(robot, hazmat=False):
    if hazmat:
        img_path = os.path.join(
            get_package_share_directory("webots_spot"), "hazmat_signs/"
        )
    else:
        img_path = os.path.join(
            get_package_share_directory("webots_spot"), "yolo_images/"
        )
    all_imgs = os.listdir(img_path)
    three_imgs = random.sample(all_imgs, 3)
    if is_wsl():
        img_path = ".." + img_path
    for idx, img in enumerate(three_imgs):
        robot.getFromDef("Image" + str(idx + 1)).getField("url").setMFString(
            0, img_path + img
        )


def set_rod(robot, red=False):
    pipe = robot.getFromDef("Pipe")
    if red:
        pipe.getField("translation").setSFVec3f([-1.84, 4.01, 0.74])
        pipe.getField("appearance").getSFNode().getField("baseColor").setSFColor(
            [1, 0, 0]
        )
    else:
        pipe.getField("translation").setSFVec3f([-1.84, 4.01, 0.85])
        pipe.getField("appearance").getSFNode().getField("baseColor").setSFColor(
            [1, 1, 0]
        )


def quat_from_aa(aa):
    return R.from_rotvec(aa[3] * np.array(aa[:3])).as_quat()


def diff_quat(q2, q1):
    return (R.from_quat(q2) * R.from_quat(q1).inv()).as_quat()


def shuffle_cubes(robot):
    combinations = []
    for i in range(3):
        for j in range(3):
            combinations.append(
                [-8, round(-4 + i * 0.2, 3), round(0.025 + j * 0.05, 3)]
            )

    locations = random.sample(combinations, 3)
    cube_locations = {}
    for c, location in zip("ABC", locations):
        cube_locations[c] = location

    cube_locations_aphabets = {}
    for cube, location in cube_locations.items():
        on_top_of_cube = False
        for c, loc in cube_locations.items():
            if cube == c:
                continue
            if (
                location[0] == loc[0]
                and location[1] == loc[1]
                and location[2] == round(loc[2] + 0.05, 3)
            ):  # 0.05 becomes 0.050...01
                on_top_of_cube = True
                cube_locations_aphabets[cube] = c
                break
            if (
                location[0] == loc[0]
                and location[1] == loc[1]
                and location[2] == loc[2] + 0.1
            ):
                on_top_of_cube = True
                cube_locations_aphabets[cube] = c
        if not on_top_of_cube:
            if location[1] == -4:
                cube_locations_aphabets[cube] = "t1"
            elif location[1] == -3.8:
                cube_locations_aphabets[cube] = "t2"
            else:
                cube_locations_aphabets[cube] = "t3"
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], 0.025]
            )  # prevent abrupt fall on floor
        else:
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], location[2]]
            )

    return cube_locations_aphabets


class SpotDriver:
    def init(self, webots_node, properties):
        rclpy.init(args=None)

        self.__node = Node("spot_driver")

        self.tfb_ = TransformBroadcaster(self.__node)

        self.__node.get_logger().info("Init SpotDriver")
        self.use_sim_time = True
        if not self.__node.has_parameter("use_sim_time"):
            self.__node.declare_parameter("use_sim_time", self.use_sim_time)
        self.use_sim_time = self.__node.get_parameter("use_sim_time")

        self.__robot = webots_node.robot
        randomise_lane(self.__robot)
        randomise_imgs(self.__robot)
        set_rod(self.__robot)
        self.cubes_loc = shuffle_cubes(self.__robot)

        self.spot_node = self.__robot.getFromDef("Spot")

        self.spot_translation = self.spot_node.getField("translation")
        self.spot_translation_initial = self.spot_translation.getSFVec3f()
        self.spot_rotation = self.spot_node.getField("rotation")
        self.spot_rotation_initial = self.spot_rotation.getSFRotation()

        self.__robot.timestep = 32

        ### Init motors
        self.motor_names = [
            "front left shoulder abduction motor",
            "front left shoulder rotation motor",
            "front left elbow motor",
            "front right shoulder abduction motor",
            "front right shoulder rotation motor",
            "front right elbow motor",
            "rear left shoulder abduction motor",
            "rear left shoulder rotation motor",
            "rear left elbow motor",
            "rear right shoulder abduction motor",
            "rear right shoulder rotation motor",
            "rear right elbow motor",
        ]
        self.motors = []
        for motor_name in self.motor_names:
            self.motors.append(self.__robot.getDevice(motor_name))

        ## Positional Sensors
        self.motor_sensor_names = [
            name.replace("motor", "sensor") for name in self.motor_names
        ]
        self.motor_sensors = []
        self.motors_pos = []
        for idx, sensor_name in enumerate(self.motor_sensor_names):
            self.motor_sensors.append(self.__robot.getDevice(sensor_name))
            self.motor_sensors[idx].enable(self.__robot.timestep)
            self.motors_pos.append(0.0)

        ## Topics
        self.__node.create_subscription(
            GaitInput, "/Spot/inverse_gait_input", self.__gait_cb, 1
        )
        self.__node.create_subscription(Twist, "/cmd_vel", self.__cmd_vel, 1)
        self.joint_state_pub = self.__node.create_publisher(
            JointState, "/joint_states", 1
        )
        self.odom_pub = self.__node.create_publisher(Odometry, "/Spot/odometry", 1)

        ## Services
        self.__node.create_service(SpotMotion, "/Spot/stand_up", self.__stand_motion_cb)
        self.__node.create_service(SpotMotion, "/Spot/sit_down", self.__sit_motion_cb)
        self.__node.create_service(SpotMotion, "/Spot/lie_down", self.__lie_motion_cb)
        self.__node.create_service(
            SpotMotion, "/Spot/shake_hand", self.__shakehand_motion_cb
        )
        self.__node.create_service(
            SpotHeight, "/Spot/set_height", self.__spot_height_cb
        )

        self.__node.create_service(
            SpotMotion, "/Spot/blocksworld_pose", self.blocksworld_pose
        )
        self.__node.create_service(Empty, "/hazmat_signs", self.hazmat_signs)
        self.__node.create_service(Empty, "/red_rod", self.red_rod)
        self.__node.create_service(BlockPose, "/get_block_pose", self.gpp_block_pose)

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
        self.x_inst = 0.0
        self.y_inst = 0.0
        self.z_inst = 0.0
        self.roll_inst = 0.0
        self.pitch_inst = 0.0
        self.yaw_inst = 0.0
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
        self.step_difference = [
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ]
        self.m_target = []
        self.paw = False
        self.paw2 = False
        self.paw_time = 0.0
        self.previous_cmd = False

    def __model_cb(self):
        spot_rot = self.spot_node.getField("rotation")
        spot_rot_val = spot_rot.getSFRotation()
        self.yaw_inst = spot_rot_val[2]

    def yaw_control(self):
        """Yaw body controller"""
        yaw_target = self.YawControl
        thr = np.pi / 2
        if (yaw_target > thr and self.yaw_inst < -thr) or (
            self.yaw_inst > thr and yaw_target < -thr
        ):
            residual = (yaw_target - self.yaw_inst) * np.sign(
                yaw_target - self.yaw_inst
            ) - 2 * np.pi
            yawrate_d = 2.0 * np.sqrt(abs(residual)) * np.sign(residual)
        else:
            residual = yaw_target - self.yaw_inst
            yawrate_d = 4.0 * np.sqrt(abs(residual)) * np.sign(residual)
        return yawrate_d

    def __cmd_vel(self, msg):
        # Override motion command
        self.fixed_motion = False

        if not self.__node.count_publishers("/Spot/inverse_gait_input"):
            StepLength = 0.15
            ClearanceHeight = 0.015
            PenetrationDepth = 0.003
            SwingPeriod = 0.3
            YawControl = 0.0
            YawControlOn = 0.0
            StepVelocity = 0.8

            self.xd = 0.0
            self.yd = 0.0
            # self.zd = 0.
            self.rolld = 0.0
            self.pitchd = 0.0
            self.yawd = 0.0
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
        motor_offsets = [0, 0.52, -1.182]
        for idx, motor in enumerate(self.motors):
            motor.setPosition(motor_offsets[idx % 3] + motors_target_pos[idx])

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
            self.rear_right_lower_leg_contact,
        ]

        # Get Desired Foot Poses
        T_bf = self.bzg.GenerateTrajectory(
            self.StepLength,
            self.LateralFraction,
            YawRate_desired,
            self.StepVelocity,
            self.T_bf0,
            self.T_bf,
            self.ClearanceHeight,
            self.PenetrationDepth,
            contacts,
        )
        joint_angles = -self.spot.IK(orn, pos, T_bf)

        target = [
            joint_angles[0][0],
            joint_angles[0][1],
            joint_angles[0][2],
            joint_angles[1][0],
            joint_angles[1][1],
            joint_angles[1][2],
            joint_angles[2][0],
            joint_angles[2][1],
            joint_angles[2][2],
            joint_angles[3][0],
            joint_angles[3][1],
            joint_angles[3][2],
        ]

        self.__talker(target)

    def handle_transforms_and_odometry(self):
        for idx, motor_sensor in enumerate(self.motor_sensors):
            self.motors_pos[idx] = motor_sensor.getValue()

        if not self.use_sim_time:
            time_stamp = self.__node.get_clock().now().to_msg()
        else:
            current_time = self.__robot.getTime()
            time_stamp = Time()
            time_stamp.sec = int(current_time)
            time_stamp.nanosec = int((current_time % 1) * 1e9)

        base_link_from_ground = HEIGHT - self.zd

        ## Odom to following:
        tfs = []
        for x in [
            "Spot",
            "A",
            "B",
            "C",
            "T1",
            "T2",
            "T3",
            "P",
            "Image1",
            "Image2",
            "Image3",
            "PlaceBox",
        ]:
            tf = TransformStamped()
            tf.header.stamp = time_stamp
            tf.header.frame_id = "odom"
            tf._child_frame_id = x if x != "Spot" else "base_link"

            part = self.__robot.getFromDef(x)
            di = part.getField("translation").getSFVec3f()
            tf.transform.translation.x = -(di[0] - self.spot_translation_initial[0])
            tf.transform.translation.y = -(di[1] - self.spot_translation_initial[1])
            tf.transform.translation.z = di[2] - self.spot_translation_initial[2]
            tf.transform.translation.z += HEIGHT + 0.095  # BASE_LINK To Ground at Rest

            r = diff_quat(
                quat_from_aa(part.getField("rotation").getSFRotation()),
                quat_from_aa(self.spot_rotation_initial),
            )
            tf.transform.rotation.x = -r[0]
            tf.transform.rotation.y = -r[1]
            tf.transform.rotation.z = r[2]
            tf.transform.rotation.w = r[3]
            tfs.append(tf)

        ## base_footprint
        tf = TransformStamped()
        tf.header.stamp = time_stamp
        tf.header.frame_id = "base_link"
        tf._child_frame_id = "base_footprint"
        tf.transform.translation.z = -base_link_from_ground
        tfs.append(tf)

        self.tfb_.sendTransform(tfs)

        ## /Spot/odometry
        tf_odom_base_link = tfs[0].transform
        translation = [
            tf_odom_base_link.translation.x,
            tf_odom_base_link.translation.y,
            tf_odom_base_link.translation.z,
        ]

        r = R.from_quat(
            [
                tf_odom_base_link.rotation.x,
                tf_odom_base_link.rotation.y,
                tf_odom_base_link.rotation.z,
                tf_odom_base_link.rotation.w,
            ]
        )
        rotation = [r.as_euler("xyz")[0], r.as_euler("xyz")[1], r.as_euler("xyz")[2]]

        if not hasattr(self, "previous_rotation"):
            self.previous_rotation = rotation
            self.previous_translation = translation
        else:
            rotation_twist = [
                (new - old) / (self.time_step / 1000)
                for new, old in zip(rotation, self.previous_rotation)
            ]
            translation_twist = [
                (new - old) / (self.time_step / 1000)
                for new, old in zip(translation, self.previous_translation)
            ]

            self.previous_time = current_time
            self.previous_rotation = rotation
            self.previous_translation = translation

            odom = Odometry()
            odom.header.frame_id = "odom"
            odom.header.stamp = time_stamp
            odom.child_frame_id = "base_link"
            odom.pose.pose.position.x = translation[0]
            odom.pose.pose.position.y = translation[1]
            odom.pose.pose.position.z = translation[2]
            r = R.from_euler("xyz", [rotation[0], rotation[1], rotation[2]])
            odom.pose.pose.orientation.x = r.as_quat()[0]
            odom.pose.pose.orientation.y = r.as_quat()[1]
            odom.pose.pose.orientation.z = r.as_quat()[2]
            odom.pose.pose.orientation.w = r.as_quat()[3]
            odom.twist.twist.linear.x = translation_twist[0]
            odom.twist.twist.linear.y = translation_twist[1]
            odom.twist.twist.linear.z = translation_twist[2]
            odom.twist.twist.angular.x = rotation_twist[0]
            odom.twist.twist.angular.y = rotation_twist[1]
            odom.twist.twist.angular.z = rotation_twist[2]
            self.odom_pub.publish(odom)

        unactuated_joints = [
            "front left piston motor",
            "front right piston motor",
            "rear left piston motor",
            "rear right piston motor",
        ]

        joint_state = JointState()
        joint_state.header.stamp = time_stamp
        joint_state.name = []
        joint_state.name.extend(self.motor_names)
        joint_state.name.extend(unactuated_joints)
        joint_state.position = []
        joint_state.position.extend(self.motors_pos)
        joint_state.position.extend([0.0 for _ in unactuated_joints])
        qty = +len(self.motor_names) + len(unactuated_joints)
        joint_state.velocity = [0.0 for _ in range(qty)]
        joint_state.effort = [0.0 for _ in range(qty)]
        self.joint_state_pub.publish(joint_state)

    def movement_decomposition(self, target, duration):
        """
        Decompose big motion into smaller motions
        """
        self.previous_cmd = True

        self.n_steps_to_achieve_target = duration * 1000 / self.__robot.timestep
        self.step_difference = [
            (target[i] - self.motors_pos[i]) / self.n_steps_to_achieve_target
            for i in range(NUMBER_OF_JOINTS)
        ]
        self.m_target = []

    def blocksworld_pose(self, request, response):
        self.spot_node.getField("translation").setSFVec3f([-7.28, -3.78, 0.081])
        self.spot_node.getField("rotation").setSFRotation([0, 0, -1, -3.14159])

        self.fixed_motion = True

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions["lie"], 1)
        response.answer = "lying down"

        return response

    def hazmat_signs(self, request, response):
        randomise_imgs(self.__robot, True)
        return response

    def red_rod(self, request, response):
        set_rod(self.__robot, True)
        return response

    def gpp_block_pose(self, request, response):
        response.location = self.cubes_loc[request.block.upper()].lower()
        return response

    def __stand_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = "performing previous command, override with bool argument"
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions["stand"], 1)
        response.answer = "standing up"
        return response

    def __sit_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = "performing previous command, override with bool argument"
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions["sit"], 1)
        response.answer = "sitting down"
        return response

    def __lie_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = "performing previous command, override with bool argument"
            return response

        self.paw = False
        self.paw2 = False
        self.movement_decomposition(motions["lie"], 1)
        response.answer = "lying down"
        return response

    def __shakehand_motion_cb(self, request, response):
        self.fixed_motion = True
        if self.previous_cmd and not request.override:
            response.answer = "performing previous command, override with bool argument"
            return response

        # Start handshake motion
        self.movement_decomposition(
            [
                -0.20,
                -0.30,
                0.05,
                0.20,
                -0.40,
                -0.19,
                -0.40,
                -0.90,
                1.18,
                0.49,
                -0.90,
                0.80,
            ],
            1,
        )
        self.paw = True
        response.answer = "shaking hands"
        return response

    def defined_motions(self):
        self.handle_transforms_and_odometry()  # Let the sensor values get updated
        if self.n_steps_to_achieve_target > 0:
            if not self.m_target:
                self.m_target = [
                    self.step_difference[i] + self.motors_pos[i]
                    for i in range(NUMBER_OF_JOINTS)
                ]
            else:  # if compared to current motors_positions, the final motion is smaller
                self.m_target = [
                    self.step_difference[i] + self.m_target[i]
                    for i in range(NUMBER_OF_JOINTS)
                ]

            # Increment motor positions by step_difference
            for idx, motor in enumerate(self.motors):
                motor.setPosition(self.m_target[idx])

            self.n_steps_to_achieve_target -= 1
        else:
            if self.paw:
                self.paw_time = self.__robot.getTime() + 4
                self.paw = False
                self.paw2 = True  # Do the shakehands motion
            else:
                self.previous_cmd = False

            self.m_target = []

        if self.paw2:
            self.previous_cmd = True
            if self.paw_time > self.__robot.getTime():
                self.motors[4].setPosition(
                    0.2 * np.sin(2 * self.__robot.getTime()) + 0.6
                )
                self.motors[5].setPosition(0.4 * np.sin(2 * self.__robot.getTime()))
            else:
                self.paw2 = False  # Sit back again
                self.movement_decomposition(
                    [
                        -0.20,
                        -0.40,
                        -0.19,
                        0.20,
                        -0.40,
                        -0.19,
                        -0.40,
                        -0.90,
                        1.18,
                        0.40,
                        -0.90,
                        1.18,
                    ],
                    1,
                )

    def __spot_height_cb(self, request, response):
        if not -0.2 <= request.height <= 0.2:
            response.answer = "set height within -0.2 and 0.2"
            return response
        self.zd = -request.height
        return response

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)

        self.front_left_lower_leg_contact = self.touch_fl.getValue()
        self.front_right_lower_leg_contact = self.touch_fr.getValue()
        self.rear_left_lower_leg_contact = self.touch_rl.getValue()
        self.rear_right_lower_leg_contact = self.touch_rr.getValue()

        if self.fixed_motion:
            self.defined_motions()
        else:
            self.spot_inverse_control()

        self.handle_transforms_and_odometry()

        # Update Spot state
        self.__model_cb()
