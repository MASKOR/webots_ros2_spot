from ctypes.wintypes import INT
from matplotlib.pyplot import step
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from spot_msg_interface.msg import Legs


#from controller import Supervisor

import sys
import numpy as np
import math
import time


NUMBER_OF_JOINTS = 12

class SpotDriver:
    def init(self, webots_node, properties):
        print('Hello Init')
        self.__robot = webots_node.robot
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
        self.__node.create_subscription(Legs, 'Spot/talker', self.__motor_cb, 2)
        self.__node.create_subscription(Bool, 'Spot/give_paw',self.__give_paw_cb, 1)

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

        self.__node.get_logger().info("name: "+ str(self.touch_fr.getName()))
        self.__node.get_logger().info("name: "+ str(self.touch_fl.getName()))
        self.__node.get_logger().info("name: "+ str(self.touch_fr.getName()))
        self.__node.get_logger().info("name: "+ str(self.touch_fl.getName()))

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
            self.__robot.step(32)
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
            self.__robot.step(32)
            self.step()

    def step(self):
        msg_fl = Bool()
        msg_fr = Bool()
        msg_rl = Bool()
        msg_rr = Bool()
        msg_fl.data = bool(self.touch_fl.getValue())
        msg_fr.data = bool(self.touch_fr.getValue())
        msg_rl.data = bool(self.touch_rl.getValue())
        msg_rr.data = bool(self.touch_rr.getValue())
        #self.__node.get_logger().info("Touch fr: " + str(self.touch_fr.getValue()))
        self.__touch_fl_pub.publish(msg_fl)
        self.__touch_fr_pub.publish(msg_fr)
        self.__touch_rl_pub.publish(msg_rl)
        self.__touch_rr_pub.publish(msg_rr)
        rclpy.spin_once(self.__node, timeout_sec=0)