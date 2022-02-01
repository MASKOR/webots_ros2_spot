import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from spot_msg_interface.msg import Legs


#from controller import Supervisor

import sys
import numpy as np
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
        
        self.__node.create_subscription(Legs, 'spot/talker', self.__motor_cb, 2)

        n_devices = self.__robot.getNumberOfDevices()
        self.__node.get_logger().info('Number of Devices'+str(n_devices))
        while index < n_devices:
            self.__node.get_logger().info("index: "+ str(index)+" "+ str(self.__robot.getDeviceByIndex(index)))
            index += 1

        ### Touch Sensors
        self.touch_fl = self.__robot.getDevice("touch_fl")
        self.touch_fr = self.__robot.getDevice("touch_fr")
        self.touch_fl.enable(self.__robot.timestep)
        self.touch_fr.enable(self.__robot.timestep)
        self.__node.get_logger().info("name: "+ str(self.touch_fr.getName()))
        self.__node.get_logger().info("name: "+ str(self.touch_fl.getName()))

    def __motor_cb(self, msg):
        self.__node.get_logger().info("Talker")
        self.movement_decomposition(msg.leg, 1.0)

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

    def step(self):
        rclpy.spin_once(self.__node, timeout_sec=0)
        #self.__node.get_logger().info('Touch FL:'+str(self.touch_fl.getValue()))