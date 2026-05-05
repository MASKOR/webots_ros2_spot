#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog

import sys
import tty
import termios
import threading
import signal

# Constants
TWIST_TOPIC = "/servo_node/delta_twist_cmds"
JOINT_TOPIC = "/servo_node/delta_joint_cmds"
EEF_FRAME_ID = "end_effector_link"
BASE_FRAME_ID = "gen3_base_link"

KEY_BINDINGS = {
    "\x1b[A": ("twist", "x", 1.0),  # UP arrow
    "\x1b[B": ("twist", "x", -1.0),  # DOWN arrow
    "\x1b[C": ("twist", "y", -1.0),  # RIGHT arrow
    "\x1b[D": ("twist", "y", 1.0),  # LEFT arrow
    ".": ("twist", "z", -1.0),
    ";": ("twist", "z", 1.0),
    "w": ("twist_angular", "y", -1.0),
    "s": ("twist_angular", "y", 1.0),
    "a": ("twist_angular", "z", 1.0),
    "d": ("twist_angular", "z", -1.0),
    "z": ("twist_angular", "x", -1.0),
    "x": ("twist_angular", "x", 1.0),
    "1": ("joint", "joint_1"),
    "2": ("joint", "joint_2"),
    "3": ("joint", "joint_3"),
    "4": ("joint", "joint_4"),
    "5": ("joint", "joint_5"),
    "6": ("joint", "joint_6"),
    "b": ("frame", BASE_FRAME_ID),
    "e": ("frame", EEF_FRAME_ID),
    "r": ("reverse", None),
    "q": ("quit", None),
}


class ServoKeyboardInput(Node):
    def __init__(self):
        super().__init__("servo_keyboard_input")
        self.twist_pub = self.create_publisher(TwistStamped, TWIST_TOPIC, 10)
        self.joint_pub = self.create_publisher(JointJog, JOINT_TOPIC, 10)

        self.frame_to_publish = BASE_FRAME_ID
        self.joint_vel_cmd = 1.0

        self.running = True
        signal.signal(signal.SIGINT, self.shutdown_handler)

    def shutdown_handler(self, signum, frame):
        self.running = False
        rclpy.shutdown()

    def read_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
            if key == "\x1b":
                key += sys.stdin.read(2)
            return key
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def key_loop(self):
        print(
            """
Reading from keyboard:
---------------------------
Use arrow keys and the '.' and ';' keys to Cartesian jog
Use 'W' to Cartesian jog in the world frame, and 'E' for the End-Effector frame
Use 1|2|3|4|5|6|7 keys to joint jog. 'R' to reverse the direction of jogging.
'Q' to quit.
"""
        )
        while self.running:
            key = self.read_key()
            if key in KEY_BINDINGS:
                binding = KEY_BINDINGS[key]
                action = binding[0]

                if len(binding) > 1:
                    value = binding[1]
                else:
                    value = None

                if action == "twist":
                    msg = TwistStamped()
                    msg.twist.linear.x = 0.0
                    msg.twist.linear.y = 0.0
                    msg.twist.linear.z = 0.0
                    if value in ["x", "y", "z"]:
                        setattr(
                            msg.twist.linear, value, binding[2]
                        )  # Use the third element in the tuple for the value
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_to_publish
                    self.twist_pub.publish(msg)
                elif action == "twist_angular":
                    msg = TwistStamped()
                    msg.twist.angular.x = 0.0
                    msg.twist.angular.y = 0.0
                    msg.twist.angular.z = 0.0
                    if value in ["x", "y", "z"]:
                        setattr(msg.twist.angular, value, binding[2])
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = self.frame_to_publish
                    self.twist_pub.publish(msg)

                elif action == "joint":
                    msg = JointJog()
                    msg.joint_names = [value]
                    msg.velocities = [self.joint_vel_cmd]
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = BASE_FRAME_ID
                    self.joint_pub.publish(msg)

                elif action == "frame":
                    self.frame_to_publish = value
                    self.get_logger().info(f"Switched frame to {value}")

                elif action == "reverse":
                    self.joint_vel_cmd *= -1
                    self.get_logger().info(
                        f"Reversed joint velocity: {self.joint_vel_cmd}"
                    )

                elif action == "quit":
                    self.get_logger().info("Exiting keyboard teleop.")
                    break
            else:
                self.get_logger().debug(f"Unhandled key: {repr(key)}")


def main(args=None):
    rclpy.init(args=args)
    node = ServoKeyboardInput()

    try:
        node.key_loop()
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()