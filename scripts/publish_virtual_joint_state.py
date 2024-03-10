import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStatePublisher(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher_ = self.create_publisher(JointState, 'joint_states', 1)
        self.timer_ = self.create_timer(1.0, self.publish_joint_states)
        self.joint_state_msg = JointState()
        self.joint_state_msg.name = ['virtual_base_link_base_link']
        self.joint_state_msg.position = [0.0]

    def publish_joint_states(self):
        self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(self.joint_state_msg)

def main(args=None):
    rclpy.init(args=args)
    joint_state_publisher = JointStatePublisher()
    rclpy.spin(joint_state_publisher)
    joint_state_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
