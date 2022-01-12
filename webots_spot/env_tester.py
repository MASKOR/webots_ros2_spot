import rclpy
import time
from rclpy.node import Node
from spot_msg_interface.msg import Legs

# ------------------ Standard pose for starting step
sit_down = [-0.40, -0.99, 1.59,
            0.40,  -0.99, 1.59,
            -0.40, -0.99, 1.59,
            0.40,  -0.99, 1.59]  # Rear right leg

stand_up = [0.20, 0.7, -1.39,  # Front left leg
            -0.20, 0.7, -1.39,  # Front right leg
            0.20, 0.7, -1.39,  # Rear left leg
            -0.20, 0.7, -1.39]  # Rear right leg



class EnvTester(Node):
    def __init__(self):
        super().__init__('env_tester')
        self.talker_pub = self.create_publisher(Legs, 'spot/talker', 1)

def main(args=None):
    rclpy.init(args=args)
    env_tester = EnvTester()

    motor_cmd = Legs()
    #motor_cmd.leg = [0.40, -0.99, 1.59, -0.40, -0.99, 1.59, 0.40, -0.99, 1.59, -0.40, -0.99, 1.59]
    motor_cmd.leg = [-0.1, 0.0, 0.0, 0.1, 0.0, 0.0, -0.1, 0.0, 0.0, 0.1, 0.0, 0.0]
    env_tester.talker_pub.publish(motor_cmd)
    env_tester.get_logger().info(str(motor_cmd))

    rclpy.spin(env_tester)
    rclpy.shutdown()


if __name__ == '__main__':
    main()