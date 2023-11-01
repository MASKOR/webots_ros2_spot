import rclpy
from rclpy.node import Node
from spot_msgs.srv import SpotMotion


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.cli = self.create_client(SpotMotion, "/Spot/lie_down")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
        self.req = SpotMotion.Request()

    def send_request(self):
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    minimal_client.send_request()

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
