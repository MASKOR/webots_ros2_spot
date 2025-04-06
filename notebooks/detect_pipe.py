import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class DetectPipe(Node):
    def __init__(self):
        super().__init__("detect_pipe")

        self.color_subscription = self.create_subscription(
            Image, "/kinova_color", self.image_callback, 10
        )

        self.depth_subscription = self.create_subscription(
            Image, "/kinova_depth", self.depth_callback, 10
        )

        self.bridge = CvBridge()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.circles = None

        # Color detection parameters
        self.color_hough_dp = 1.0
        self.color_hough_min_dist = 20
        self.color_hough_param1 = 80
        self.color_hough_param2 = 50
        self.color_min_radius = 20
        self.color_max_radius = 50

        # Based on topic echo
        self.depth_width = 480
        self.depth_height = 270
        self.color_width = 640
        self.color_height = 480
        self.scale_x = self.depth_width / self.color_width
        self.scale_y = self.depth_height / self.color_height

        # Offset adjustments (tune these manually)
        self.offset_x = -20  # Adjust based on  shift
        self.offset_y = 0  # Adjust if vertical shift is noticed

        self.get_logger().info("DetectPipe node has been started.")

    def image_callback(self, msg):
        try:
            self.latest_color_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = self.latest_color_frame.copy()

            # Debug
            self.get_logger().debug(f"Color image shape: {frame.shape}")

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.GaussianBlur(gray, (5, 5), 0)

            self.circles = cv2.HoughCircles(
                gray,
                cv2.HOUGH_GRADIENT,
                dp=self.color_hough_dp,
                minDist=self.color_hough_min_dist,
                param1=self.color_hough_param1,
                param2=self.color_hough_param2,
                minRadius=self.color_min_radius,
                maxRadius=self.color_max_radius,
            )

            # Draw circles and get depth
            if self.circles is not None:
                self.circles = np.round(self.circles[0, :]).astype("int")
                for x, y, r in self.circles:
                    # Draw on color image (using original radius)
                    cv2.circle(frame, (x, y), r, (0, 255, 0), 4)
                    cv2.rectangle(
                        frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1
                    )

                    # Scale to depth coordinates
                    x_scaled = int(x * self.scale_x) + self.offset_x
                    y_scaled = int(y * self.scale_y) + self.offset_y
                    r_scaled = int(r * self.scale_x)  # Scale radius

                    # Check depth if available
                    if self.latest_depth_frame is not None:
                        if (
                            0 <= y_scaled < self.depth_height
                            and 0 <= x_scaled < self.depth_width
                        ):
                            depth_value = self.latest_depth_frame[y_scaled, x_scaled]
                            self.get_logger().info(
                                f"Circle at ({x}, {y}) [scaled: ({x_scaled}, {y_scaled})], "
                                f"radius {r} [scaled: {r_scaled}], depth: {depth_value}"
                            )
                        else:
                            self.get_logger().warn(
                                f"Scaled circle at ({x_scaled}, {y_scaled}) with radius {r_scaled} "
                                f"out of depth bounds ({self.depth_width}x{self.depth_height})"
                            )
                    else:
                        self.get_logger().info(
                            f"Circle at ({x}, {y}) with radius {r} (no depth yet)"
                        )
            else:
                self.get_logger().debug("No circles detected in color image")

            cv2.imshow("Color - Detected Circles", frame)
            cv2.imshow("Color - Grayscale", gray)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {str(e)}")

    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="passthrough"
            )

            if self.latest_depth_frame is None or self.latest_depth_frame.size == 0:
                self.get_logger().error("Error: depth image is empty")
                return

            # Debug
            self.get_logger().debug(
                f"Depth image shape: {self.latest_depth_frame.shape}"
            )

            depth_display = cv2.convertScaleAbs(self.latest_depth_frame)
            depth_display = cv2.cvtColor(depth_display, cv2.COLOR_GRAY2BGR)

            # Overlay circles from color image
            if self.circles is not None:
                for x, y, r in self.circles:
                    x_scaled = int(x * self.scale_x) + self.offset_x
                    y_scaled = int(y * self.scale_y) + self.offset_y
                    r_scaled = int(r * self.scale_x)

                    # Draw on depth image (using scaled radius)
                    cv2.circle(
                        depth_display, (x_scaled, y_scaled), r_scaled, (0, 255, 0), 4
                    )
                    cv2.rectangle(
                        depth_display,
                        (x_scaled - 5, y_scaled - 5),
                        (x_scaled + 5, y_scaled + 5),
                        (0, 128, 255),
                        -1,
                    )

            cv2.imshow("Depth - Overlay", depth_display)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error in depth_callback: {str(e)}")


def main(args=None):
    try:
        rclpy.init(args=args)
        detect_pipe = DetectPipe()
        rclpy.spin(detect_pipe)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            detect_pipe.destroy_node()
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
