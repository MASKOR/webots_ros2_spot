import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import csv


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

        self.color_hough_dp = 1.0
        self.color_hough_min_dist = 20
        self.color_hough_param1 = 80
        self.color_hough_param2 = 50
        self.color_min_radius = 20
        self.color_max_radius = 50

        # Scaling factors (based on topic echo)
        self.depth_width = 480
        self.depth_height = 270
        self.color_width = 640
        self.color_height = 480
        self.scale_x = self.depth_width / self.color_width
        self.scale_y = self.depth_height / self.color_height

        # Offset(tune these manually)
        self.offset_x = -20
        self.offset_y = 0

        # CSV file setup
        self.csv_file = "detected_circles.csv"
        with open(self.csv_file, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow(["x", "y", "radius", "avg_depth_meters"])

        self.get_logger().info("DetectPipe node has been started.")

    def image_callback(self, msg):
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
                cv2.rectangle(frame, (x - 5, y - 5), (x + 5, y + 5), (0, 128, 255), -1)

                # Scale to depth coordinates
                x_scaled = int(x * self.scale_x) + self.offset_x
                y_scaled = int(y * self.scale_y) + self.offset_y
                r_scaled = int(r * self.scale_x)

                # Check depth if available
                if self.latest_depth_frame is not None:
                    if (
                        0 <= y_scaled < self.depth_height
                        and 0 <= x_scaled < self.depth_width
                    ):
                        # Extract depth ROI around the circle
                        y_min = max(0, y_scaled - r_scaled)
                        y_max = min(self.depth_height, y_scaled + r_scaled)
                        x_min = max(0, x_scaled - r_scaled)
                        x_max = min(self.depth_width, x_scaled + r_scaled)
                        depth_roi = self.latest_depth_frame[y_min:y_max, x_min:x_max]

                        if depth_roi.size == 0:
                            self.get_logger().warn(
                                f"Empty depth ROI at ({x_scaled}, {y_scaled})"
                            )
                            continue

                        valid_depths = depth_roi[depth_roi > 0]
                        if valid_depths.size == 0:
                            self.get_logger().warn(
                                f"No valid depth values in ROI at ({x_scaled}, {y_scaled})"
                            )
                            continue

                        avg_depth = np.mean(valid_depths)
                        distance_meters = (
                            avg_depth / 1000.0
                        )  # (assuming depth is in mm)
                        self.get_logger().info(
                            f"Circle at ({x}, {y}) [scaled: ({x_scaled}, {y_scaled})], "
                            f"radius {r} [scaled: {r_scaled}], "
                            f"avg distance: {distance_meters:.2f} meters"
                        )

                        with open(self.csv_file, mode="a", newline="") as file:
                            writer = csv.writer(file)
                            writer.writerow([x, y, r, distance_meters])
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

    def depth_callback(self, msg):
        self.latest_depth_frame = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )  # This is where we get the depth

        if self.latest_depth_frame is None or self.latest_depth_frame.size == 0:
            self.get_logger().error("Error: depth image is empty")
            return

        # Debug
        self.get_logger().debug(f"Depth image shape: {self.latest_depth_frame.shape}")

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
