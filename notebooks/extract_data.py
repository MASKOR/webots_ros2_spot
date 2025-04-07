from rosbags.rosbag2 import Reader
from rosbags.typesys import Stores, get_typestore
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

bridge = CvBridge()
bag_path = "/path/to_your/rosbag2_file"  # Don't forget
output_dir = "./dataset"
os.makedirs(f"{output_dir}/color", exist_ok=True)
os.makedirs(f"{output_dir}/depth", exist_ok=True)

typestore = get_typestore(Stores.ROS2_HUMBLE)

color_images = []
depth_images = []
twist_commands = []
timestamps = []

with Reader(bag_path) as reader:
    color_msgs = []
    depth_msgs = []
    twist_msgs = []

    for connection, timestamp, rawdata in reader.messages():
        msg = typestore.deserialize_cdr(rawdata, connection.msgtype)
        if connection.topic == "/kinova_color":
            color_msgs.append((timestamp, msg))
        elif connection.topic == "/kinova_depth":
            depth_msgs.append((timestamp, msg))
        elif connection.topic == "/twist_controller/commands":
            twist_msgs.append((timestamp, msg))

    # Synchronize messages (assuming timestamps are close enough)
    for t_color, color_msg in color_msgs:
        # Find closest depth and twist messages
        closest_depth = min(depth_msgs, key=lambda x: abs(x[0] - t_color), default=None)
        closest_twist = min(twist_msgs, key=lambda x: abs(x[0] - t_color), default=None)

        if (
            closest_depth
            and closest_twist
            and abs(closest_depth[0] - t_color) < 1e9
            and abs(closest_twist[0] - t_color) < 1e9
        ):  # Check if the closest messages are within a reasonable time window
            color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(closest_depth[1], "passthrough")
            twist = closest_twist[1]

            color_path = f"{output_dir}/color/color_{t_color}.png"
            depth_path = f"{output_dir}/depth/depth_{t_color}.npy"
            cv2.imwrite(color_path, color_img)
            np.save(depth_path, depth_img)

            color_images.append(color_path)
            depth_images.append(depth_path)
            twist_commands.append(
                [
                    twist.twist.linear.x,
                    twist.twist.linear.y,
                    twist.twist.linear.z,
                    twist.twist.angular.x,
                    twist.twist.angular.y,
                    twist.twist.angular.z,
                ]
            )
            timestamps.append(t_color)

np.save(f"{output_dir}/twist_commands.npy", np.array(twist_commands))
np.save(f"{output_dir}/timestamps.npy", np.array(timestamps))
print(
    f"Extracted {len(color_images)} color images, {len(depth_images)} depth images, and {len(twist_commands)} twist commands."
)
