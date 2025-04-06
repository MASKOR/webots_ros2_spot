from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
from cv_bridge import CvBridge
import cv2
import os
import numpy as np

bridge = CvBridge()
bag_path = "/home/guts/rosbags_dexterity/rosbag2_2025_04_05-13_38_10/"
output_dir = "./dataset"
os.makedirs(f"{output_dir}/color", exist_ok=True)
os.makedirs(f"{output_dir}/depth", exist_ok=True)

# Lists to store synchronized data
color_images = []
depth_images = []
twist_commands = []
timestamps = []

# Open the ROS bag
with Reader(bag_path) as reader:
    color_msgs = []
    depth_msgs = []
    twist_msgs = []

    # Collect all messages
    for connection, timestamp, rawdata in reader.messages():
        msg = deserialize_cdr(rawdata, connection.msgtype)
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
        ):  # 1 sec tolerance
            # Convert messages
            color_img = bridge.imgmsg_to_cv2(color_msg, "bgr8")
            depth_img = bridge.imgmsg_to_cv2(closest_depth[1], "passthrough")
            twist = closest_twist[1]  # Assuming geometry_msgs/Twist

            # Save data
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

# Save twist commands to a file
np.save(f"{output_dir}/twist_commands.npy", np.array(twist_commands))
print(f"Extracted {len(timestamps)} synchronized samples.")
