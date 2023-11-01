#!/usr/bin/env python3

from math import radians
import numpy as np
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2
import webots_spot.point_cloud2 as pc2

import open3d as o3d
from scipy.spatial.transform import Rotation as R

intrinsic = o3d.camera.PinholeCameraIntrinsic(
    width=424,
    height=240,
    fx=212.1688885346757,
    fy=212.1688885346757,
    cx=212.0,
    cy=120.0,
)


def transformation_matrix(tr, rt_quat):
    tr = np.array([tr])
    r = R.from_quat(rt_quat).as_matrix()
    r_tr = np.concatenate((r, tr.T), axis=1)
    return np.concatenate((r_tr, np.array([[0, 0, 0, 1]])), axis=0)


tfs = {
    "left_head_depth": transformation_matrix(
        [0.463, 0.045, 0.030], [-0.032, 0.226, -0.180, 0.957]
    ),
    "right_head_depth": transformation_matrix(
        [0.463, -0.045, 0.028], [0.053, 0.210, 0.180, 0.960]
    ),
    "left_flank_depth": transformation_matrix(
        [-0.136, 0.110, 0.066], [-0.142, 0.142, 0.693, 0.693]
    ),
    "right_flank_depth": transformation_matrix(
        [-0.136, -0.110, 0.066], [0.145, 0.145, -0.692, 0.692]
    ),
    "rear_depth": transformation_matrix(
        [-0.278 - 0.15, -1.398 + 1.4, 0.629 - 0.6], [-0.235, 0.000, 0.972, -0.000]
    ),
    # Using magic numbers for rear depth as the frame is somehow at an incorrect position
}
sensor_locations = list(tfs.keys())

r = R.from_euler("xyz", [radians(90), radians(180), radians(90)])
# Webots uses different orientations
rectifying_tf = transformation_matrix(
    [0, 0, 0], [r.as_quat()[0], r.as_quat()[1], r.as_quat()[2], r.as_quat()[3]]
)

rectified_tfs = {frame: np.matmul(tfs[frame], rectifying_tf) for frame in tfs}


class DepthToPCD2(Node):
    def __init__(self) -> None:
        # ROS2 init
        super().__init__("d2pcd")

        self.create_subscription(Image, "/Spot/left_head_depth", self.frontleft_fn, 10)
        self.create_subscription(
            Image, "/Spot/right_head_depth", self.frontright_fn, 10
        )
        self.create_subscription(Image, "/Spot/left_flank_depth", self.left_fn, 10)
        self.create_subscription(Image, "/Spot/right_flank_depth", self.right_fn, 10)
        self.create_subscription(Image, "/Spot/rear_depth", self.back_fn, 10)
        self.create_timer(0.01, self.run)
        self.pcd_pub = self.create_publisher(PointCloud2, "/Spot/pointcloud2", 10)

        self.check_fl = False
        self.check_fr = False
        self.check_l = False
        self.check_r = False
        self.check_b = False

        self.bridge = CvBridge()
        self.pcd = o3d.geometry.PointCloud()

    def frontleft_fn(self, msg: Image) -> None:
        self.frontleft_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.check_fl = True

    def frontright_fn(self, msg: Image) -> None:
        self.frontright_img = self.bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )
        self.check_fr = True

    def left_fn(self, msg: Image) -> None:
        self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.check_l = True

    def right_fn(self, msg: Image) -> None:
        self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.check_r = True

    def back_fn(self, msg: Image) -> None:
        self.back_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.check_b = True

    def run(self):
        if not self.count_subscribers("/Spot/pointcloud2"):
            return
        if (
            self.check_fl
            and self.check_fr
            and self.check_l
            and self.check_r
            and self.check_b
        ):
            self.check_fl = False
            self.check_fr = False
            self.check_l = False
            self.check_r = False
            self.check_b = False

            # Depth image to Open3d pointcloud
            open3d_cloud = [
                self.pcd.create_from_depth_image(
                    o3d.geometry.Image((depth_image).astype(np.float32)), intrinsic
                )
                for depth_image in [
                    self.frontleft_img,
                    self.frontright_img,
                    self.left_img,
                    self.right_img,
                    self.back_img,
                ]
            ]

            # Voxel downsampling
            open3d_cloud = [
                pcd.voxel_down_sample(voxel_size=0.2) for pcd in open3d_cloud
            ]

            # Combine the pointclouds
            open3d_cloud[0].transform(rectified_tfs[sensor_locations[0]])
            cloud_data = np.asarray(open3d_cloud[0].points).tolist()
            for i in range(4):
                open3d_cloud[i + 1].transform(rectified_tfs[sensor_locations[i + 1]])
                cloud_data.extend(np.asarray(open3d_cloud[i + 1].points).tolist())

            # Cloud data to Pointcloud2
            ros_cloud = self.convertCloudFromOpen3dToRos(cloud_data)
            self.pcd_pub.publish(ros_cloud)

    # Convert the datatype of point cloud from Open3D to ROS PointCloud2 (XYZRGB only)
    def convertCloudFromOpen3dToRos(self, cloud_data, frame_id="base_link"):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        # create ros_cloud
        return pc2.create_cloud_xyz32(header, cloud_data)


def main(args=None):
    rclpy.init(args=args)
    d2pcd = DepthToPCD2()

    try:
        rclpy.spin(d2pcd)
    except KeyboardInterrupt:
        d2pcd.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
