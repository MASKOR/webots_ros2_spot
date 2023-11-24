#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from webots_spot_msgs.srv import BlockPose
from std_srvs.srv import Empty

from controller import Supervisor

import os
import random
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import is_wsl


def randomise_lane(robot):
    if random.random() < 0.5:  # 50% probability for a rightside lane
        l1 = robot.getFromDef("Lane1")
        l1.getField("translation").setSFVec3f([6.6778, 1.90426, 0.001])
        l1.getField("rotation").setSFRotation([0, 0, -1, -1.04721])
        l2 = robot.getFromDef("Lane2")
        l2.getField("translation").setSFVec3f([3.48651, 2.88433, 0.001])
        l2.getField("rotation").setSFRotation([0, 0, -1, -1.57081])
        l3 = robot.getFromDef("Lane3")
        l3.getField("translation").setSFVec3f([2.02652, 1.67433, 0.001])
        l3.getField("rotation").setSFRotation([0, 0, -1, 0])


def randomise_imgs(robot, hazmat=False):
    if hazmat:
        img_path = os.path.join(
            get_package_share_directory("webots_spot"), "hazmat_signs/"
        )
    else:
        img_path = os.path.join(
            get_package_share_directory("webots_spot"), "yolo_images/"
        )
    all_imgs = os.listdir(img_path)
    three_imgs = random.sample(all_imgs, 3)
    if is_wsl():
        img_path = ".." + img_path
    for idx, img in enumerate(three_imgs):
        robot.getFromDef("Image" + str(idx + 1)).getField("url").setMFString(
            0, img_path + img
        )


def set_rod(robot, red=False):
    pipe = robot.getFromDef("Pipe")
    if red:
        pipe.getField("translation").setSFVec3f([-1.84, 4.01, 0.74])
        pipe.getField("appearance").getSFNode().getField("baseColor").setSFColor(
            [1, 0, 0]
        )
    else:
        pipe.getField("translation").setSFVec3f([-1.84, 4.01, 0.85])
        pipe.getField("appearance").getSFNode().getField("baseColor").setSFColor(
            [1, 1, 0]
        )


def shuffle_cubes(robot):
    combinations = []
    for i in range(3):
        for j in range(3):
            combinations.append(
                [-8, round(-4 + i * 0.2, 3), round(0.025 + j * 0.05, 3)]
            )

    locations = random.sample(combinations, 3)
    cube_locations = {}
    for c, location in zip("ABC", locations):
        cube_locations[c] = location

    cube_locations_aphabets = {}
    for cube, location in cube_locations.items():
        on_top_of_cube = False
        for c, l in cube_locations.items():
            if cube == c:
                continue
            if (
                location[0] == l[0]
                and location[1] == l[1]
                and location[2] == round(l[2] + 0.05, 3)
            ):  # 0.05 becomes 0.050...01
                on_top_of_cube = True
                cube_locations_aphabets[cube] = c
                break
            if (
                location[0] == l[0]
                and location[1] == l[1]
                and location[2] == l[2] + 0.1
            ):
                on_top_of_cube = True
                cube_locations_aphabets[cube] = c
        if not on_top_of_cube:
            if location[1] == -4:
                cube_locations_aphabets[cube] = "t1"
            elif location[1] == -3.8:
                cube_locations_aphabets[cube] = "t2"
            else:
                cube_locations_aphabets[cube] = "t3"
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], 0.025]
            )  # prevent abrupt fall on floor
        else:
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], location[2]]
            )

    return cube_locations_aphabets


class ArenaModifier(Node):
    def __init__(self):
        super().__init__("ArenaModifier")

        self.__robot = Supervisor()
        self.__timestep = int(self.__robot.getBasicTimeStep())

        self.create_timer(1 / 1000, self.step_callback)

        # self.create_timer(1, self.timer_cb)

        self.create_service(Empty, "/hazmat_signs", self.hazmat_signs)
        self.create_service(Empty, "/red_rod", self.red_rod)
        self.create_service(BlockPose, "/get_block_pose", self.gpp_block_pose)

        self.i = 0

        randomise_lane(self.__robot)
        randomise_imgs(self.__robot)
        set_rod(self.__robot)
        self.cubes_loc = shuffle_cubes(self.__robot)

    def step_callback(self):
        if self.__robot.step(self.__timestep) < 0:
            self.get_logger().info("ArenaModifier is shutting down...")

    def timer_cb(self):
        pass

    def hazmat_signs(self, request, response):
        randomise_imgs(self.__robot, True)
        return response

    def red_rod(self, request, response):
        set_rod(self.__robot, True)
        return response

    def gpp_block_pose(self, request, response):
        response.location = self.cubes_loc[request.block.upper()].lower()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArenaModifier()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()