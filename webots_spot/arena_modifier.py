from webots_spot_msgs.msg import Int32
from webots_spot_msgs.srv import BlockPose
from std_srvs.srv import Empty

import os
import random
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.utils import is_wsl

BOX_COLOR = {"red": [1, 0, 0], "green": [0, 1, 0], "blue": [0, 0, 1]}


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


def randomise_imgs(robot, simpler_imagebox, hazmat=False):
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

    if simpler_imagebox:
        for idx in range(len(three_imgs)):
            robot.getFromDef("Image" + str(idx + 1)).getField(
                "image_translation"
            ).setSFVec3f([0, -0.3, 0.25])
            robot.getFromDef("Image" + str(idx + 1)).getField(
                "image_rotation"
            ).setSFRotation([1, 0, 0, 1.57081])


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

    cube_locations_alphabets = {}
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
                cube_locations_alphabets[cube] = c
                break
            if (
                location[0] == l[0]
                and location[1] == l[1]
                and location[2] == l[2] + 0.1
            ):
                on_top_of_cube = True
                cube_locations_alphabets[cube] = c
        if not on_top_of_cube:
            if location[1] == -4:
                cube_locations_alphabets[cube] = "t1"
            elif location[1] == -3.8:
                cube_locations_alphabets[cube] = "t2"
            else:
                cube_locations_alphabets[cube] = "t3"
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], 0.025]
            )  # prevent abrupt fall on floor
        else:
            robot.getFromDef(cube).getField("translation").setSFVec3f(
                [location[0], location[1], location[2]]
            )

    return cube_locations_alphabets


class ArenaModifier:
    def __init__(self, node, robot):
        self.node = node
        self.__robot = robot

        self.exog_pub = self.node.create_publisher(Int32, "/arena3_exog", 1)
        self.arena3_score_pub = self.node.create_publisher(Int32, "/arena3_score", 1)

        self.node.create_timer(1, self.timer_cb)
        self.node.create_timer(10, self.yellow_box_exog_timer_cb)

        self.node.create_service(Empty, "/hazmat_signs", self.hazmat_signs)
        self.node.create_service(Empty, "/red_rod", self.red_rod)
        self.node.create_service(BlockPose, "/get_block_pose", self.gpp_block_pose)

        if not self.node.has_parameter("simpler_imagebox"):
            self.node.declare_parameter("simpler_imagebox", False)
        self.simpler_imagebox = self.node.get_parameter("simpler_imagebox").value

        # Start randomising stuffs
        randomise_lane(self.__robot)
        randomise_imgs(self.__robot, self.simpler_imagebox)
        set_rod(self.__robot)
        self.cubes_loc = shuffle_cubes(self.__robot)

    def yellow_box_exog_timer_cb(self):
        if not hasattr(self, "chosen_yellow_box"):
            return

        msg = Int32()
        msg.data = self.chosen_yellow_box
        self.exog_pub.publish(msg)

    def timer_cb(self):
        # Stuffs for Arena 3
        if not hasattr(self, "box_colors"):
            # Randomise Drop Box colors
            self.box_colors = list(BOX_COLOR.keys())
            random.shuffle(self.box_colors)

            # Change Drop Box colors with above random sequence
            for idx, color in enumerate(self.box_colors):
                self.__robot.getFromDef(f"DropBox{idx+1}").getField(
                    "legAppearance"
                ).getSFNode().getField("baseColor").setSFColor(BOX_COLOR[color])

            # Save Drop Box positions by color: position pair
            self.box_positions = {}
            for idx, color in enumerate(self.box_colors):
                self.box_positions[color] = (
                    self.__robot.getFromDef(f"DropBox{idx+1}")
                    .getField("translation")
                    .getSFVec3f()
                )

            self.chosen_yellow_box = random.randint(1, 3)
            self.chosen_yellow_box_position = (
                self.__robot.getFromDef(f"YellowDropBox_{self.chosen_yellow_box}")
                .getField("translation")
                .getSFVec3f()
            )

            # Initial Arena 3 Score
            self.current_score = 500

        # Open respective doors where box and cube match
        for idx, color in enumerate(self.box_colors):
            if self.check_dropbox_and_cubes(
                color
            ) and self.check_yellow_dropbox_and_cubes(idx + 1):
                self.open_door(f"Door{idx+1}")

        # Scoring in Arena 3
        if not self.spot_has_passed_door3():
            self.current_score = 500 - int(self.__robot.getTime())
        score_msg = Int32()
        score_msg.data = self.current_score
        self.arena3_score_pub.publish(score_msg)

    def spot_has_passed_door3(self):
        spot_x = self.__robot.getFromDef("Spot").getField("translation").getSFVec3f()[0]
        door3_x = (
            self.__robot.getFromDef("Door3").getField("translation").getSFVec3f()[0]
        )

        if spot_x < door3_x:
            return True
        return False

    def open_door(self, door):
        door_tr_vec = self.__robot.getFromDef(door).getField("translation")
        door_tr = door_tr_vec.getSFVec3f()
        door_tr_vec.setSFVec3f([door_tr[0], 20, door_tr[2]])
        return False

    def check_yellow_dropbox_and_cubes(self, checking_door_number):
        # We check only for the Chosen Yellow Box, otherwise the door may open
        if self.chosen_yellow_box != checking_door_number:
            return True

        for color in BOX_COLOR.keys():
            for idx in range(3):
                current_cube_name = f"{color.upper()}_{idx+1}"
                cube_tr_def = self.__robot.getFromDef(current_cube_name).getField(
                    "translation"
                )
                cube_tr = cube_tr_def.getSFVec3f()

                # If any of the 9 cubes in the chosen yellow Drop Box, return true
                if (
                    abs(self.chosen_yellow_box_position[0] - cube_tr[0]) < 0.3
                    and abs(self.chosen_yellow_box_position[1] - cube_tr[1]) < 0.3
                    and cube_tr[2] < 0.1
                ):
                    return True

        return False

    def check_dropbox_and_cubes(self, color):
        if not hasattr(self, "initial_cube_positions"):
            self.initial_cube_positions = {}

        box_tr = self.box_positions[color]

        # Check with single color all 3 cubes one by one
        for i in range(3):
            current_cube_name = f"{color.upper()}_{i+1}"
            cube_tr_def = self.__robot.getFromDef(current_cube_name).getField(
                "translation"
            )
            cube_tr = cube_tr_def.getSFVec3f()

            # Saves initial cube positions
            if current_cube_name not in self.initial_cube_positions.keys():
                self.initial_cube_positions[current_cube_name] = cube_tr

            # If any of the 3 same colored cubes in Drop Box, return true
            if (
                abs(box_tr[0] - cube_tr[0]) < 0.3
                and abs(box_tr[1] - cube_tr[1]) < 0.3
                and cube_tr[2] < 0.1
            ):
                return True
            elif cube_tr[2] < 0.1:
                if not (
                    abs(self.chosen_yellow_box_position[0] - cube_tr[0]) < 0.3
                    and abs(self.chosen_yellow_box_position[1] - cube_tr[1]) < 0.3
                    and cube_tr[2] < 0.1
                ):
                    self.__robot.getFromDef(current_cube_name).getField(
                        "rotation"
                    ).setSFRotation([0.707, -0.707, 0, 0])
                    cube_tr_def.setSFVec3f(
                        self.initial_cube_positions[current_cube_name]
                    )

        return False

    def hazmat_signs(self, request, response):
        randomise_imgs(self.__robot, self.simpler_imagebox, True)
        return response

    def red_rod(self, request, response):
        set_rod(self.__robot, True)
        return response

    def gpp_block_pose(self, request, response):
        response.location = self.cubes_loc[request.block.upper()].lower()
        return response
