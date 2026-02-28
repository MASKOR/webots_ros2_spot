#!/usr/bin/env python

"""Launch Webots Gen3 and Spot simulation with MoveIt2."""

import os
import pathlib
import yaml
from launch.actions import LogInfo
from launch import LaunchDescription
from launch_param_builder import ParameterBuilder
from launch_ros.actions import Node
from ament_index_python.packages import (
    get_package_share_directory,
    get_packages_with_prefixes,
)


PACKAGE_NAME = "webots_spot"


def generate_launch_description():
    launch_description_nodes = []
    package_dir = get_package_share_directory(PACKAGE_NAME)

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, "resource", filename)).read_text()

    def load_yaml(filename):
        return yaml.safe_load(load_file(filename))

    # Get parameters for the Servo node
    servo_params = {
        "moveit_servo": ParameterBuilder("webots_spot")
        .yaml("resource/moveit_servo_config.yaml")
        .to_dict()
    }

    # This filter parameter should be >1. Increase it for greater smoothing but slower motion.
    low_pass_filter_coeff = {"butterworth_filter_coeff": 1.5}
    update_period = {"update_period": 0.05}
    planning_group_name = {"planning_group_name": "manipulator"}

    # Check if moveit is installed
    if "moveit" in get_packages_with_prefixes():
        # Configuration
        description = {"robot_description": load_file("spot.urdf")}
        description_semantic = {"robot_description_semantic": load_file("gen3.srdf")}
        description_kinematics = {
            "robot_description_kinematics": load_yaml("moveit_kinematics.yaml")
        }
        description_joint_limits = {
            "robot_description_planning": load_yaml("moveit_joint_limits.yaml")
        }
        sim_time = {"use_sim_time": True}

        launch_description_nodes.append(
            Node(
                package="moveit_servo",
                executable="servo_node_main",
                parameters=[
                    servo_params,
                    # low_pass_filter_coeff,
                    planning_group_name,
                    # update_period,
                    servo_params,
                    description,
                    description_semantic,
                    description_kinematics,
                    description_joint_limits,
                    sim_time,
                ],
                output="screen",
            )
        )
        # launch_description_nodes.append(
        #     Node(
        #         package="webots_spot",
        #         executable="set_ready_position",
        #         output="screen",
        #     )
        # )

    else:
        launch_description_nodes.append(
            LogInfo(
                msg='"moveit" package is not installed, \
                                                please install it in order to run this demo.'
            )
        )

    return LaunchDescription(launch_description_nodes)
