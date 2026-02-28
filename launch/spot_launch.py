#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

package_dir = get_package_share_directory("webots_spot")


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    # kinova Driver node
    gen3_ros2_control_params = os.path.join(
        package_dir, "resource", "ros2_controllers.yaml"
    )
    gen3_driver = WebotsController(
        robot_name="Gen3",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "ros2_control.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            gen3_ros2_control_params,
        ],
    )

    # ROS2 control spawners for gen3
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["kinova_joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    robotiq_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "robotiq_gripper_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        robotiq_gripper_controller,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=gen3_driver, nodes_to_start=ros2_control_spawners
    )

    initial_manipulator_positioning = Node(
        package="webots_spot",
        executable="retract_manipulator",
        output="screen",
    )

    return [gen3_driver, waiting_nodes, initial_manipulator_positioning]


def generate_launch_description():
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", "rescue_arena.wbt"])
    )
    ros2_supervisor = Ros2SupervisorLauncher()

    spot_driver = WebotsController(
        robot_name="Spot",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "spot_control.urdf"
                ),
                "use_sim_time": True,
                "set_robot_state_publisher": False,  # foot positions are wrong with webot's urdf
            }
        ],
        respawn=True,
    )

    with open(os.path.join(package_dir, "resource", "spot.urdf")) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_desc,
                "use_sim_time": True,
            }
        ],
    )

    kinect_depth_image_proc = ComposableNodeContainer(
        name="registered_depth_images",
        namespace="Spot/kinect",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depth_image_proc",
                plugin="depth_image_proc::RegisterNode",
                name="register_node",
                namespace="Spot/kinect",
                remappings=[
                    ("rgb/camera_info", "/Spot/kinect_color/camera_info"),
                    ("depth/camera_info", "/Spot/kinect_range/camera_info"),
                    ("depth/image_rect", "/Spot/kinect_range/image"),
                ],
                parameters=[
                    {"fill_upsampling_holes": True, "use_sim_time": True}
                ],
            ),
        ],
        output="both",
        parameters=[{"use_sim_time": True}],
    )

    # spot_pointcloud2 = Node(
    #     package='webots_spot',
    #     executable='spot_pointcloud2',
    #     output='screen',
    # )

    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        remappings=[
            ("cloud_in", "/Spot/Velodyne_Puck/point_cloud"),
        ],
        parameters=[
            {
                "transform_tolerance": 0.01,
                "min_height": 0.0,
                "max_height": 1.0,
                "angle_min": -3.14,
                "angle_max": 3.14,
                "angle_increment": 0.00872,
                "scan_time": 0.1,
                "range_min": 0.9,
                "range_max": 100.0,
                "use_inf": True,
                "inf_epsilon": 1.0,
            }
        ],
        name="pointcloud_to_laserscan",
    )

    return LaunchDescription(
        [
            webots,
            ros2_supervisor,
            spot_driver,
            robot_state_publisher,
            # spot_pointcloud2,
            webots_event_handler,
            reset_handler,
            pointcloud_to_laserscan_node,
            kinect_depth_image_proc,
        ]
        + get_ros2_nodes()
    )