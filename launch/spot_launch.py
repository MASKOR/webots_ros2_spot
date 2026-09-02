#!/usr/bin/env python

import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import (
    WaitForControllerConnection,
)


package_dir = get_package_share_directory("webots_spot")


# Define all the ROS 2 nodes that need to be restart on simulation reset here
def get_ros2_nodes(*args):
    # SpotArm Driver node
    spotarm_ros2_control_params = os.path.join(
        package_dir, "resource", "spotarm_ros2_controllers.yaml"
    )
    spotarm_driver = WebotsController(
        robot_name="SpotArm",
        parameters=[
            {
                "robot_description": os.path.join(
                    package_dir, "resource", "spotarm_control.urdf"
                )
            },
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            spotarm_ros2_control_params,
        ],
    )

    # ROS2 control spawners for SpotArm
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_trajectory_controller", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=["spotarm_joint_state_broadcaster", "-c", "/controller_manager"]
        + controller_manager_timeout,
    )
    tiago_gripper_joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        prefix=controller_manager_prefix,
        arguments=[
            "tiago_gripper_joint_trajectory_controller",
            "-c",
            "/controller_manager",
        ]
        + controller_manager_timeout,
    )

    ros2_control_spawners = [
        trajectory_controller_spawner,
        joint_state_broadcaster_spawner,
        tiago_gripper_joint_trajectory_controller_spawner,
    ]

    # Wait for the simulation to be ready to start RViz, the navigation and spawners
    waiting_nodes = WaitForControllerConnection(
        target_driver=spotarm_driver, nodes_to_start=ros2_control_spawners
    )

    initial_manipulator_positioning = Node(
        package="webots_spot",
        executable="retract_manipulator",
        output="screen",
    )

    return [spotarm_driver, waiting_nodes, initial_manipulator_positioning]


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="maze.wbt",
        description="World file to load, relative to the package 'worlds' directory",
    )

    # spot_driver publishes 'odom' spawn-relative and rotated 180deg about Z, so
    # 'map' == Webots world frame is recovered with a static map->odom equal to
    # the robot's spawn pose. Defaults match the Spot node in maze.wbt.
    # Disable (publish_map_odom:=false) when running nav2/AMCL, which owns map->odom.
    publish_map_odom_arg = DeclareLaunchArgument(
        "publish_map_odom", default_value="true"
    )
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="8.39")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="-0.94")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="3.14159")

    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static",
        output="screen",
        arguments=[
            "--x", LaunchConfiguration("spawn_x"),
            "--y", LaunchConfiguration("spawn_y"),
            "--z", "0",
            "--yaw", LaunchConfiguration("spawn_yaw"),
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_map_odom")),
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution(
            [package_dir, "worlds", LaunchConfiguration("world")]
        )
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
            world_arg,
            publish_map_odom_arg,
            spawn_x_arg,
            spawn_y_arg,
            spawn_yaw_arg,
            map_to_odom_tf,
            webots,
            ros2_supervisor,
            spot_driver,
            robot_state_publisher,
            # spot_pointcloud2,
            webots_event_handler,
            reset_handler,
            pointcloud_to_laserscan_node,
        ]
        + get_ros2_nodes()
    )
