#!/usr/bin/env python

import os
import tempfile
import xacro
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
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection

package_dir = get_package_share_directory("webots_spot")


def _base_driver_urdf(prefix):
    """Render spot_control.urdf.xacro for one robot and drop it in a temp .urdf.

    The webots driver reads a file path; passing a raw string is deprecated. The
    xacro just stamps `prefix` (= ns) into every sensor frameName / the IMU topic.
    """
    xacro_path = os.path.join(package_dir, "resource", "spot_control.urdf.xacro")
    content = xacro.process_file(xacro_path, mappings={"prefix": prefix}).toxml()
    tmp = tempfile.NamedTemporaryFile(
        mode="w", suffix=f"_{prefix}_control.urdf", delete=False
    )
    tmp.write(content)
    tmp.close()
    return tmp.name


def get_robot_pipeline(ns, base_robot, arm_robot):
    """All ROS 2 nodes for a single Spot instance.

    ns         -- ROS namespace / frame prefix for this robot, e.g. "Spot1"
    base_robot -- Webots <name> of the quadruped Robot node, e.g. "Spot1"
    arm_robot  -- Webots <name> of the *nested* SpotArm Robot node, e.g. "SpotArm1"

    The arm motors (spotarm_*_joint, Slider11, gripper_*_finger_joint) belong to
    the nested SpotArm robot, so the ros2_control driver must connect to
    `arm_robot`, NOT to `base_robot`.
    """
    spotarm_ros2_control_params = os.path.join(
        package_dir, "resource", "spotarm_ros2_controllers.yaml"
    )

    # --- Base quadruped driver (SpotDriver plugin + IMU) -> connects to Spot* ---
    # No `namespace=`: the driver already publishes device topics under the
    # Webots robot name ("Spot1"/"Spot2"), so adding a ROS namespace would
    # double it (/Spot1/Spot1/...). The xacro prefix gives each sensor a
    # "<ns>/"-prefixed frame_id instead.
    base_driver = WebotsController(
        robot_name=base_robot,
        parameters=[
            {"robot_description": _base_driver_urdf(ns)},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
        ],
        respawn=True,
    )

    # --- Arm driver (ros2_control) -> connects to the nested SpotArm* robot ---
    arm_driver = WebotsController(
        robot_name=arm_robot,
        namespace=ns,
        parameters=[
            {"robot_description": os.path.join(package_dir, "resource", "spotarm_control.urdf")},
            {"use_sim_time": True},
            {"set_robot_state_publisher": False},
            spotarm_ros2_control_params,
        ],
    )

    # --- Namespaced ros2_control spawners ---
    controller_manager_timeout = ["--controller-manager-timeout", "500"]
    controller_manager_prefix = "python.exe" if os.name == "nt" else ""
    target_cm = f"/{ns}/controller_manager"

    def spawner(controller):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            prefix=controller_manager_prefix,
            arguments=[controller, "-c", target_cm] + controller_manager_timeout,
        )

    ros2_control_spawners = [
        spawner("spotarm_joint_trajectory_controller"),
        spawner("spotarm_joint_state_broadcaster"),
        spawner("tiago_gripper_joint_trajectory_controller"),
    ]

    waiting_nodes = WaitForControllerConnection(
        target_driver=arm_driver, nodes_to_start=ros2_control_spawners
    )

    # retract_manipulator uses absolute /<controller>/follow_joint_trajectory
    # topics; remap them into this robot's namespace.
    initial_manipulator_positioning = Node(
        package="webots_spot",
        executable="retract_manipulator",
        output="screen",
        namespace=ns,
        remappings=[
            (
                "/spotarm_joint_trajectory_controller/follow_joint_trajectory",
                f"/{ns}/spotarm_joint_trajectory_controller/follow_joint_trajectory",
            ),
            (
                "/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory",
                f"/{ns}/tiago_gripper_joint_trajectory_controller/follow_joint_trajectory",
            ),
        ],
    )

    # Full kinematic TF tree (base_link -> legs / arm / sensor links) for this
    # robot. The driver xacro has NO links; the model is spot.urdf, published
    # here with a per-robot frame_prefix so frames match spot_driver's
    # "<ns>/base_link" broadcast. Fed by /<ns>/joint_states (legs from
    # spot_driver + arm from the joint_state_broadcaster).
    with open(os.path.join(package_dir, "resource", "spot.urdf")) as f:
        spot_description = f.read()

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=ns,
        parameters=[{
            "robot_description": spot_description,
            "use_sim_time": True,
            "frame_prefix": f"{ns}/",
        }],
    )

    pointcloud_to_laserscan_node = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="pointcloud_to_laserscan",
        namespace=ns,
        remappings=[
            ("cloud_in", f"/{ns}/Velodyne_Puck/point_cloud"),
            ("scan", f"/{ns}/scan"),
        ],
        parameters=[{
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
        }],
    )

    return [
        base_driver,
        arm_driver,
        waiting_nodes,
        robot_state_publisher,
        initial_manipulator_positioning,
        pointcloud_to_laserscan_node,
    ]


def get_ros2_nodes(*args):
    """Runtime nodes for both instances; re-created on simulation reset."""
    return (
        get_robot_pipeline("Spot1", "Spot1", "SpotArm1")
        + get_robot_pipeline("Spot2", "Spot2", "SpotArm2")
    )


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="maze.wbt",
        description="World file to load, relative to the package 'worlds' directory",
    )

    publish_map_odom_arg = DeclareLaunchArgument("publish_map_odom", default_value="true")

    map_to_odom_spot1 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_spot1",
        output="screen",
        arguments=["--x", "8.39", "--y", "-0.94", "--z", "0", "--yaw", "3.14159",
                   "--frame-id", "map", "--child-frame-id", "Spot1/odom"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_map_odom")),
    )

    map_to_odom_spot2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_spot2",
        output="screen",
        arguments=["--x", "-10.77", "--y", "-1.3", "--z", "0", "--yaw", "0",
                   "--frame-id", "map", "--child-frame-id", "Spot2/odom"],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(LaunchConfiguration("publish_map_odom")),
    )

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, "worlds", LaunchConfiguration("world")])
    )
    ros2_supervisor = Ros2SupervisorLauncher()

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

    return LaunchDescription(
        [
            world_arg,
            publish_map_odom_arg,
            map_to_odom_spot1,
            map_to_odom_spot2,
            webots,
            ros2_supervisor,
            webots_event_handler,
            reset_handler,
        ]
        + get_ros2_nodes()
    )
