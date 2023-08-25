#!/usr/bin/env python

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController

package_dir = get_package_share_directory('webots_spot')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default=True)
    
    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'spot.wbt']),
        ros2_supervisor=True
    )

    spot_driver = WebotsController(
        robot_name='Spot',
        parameters=[{
            'robot_description': os.path.join(package_dir, 'resource', 'spot.urdf'),
            'use_sim_time': use_sim_time,
            'set_robot_state_publisher': False, # foot positions are wrong with webot's urdf
        }],
        respawn=True
    )

    with open(os.path.join(package_dir, 'resource', 'rd_spot.urdf')) as f:
        robot_desc = f.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': use_sim_time,
        }],
    )

    spot_pointcloud2 = Node(
        package='webots_spot',
        executable='spot_pointcloud2',
        output='screen',
    )

    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/Spot/Velodyne_Puck/point_cloud'), ],
        parameters=[{
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14,
            'angle_max': 3.14,
            'angle_increment': 0.00872,
            'scan_time': 0.1,
            'range_min': 0.9,
            'range_max': 100.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
        name='pointcloud_to_laserscan'
    )

    return LaunchDescription([
        webots,
        webots._supervisor,
        spot_driver,
        robot_state_publisher,
        # spot_pointcloud2,
        webots_event_handler,
        pointcloud_to_laserscan_node
    ])
