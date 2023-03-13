import os
import pathlib
import launch
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch_ros.actions import Node
from webots_ros2_driver.utils import controller_url_prefix
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher, Ros2SupervisorLauncher

package_dir = get_package_share_directory('webots_spot')


def get_ros2_nodes(*args):
    spot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'Spot'},
        parameters=[
            {'robot_description': pathlib.Path(os.path.join(package_dir, 'resource', 'spot.urdf')).read_text()},
            {'use_sim_time': False},
            {'set_robot_state_publisher': False}, # foot positions are wrong with webot's urdf
        ],
    )
    return [
        spot_driver,
    ]


def generate_launch_description():
    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'resource', filename)).read_text()

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'spot.wbt'])
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': load_file('rd_spot.urdf')},
            {'use_sim_time': False},
        ],
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
    ros2_supervisor = Ros2SupervisorLauncher()
    # The following line is important!
    # This event handler respawns the ROS 2 nodes on simulation reset (supervisor process ends).
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=ros2_supervisor,
            on_exit=get_ros2_nodes,
        )
    )

    return LaunchDescription([
        webots,
        ros2_supervisor,
        robot_state_publisher,
        # spot_pointcloud2,
        webots_event_handler,
        Node(
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
        ),
        reset_handler
    ] + get_ros2_nodes())