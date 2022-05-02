import os
import pathlib
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher


def generate_launch_description():
    package_dir = get_package_share_directory('webots_spot')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'spot.urdf')).read_text()

    webots = WebotsLauncher(
        world=os.path.join(package_dir, 'worlds', 'spot.wbt')
    )

    spot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        parameters=[
            {'robot_description': robot_description},
            {'set_robot_state_publisher': True},
        ],
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': '<robot name="Spot"><link name=""/></robot>'
        }],
    )
    env_tester = Node(
        package='webots_spot',
        executable='env_tester',
    )

    return LaunchDescription([
        webots,
        spot_driver,
        robot_state_publisher,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
