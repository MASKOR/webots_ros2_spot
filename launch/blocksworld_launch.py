import os
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription


def generate_launch_description():
    gpp = os.path.join(get_package_share_directory("webots_spot"), "resource")

    gpp_stacker = Node(
        package="webots_spot",
        executable="gpp_stacker",
        output="screen",
    )

    gologpp_agent = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="gologpp_agent",
                executable="gologpp_agent",
                output="screen",
                parameters=[{"gpp_code": gpp + "/webots_blocksworld.gpp"}],
            )
        ],
    )

    return LaunchDescription(
        [
            gpp_stacker,
            gologpp_agent,
        ]
    )
