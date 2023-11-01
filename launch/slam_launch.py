import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default=False)

    slam_toolbox = Node(
        parameters=[
            {"use_sim_time": use_sim_time},
            {"base_frame": "base_link"},
            {"max_laser_range": 10.0},
        ],
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
    )
    slam_rviz_config = os.path.join(
        get_package_share_directory("webots_spot"), "resource", "slam.rviz"
    )
    slam_rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + slam_rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            slam_toolbox,
            slam_rviz,
        ]
    )
