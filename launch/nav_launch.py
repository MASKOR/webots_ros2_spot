import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    package_dir = get_package_share_directory("webots_spot")
    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    nav2_map = os.path.join(package_dir, "map", "map.yaml")
    nav2_params = os.path.join(package_dir, "params", "nav2_params.yaml")

    use_rviz = LaunchConfiguration("rviz", default=True)
    rviz_config = os.path.join(package_dir, "resource", "nav2.rviz")
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["--display-config=" + rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=launch.conditions.IfCondition(use_rviz),
    )

    initial_pose = LaunchConfiguration("set_initial_pose", default=False)
    spot_initial_pose = Node(
        package="webots_spot",
        executable="set_initial_pose",
        output="screen",
        condition=launch.conditions.IfCondition(initial_pose),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments=[
            ("map", nav2_map),
            ("use_sim_time", use_sim_time),
            ("params_file", nav2_params),
        ],
    )

    return LaunchDescription(
        [
            rviz,
            spot_initial_pose,
            nav2,
        ]
    )
