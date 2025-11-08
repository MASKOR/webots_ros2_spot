import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, PythonExpression, LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node


def generate_launch_description():
    # path to this pkg
    pkg_webots_spot = get_package_share_directory("webots_spot")

    # Comment Alex: One can have different maps for same worlds
    # Is this to much choice for a tutorial?

    # Launch arguments
    available_map_names = [
        f[:-3]
        for f in os.listdir(os.path.join(pkg_webots_spot, "map"))
        if f.endswith(".h5")
    ]

    map_name = LaunchConfiguration("map_name")
    start_rviz = LaunchConfiguration("start_rviz")

    # Move Base Flex
    move_base_flex = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_webots_spot, "launch", "mbf_octo_navigation_server_launch.py"]
            )
        ),
    )
        # Rviz2 send_goal Node
    delayed_exe_path = Node(
        package="bring_up_alert_nav",
        executable="exe_path_node",
        name="exe_path",
        output="screen")

    stamped_conversion = Node(
        package="alert_utils",
        executable="stamped_twist_converter",
        name="stamped_twist_converter",
        output="screen")

    # Start rviz, if desired
    # rviz = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     parameters=[
    #         {"use_sim_time": True},
    #     ],
    #     arguments=[
    #         "-d",
    #         PathJoinSubstitution([pkg_mesh_navigation_tutorials, "resource", "mesh_nav.rviz"]),
    #     ],
    #     condition=IfCondition(start_rviz),
    # )

    ### Helper tf2 static publisher map -> vision to visualize path in rviz
    ### only for webots
    map_vision = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "map", "vision"],
    )

    pcl_mapfilter = Node(
        package="alert_utils",
        executable="pcl_mapfilter",
        name="pcl_mapfilter",
        output="screen"
    )

    return LaunchDescription(

        [
            map_vision,
            move_base_flex,
            stamped_conversion,
            delayed_exe_path,
            pcl_mapfilter,
        ]
    )