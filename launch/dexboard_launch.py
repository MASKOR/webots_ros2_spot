from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            # Linear Center (Reference)
            # Node(
            #     package="tf2_ros",
            #     executable="static_transform_publisher",
            #     output="screen",
            #     arguments=["0", "0", "0", "3.14", "0", "0", "linear_inspection", "linear_board"],
            # ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.45", "0", "0.0", "1.5707963", "0", "1.5707963", "linear_board", "linear_front"],
            ),
            
            # Linear Far Left (Flat, -8cm height)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.3", "0", "0.08", "0", "0", "0", "linear_front", "front_left"],
            ),
            
            # Linear Far Right (Flat, -8cm height)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0", "0.08", "0", "0", "0", "linear_front", "front_right"],
            ),
            
            # Linear Angled Left (Mid-point, -5cm height, Rolled -30 deg inward)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[ "0.3", "0", "0.08", "0", "-0.436", "0", "linear_front", "angled_left"],
            ),
            
            # Linear Angled Right (Mid-point, -5cm height, Rolled +30 deg inward)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0", "0.08", "0", "0.436", "0", "linear_front", "angled_right"],
            ),

            # Omni Center (Reference) WIP
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0", "0.0", "1.5707963", "0", "1.5707963", "omni_board", "omni_base"],
            ),

            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.0", "0", "0.05", "0.0", "0", "0", "omni_base", "omni_front"],
            ),
            
            # Yaw 45 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.3", "0.2", "0.10", "0.785", "-0.85", "0", "omni_base", "top_left"],
            ),
            
            #  Yaw -45 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0.2", "0.10", "-0.785", "0.85", "0", "omni_base", "top_right"],
            ),
            
            # Yaw 135 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.33", "-0.2", "-0.07", "-3.4", "0.7", "0.3", "omni_base", "bottom_left"],
            ),
            
            # Yaw -135 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.33", "-0.2", "-0.07", "3.4", "-0.7", "0.3", "omni_base", "bottom_right"],
            ),
        ]
    )