from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            # ---------------------------------------------------------
            # LINEAR BOARD TRANSFORMS
            # ---------------------------------------------------------
            
            # Linear Center (Reference)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                # Args: x y z yaw pitch roll frame_id child_frame_id
                # Yaw 1.57 (Left), Roll -1.57 (Green Down, Blue Forward)
                arguments=["0", "0", "0", "1.5708", "0", "1.5708", "linear_board", "linear_front"],
            ),
            
            # Linear Far Left (Flat, -8cm height)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.3", "0", "-0.08", "0", "0", "0", "linear_front", "front_left"],
            ),
            
            # Linear Far Right (Flat, -8cm height)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0", "-0.08", "0", "0", "0", "linear_front", "front_right"],
            ),
            
            # Linear Angled Left (Mid-point, -5cm height, Rolled -30 deg inward)
            # Args: x y z yaw pitch roll
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=[ "0.3", "0", "-0.08", "0", "0.523", "0", "linear_front", "angled_left"],
            ),
            
            # Linear Angled Right (Mid-point, -5cm height, Rolled +30 deg inward)
            # Args: x y z yaw pitch roll
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.3", "0", "-0.08", "0", "-0.523", "0", "linear_front", "angled_right"],
            ),

            # ---------------------------------------------------------
            # OMNI BOARD TRANSFORMS
            # ---------------------------------------------------------

            # Omni Center (Reference)
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0", "0", "0", "0.7854", "0", "0", "omni_board", "omni_front"],
            ),
            
            # Omni Top Left (+X, +Y) -> Yaw 45 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.15", "0.15", "-0.05", "0.785", "0.785", "0", "omni_front", "top_left"],
            ),
            
            # Omni Top Right (+X, -Y) -> Yaw -45 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["0.15", "-0.15", "-0.05", "-0.785", "0.785", "0", "omni_front", "top_right"],
            ),
            
            # Omni Bottom Left (-X, +Y) -> Yaw 135 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.15", "0.15", "-0.05", "2.356", "0.785", "0", "omni_front", "bottom_left"],
            ),
            
            # Omni Bottom Right (-X, -Y) -> Yaw -135 deg, Pitch 45 deg
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                output="screen",
                arguments=["-0.15", "-0.15", "-0.05", "-2.356", "0.785", "0", "omni_front", "bottom_right"],
            ),
        ]
    )