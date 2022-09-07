import os
import pathlib
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from webots_ros2_driver.webots_launcher import WebotsLauncher
import random

alphabets = 'ABCDEFGHIJ'
tag_locations = \
[
    ['-1.25 9.2 0.5', '0 0 1 0'],
    ['0.6 10.85 0.5', '0 0 1 -1.57'],
    ['5.0 10.85 0.5', '0 0 1 -1.57'],
    ['6.95 8.7 0.5', '0 0 1 0'],
    ['10.85 7.7 0.5', '0 0 1 0'],
    ['9.0 6.13 0.5', '0 0 1 -1.57'],
    ['9.4 -1.97 0.5', '0 0 1 -1.57'],
    ['7.78 0.0 0.5', '0 0 1 0'],
    ['6.5 -1.97 0.5', '0 0 1 -1.57'],
    ['1.26 0.8 0.5', '0 0 1 0']
]

def generate_launch_description():
    package_dir = get_package_share_directory('webots_spot')
    robot_description = pathlib.Path(os.path.join(package_dir, 'resource', 'spot.urdf')).read_text()

    world = os.path.join(package_dir, 'worlds', 'spot.wbt')
    with open(world, 'r') as f:
        world_txt = f.read()

    modified_world = os.path.join(package_dir, 'worlds', 'modified_spot.wbt')
    with open(modified_world, 'w') as f:
        tags = random.sample(range(0, 20), 5)
        locations = random.sample(range(0, 10), 5)
        pairs = ''
        for t, l in zip(tags, locations):
            apriltag = 'tag36_11_' + str(t).zfill(5) + ' {\n'
            apriltag = apriltag + '  translation ' + tag_locations[l][0] + '\n'
            apriltag = apriltag + '  rotation ' + tag_locations[l][1] + '\n}'

            pairs = pairs + alphabets[l] + ':' + str(t) + '\n'
            
            world_txt = world_txt + apriltag
        f.write(world_txt)

    with open('pairs.txt', 'w') as f:
        f.write(pairs)

    webots = WebotsLauncher(
        world=modified_world
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
    spot_pointcloud2 = Node(
        package='webots_spot',
        executable='spot_pointcloud2',
        output='screen',
    )
    
    apriltag = LaunchConfiguration('detect_tags', default=False)
    spot_apriltag = Node(
        package='webots_spot',
        executable='apriltag_ros',
        output='screen',
        condition=launch.conditions.IfCondition(apriltag),        
    )
    
    spot_initial_pose = Node(
        package='webots_spot',
        executable='set_initial_pose',
        output='screen',
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
    webots_event_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots,
            on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
        )
    )
    
    return LaunchDescription([
        spot_initial_pose,
        webots,
        spot_driver,
        robot_state_publisher,
        spot_pointcloud2,
        spot_apriltag,
        webots_event_handler,
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/Spot/pointcloud2'), ('scan', '/Spot/scan')],
            parameters=[{
                'target_frame': 'base_link',
                'transform_tolerance': 0.01,
                'min_height': -0.5,
                'max_height': 0.5,
                'angle_min': -3.14,  # -M_PI/2
                'angle_max': 3.14,  # M_PI/2
                'angle_increment': 0.0174,  # M_PI/180.0
                'scan_time': 0.1,
                'range_min': 0.45,
                'range_max': 20.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
    ])
