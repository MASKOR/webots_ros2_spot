import os
import pathlib
import launch
from launch import LaunchDescription
from launch.substitutions.path_join_substitution import PathJoinSubstitution
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

table_locations = \
[
    '1.54964 6.2296 0',
    '0.15 7.97 0',
    '0.75 10.69 0',
    '5.08 10.69 0',
    '6.79 8.65 0',
]
can_locations = \
[
    '1.66689 6.23468 0.86107',
    '0.16106 8.08 0.86107',
    '0.754069 10.6 0.86107',
    '5.07304 10.6 0.86107',
    '6.67712 8.65262 0.86107',
]


def generate_launch_description():
    package_dir = get_package_share_directory('webots_spot')

    def load_file(filename):
        return pathlib.Path(os.path.join(package_dir, 'resource', filename)).read_text()

    world = os.path.join(package_dir, 'worlds', 'spot.wbt')
    with open(world, 'r') as f:
        world_txt = f.read()

    modified_world = os.path.join(package_dir, 'worlds', 'modified_spot.wbt')
    with open(modified_world, 'w') as f:
        tags = random.sample(range(0, 20), 5)
        locations = random.sample(range(0, 10), 5)
        externproto_txt = ''
        pairs = ''
        for t, l in zip(tags, locations):
            tag_id = 'tag36_11_' + str(t).zfill(5)

            externproto_txt = externproto_txt + 'EXTERNPROTO "../protos/apriltags/protos/' + tag_id + '.proto"\n'

            apriltag = tag_id + ' {\n'
            apriltag = apriltag + '  translation ' + tag_locations[l][0] + '\n'
            apriltag = apriltag + '  rotation ' + tag_locations[l][1] + '\n}\n'

            pairs = pairs + alphabets[l] + ':' + str(t) + '\n'
            
            world_txt = world_txt + apriltag

        world_txt = world_txt.split('\n')
        world_txt[1] = externproto_txt
        world_txt = '\n'.join(world_txt)

        can_shuffling = random.sample(range(0, 5), 5)

        tables_cans = ''
        idx = 0
        can_tf_publishing_nodes = []
        for table_location,can_location,random_can in \
            zip(table_locations,can_locations,can_shuffling):
            
            idx += 1
            if random_can < 2: # Green Can
                can_color = '0 1 0'
                table_height = 0.8
            elif random_can < 4: # Yellow Can
                can_color = '1 1 0'
                table_height = 0.5
            else: # Red Can
                can_color = '1 0 0'
                table_height = 0.2

            can_xy_tf = can_location.split(' ')
            can_tf_publishing_nodes.append(Node(
                    package='tf2_ros',
                    executable='static_transform_publisher',
                    output='screen',
                    arguments=[str(can_xy_tf[0]), str(can_xy_tf[1]), str(table_height + 0.0611), '0.', '0', '0', 'odom', 'can' + str(idx)],
                ))

            tables_cans += \
"""
Table {
  translation """ + table_location + """
  name \"table_p""" + str(idx) + """\"
  size 0.3 0.3 """ + str(table_height) + """
  feetSize 0.04 0.05
  legAppearance BrushedAluminium {
  }
}
Can {
  translation """ + can_location + """
  rotation 0 0 -2 0
  name \"can_p""" + str(idx) + """\"
  color """ + can_color + """
}"""

        f.write(world_txt + tables_cans)

    with open('pairs.txt', 'w') as f:
        f.write(pairs)

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', 'modified_spot.wbt'])
    )

    spot_driver = Node(
        package='webots_ros2_driver',
        executable='driver',
        output='screen',
        additional_env={'WEBOTS_CONTROLLER_URL': 'Spot'},
        parameters=[
            {'robot_description': load_file('spot.urdf')},
            {'use_sim_time': False},
            {'set_robot_state_publisher': False}, # foot positions are wrong with webot's urdf
        ],
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
    
    apriltag = LaunchConfiguration('detect_tags', default=False)
    spot_apriltag = Node(
        package='webots_spot',
        executable='apriltag_ros',
        output='screen',
        condition=launch.conditions.IfCondition(apriltag),        
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
        webots,
        spot_driver,
        robot_state_publisher,
        # spot_pointcloud2,
        spot_apriltag,
        webots_event_handler,
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['10.46', '8.5', '0', '0', '0', '0', 'odom', 'green_bin'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['10.46', '7.78', '0', '0', '0', '0', 'odom', 'yellow_bin'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            output='screen',
            arguments=['10.46', '7.04', '0', '0', '0', '0', 'odom', 'red_bin'],
        ),
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
    ] + can_tf_publishing_nodes)
