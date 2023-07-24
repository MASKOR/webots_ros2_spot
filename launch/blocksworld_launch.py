import os
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    gpp = os.path.join(get_package_share_directory('webots_spot'), 'resource')

    gologpp_agent = Node(
        package='gologpp_agent',
        executable='gologpp_agent',
        output='screen',
        parameters=[{'gpp_code': gpp + '/webots_blocksworld.gpp'}],
    )
    
    return LaunchDescription([
        gologpp_agent,
    ])
