import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch argument for map file path
    map_file_path = LaunchConfiguration('map_file_path', default=os.path.join(get_package_share_directory('acrux_navigation'), 'maps', 'my_map.yaml'))

    # Define map_server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_saver_server',
        name='map_saver_server',
        parameters=[{'use_sim_time': False, 'map_path': map_file_path}],  # Set the map_path parameter
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_file_path', default_value=map_file_path, description='Path to save the map file'),

        map_server_node,

        LogInfo(msg=['Map saved at ', map_file_path])
    ])