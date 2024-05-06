import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
import launch_ros

def generate_launch_description():
    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    prefix_address = get_package_share_directory('acrux_navigation') 
    params_directory= os.path.join(prefix_address, 'config', 'nav2_params.yaml')
    map_directory = os.path.join(get_package_share_directory('acrux_bringup'), 'maps','room.yaml')
    
    params_file = LaunchConfiguration('params_file')
    exploration = LaunchConfiguration('exploration') 
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    navigation_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'map_file': map_file,  # Pass map_file argument
            'use_sim_time': use_sim_time  # Pass use_sim_time argument
        }.items()
    )
    
    map_server_node = launch_ros.actions.Node(
        package='nav2_map_server',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'yaml_filename': map_file}]
    )
    
    lifecycle_node = launch_ros.actions.Node(
        package='nav2_lifecycle_manager',
        condition=IfCondition(PythonExpression(['not ', exploration])),
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}, {'autostart': True}, {'node_names': ['map_server']}]
    )
    
    return LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='params_file',
            default_value=params_directory,
            description='Map to be used'
        ),
        launch.actions.DeclareLaunchArgument(
            name='exploration',
            default_value='True',
            description='Flag to enable exploration'
        ),
        launch.actions.DeclareLaunchArgument(
            name='map_file',
            default_value=map_directory,
            description='Map to be used'
        ),
        launch.actions.DeclareLaunchArgument(
            name='use_sim_time',
            default_value='False',
            description='Flag to enable use_sim_time'
        ),
        navigation_launch_cmd,
        map_server_node,
        lifecycle_node
    ])
