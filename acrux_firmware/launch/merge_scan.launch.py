#
#   created by: Michael Jonathan (mich1342)
#   github.com/mich1342
#   24/2/2022
#
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        
        launch_ros.actions.Node(
            package='ira_laser_tools',
            executable='laserscan_multi_merger',
            name='laserscan_multi_merger',
            output='screen',
            parameters=[
                {'destination_frame': 'lidar_1'},
                {'cloud_destination_topic': '/cloud'},
                {'scan_destination_topic': '/scan'},
                {'laserscan_topics': '/lidar/scan /camera/scan'},
                {'angle_min': -3.1452},
                {'angle_max': 3.1452},
                {'angle_increment': 0.00437},
                {'scan_time': 0.033333},
                {'range_min': 0.01},
                {'range_max': 6.0}
            ]
        ),
        # Call pointcloud_to_laserscan package
        launch_ros.actions.Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in','/camera/depth/color/points'),
                        ('/scan','/camera/scan')],
            parameters=[{
                'target_frame': 'camera_link',
                'transform_tolerance': 0.01,
                'min_height': -0.06, #height of target frame is considered height 0.0m nad realsense tolerance is 0.05m
                'max_height': 0.95,
                'angle_min': -1.5708,  # -M_PI/2
                'angle_max': 1.5708,  # M_PI/2
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.25,
                'range_max': 3.0,
                'use_inf': True,
                'inf_epsilon': 1.0

            }],
            name='pointcloud_to_laserscan_1'
            ),
        
    ])
