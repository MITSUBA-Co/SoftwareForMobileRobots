from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            name='scanner', default_value='scanner',
            description='Namespace for sample topics'
        ),
        
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/hokuyo_cloud2'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': 'base_scan',
                'transform_tolerance': 0.01,
                'min_height': 0.1,
                'max_height': 1.0,
                'angle_min': -1.833,  # -M_PI/2
                'angle_max': 1.833,  # M_PI/2
                'angle_increment': 0.00436332313,  # 0.25[deg]
                'scan_time': 0.3333,
                'range_min': 0.2,
                'range_max': 35.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        )
    ])
