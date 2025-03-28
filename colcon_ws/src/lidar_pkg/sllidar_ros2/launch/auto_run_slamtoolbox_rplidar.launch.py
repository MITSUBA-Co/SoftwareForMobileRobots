
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    nav2_bringup = get_package_share_directory('nav2_bringup')
    rplidar_ros2 = get_package_share_directory('sllidar_ros2')
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    params_file = os.path.join(mitsuba_launch, 'yaml', 'nav2_params.yaml')
    rviz = os.path.join(mitsuba_launch, 'rviz','auto_run.rviz')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mitsuba_launch,'launch','joystick_run.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(rplidar_ros2,'launch','sllidar_launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(mitsuba_launch,'launch','navigation_launch.py')),
            launch_arguments={ 'params_file': params_file }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(mitsuba_launch,'launch','localization_slam_toolbox_launch.py')
            )
        ),
        # IncludeLaunchDescription(
            # PythonLaunchDescriptionSource(os.path.join(mitsuba_launch,'launch','collision_monitor_node.launch.py'))
        # ),
        Node( package='rviz2', executable='rviz2', arguments=['-d',rviz] )
    ])
    
