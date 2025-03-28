import os
import subprocess
import time
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

#subprocess.call('cansend can0 121#0501010001000001', shell=True)
#time.sleep(0.1)
#subprocess.call('cansend can0 122#0501010001000001', shell=True)
#time.sleep(0.1)
#subprocess.call('cansend can0 0E0#0000000000000000', shell=True)

def generate_launch_description():
    urg3d_node2 = get_package_share_directory('urg3d_node2')
 
    return LaunchDescription([
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource( os.path.join(urg3d_node2,'launch','urg3d_node2.launch.py') ),
		),
		IncludeLaunchDescription(
			PythonLaunchDescriptionSource( os.path.join(urg3d_node2,'launch','sample_pointcloud_to_laserscan_launch.py') ),
		),
    ])
    
