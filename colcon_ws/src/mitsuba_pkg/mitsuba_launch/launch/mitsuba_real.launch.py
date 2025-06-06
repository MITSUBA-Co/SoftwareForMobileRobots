#!/usr/bin/env python3
# Copyright 2024 MITSUBA Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    urg_node2 = get_package_share_directory('urg_node2')
    #rplidar_ros2 = get_package_share_directory('sllidar_ros2')
    #urg3d_node2 = get_package_share_directory('urg3d_node2')

    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription(
        [
            PushRosNamespace(namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mitsuba_launch, 'launch', 'joystick_run.launch.py')
                ),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(urg_node2, 'launch', 'urg_node2.launch.py')
                )
                #PythonLaunchDescriptionSource(os.path.join(rplidar_ros2,'launch','sllidar_launch.py'))
                #PythonLaunchDescriptionSource(os.path.join(urg3d_node2,'launch','urg3d_node2_scan.py'))
            ),
            Node(package='mitsuba_gui', executable='gui_main', output='screen'),
            #Node(package='sound_play', executable='soundplay_node.py', output='screen'),
        ]
    )
