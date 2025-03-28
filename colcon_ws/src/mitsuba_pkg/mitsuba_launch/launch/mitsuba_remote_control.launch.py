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
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    home_directory = os.path.expanduser('~')
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    namespace = LaunchConfiguration('namespace', default='')

    return LaunchDescription(
        [
            ExecuteProcess(cmd=['node', 'rosbridge.js'],
                           cwd=os.path.join(home_directory, 'colcon_ws/src/remote_control/ros2-web-bridge/bin'),
                           output='screen'),
            ExecuteProcess(cmd=['node', 'index.js'],
                           cwd=os.path.join(home_directory, 'colcon_ws/src/remote_control/ros2-web-bridge/examples'),
                           output='screen'),
            Node(package='v4l2_camera', executable='v4l2_camera_node', output='screen',
                 parameters=[{'video_device': '/dev/video0'},
                             {'image_size': [640, 480]}]),
            Node(package='fisheye_projection', executable='fisheye_projection_node', output='screen'),
            Node(package='web_video_server', executable='web_video_server', output='screen'),
            PushRosNamespace(namespace=namespace),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(mitsuba_launch, 'launch', 'joystick_run.launch.py')
                ),
            ),
        ]
    )
