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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo


def generate_launch_description():
    namespace = LaunchConfiguration('namespace', default='')

    mitsuba_launch = get_package_share_directory('mitsuba_launch')

    rviz = os.path.join(mitsuba_launch, 'rviz', 'auto_run.rviz')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'),
                  ('/map', 'map'),
                  ('/map_metadata', 'map_metadata'),]

    push_ros_namespace_cmd = PushRosNamespace(namespace=namespace)

    rviz2_cmd = Node(package='rviz2', executable='rviz2', remappings=remappings, arguments=['-d', rviz])

    ld = LaunchDescription()
    ld.add_action(push_ros_namespace_cmd)
    ld.add_action(rviz2_cmd)

    return ld
