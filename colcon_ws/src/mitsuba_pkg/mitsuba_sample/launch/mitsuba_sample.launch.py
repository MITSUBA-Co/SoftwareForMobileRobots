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
from launch_ros.actions import Node


def generate_launch_description():
    mitsuba_sample = get_package_share_directory('mitsuba_sample')
    mitsuba_sample_param = os.path.join(mitsuba_sample, 'yaml', 'mitsuba_sample.yaml')

    sdf_file = os.path.join(mitsuba_sample, 'sdf', 'mitsuba_sample.sdf')

    rviz = os.path.join(mitsuba_sample, 'rviz/mitsuba_sample.rviz')

    return LaunchDescription(
        [
            Node(
                package='mitsuba_sample',
                executable='sample_node',
                parameters=[mitsuba_sample_param],
                output='screen',
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                arguments=[sdf_file],
            ),
            Node(package='rviz2', executable='rviz2', arguments=['-d', rviz]),
        ]
    )
