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
import launch
import launch_ros.actions


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')

    config_filepath = os.path.join(mitsuba_launch, 'yaml', 'f310.config.yaml')
    joy_dev = '/dev/input/js0'

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='joy',
                executable='joy_node',
                name='joy_node',
                parameters=[
                    {
                        'dev': joy_dev,
                        'deadzone': 0.3,
                        'autorepeat_rate': 20.0,
                    }
                ],
            ),
            launch_ros.actions.Node(
                package='teleop_twist_joy',
                executable='teleop_node',
                name='teleop_twist_joy_node',
                parameters=[config_filepath],
                remappings=[('cmd_vel', 'cmd_vel_man')],
            ),
        ]
    )
