# Copyright (c) 2018 Intel Corporation
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

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    mitsuba_launch = get_package_share_directory('mitsuba_launch')

    params_file = os.path.join(mitsuba_launch, 'yaml', 'f310.config.yaml')
    joy_dev = '/dev/input/js0'

    namespace = LaunchConfiguration('namespace', default='')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)

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
                parameters=[configured_params],
                remappings=[('cmd_vel', 'cmd_vel_man')],
            ),
        ]
    )
