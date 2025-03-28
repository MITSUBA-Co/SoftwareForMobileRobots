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

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():

    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    params_file = os.path.join(mitsuba_launch, 'yaml', 'robot_params.yaml')
    pkg_prefix = get_package_prefix('mitsuba_diff_drive')
    lib_path = os.path.join(pkg_prefix, 'lib', 'mitsuba_diff_drive')
    

    remappings = [('/tf', 'tf')]

    namespace = LaunchConfiguration('namespace', default='')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)

    return LaunchDescription([
        ExecuteProcess(
            cmd=[
                os.path.join(lib_path, 'logging'),
                '--ros-args', '--params-file', params_file
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                os.path.join(lib_path, 'cmd_to_can'),
                '--ros-args', '--params-file', params_file
            ],
            output='screen'
        ),
        ExecuteProcess(
            cmd=[
                os.path.join(lib_path, 'can_to_odo'),
                '--ros-args', '--params-file', params_file,
                '--remap', '/tf:=tf'
            ],
            output='screen'
        ),
    ])