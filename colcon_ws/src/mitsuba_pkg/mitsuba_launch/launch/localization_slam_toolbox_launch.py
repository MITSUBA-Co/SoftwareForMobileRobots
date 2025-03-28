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

from launch import LaunchDescription
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    idir = os.path.join(mitsuba_launch, 'map')
    files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]

    # shareディレクトリではなく、srcディレクトリのmapフォルダを取得
    if len(files) > 0:
        mitsuba_launch_src = os.path.dirname(os.path.realpath(os.path.join(idir, files[0])))
    map_file = os.path.join(mitsuba_launch_src, 'map')

    select_map_file = LaunchConfiguration('select_map_file', default=map_file)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'),
                  ('/map', 'map'),
                  ('/map_metadata', 'map_metadata'),]

    params_file = os.path.join(mitsuba_launch, 'yaml', 'mapper_params_localization.yaml')
    namespace = LaunchConfiguration('namespace', default='')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)
    
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                parameters=[
                    configured_params,
                    {'map_file_name': select_map_file},
                ],
                remappings=remappings,
                package='slam_toolbox',
                executable='localization_slam_toolbox_node',
                name='slam_toolbox',
                output='screen',
            )
        ]
    )
