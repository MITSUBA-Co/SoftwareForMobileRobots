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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    rviz = os.path.join(mitsuba_launch, 'rviz', 'route_setting.rviz')

    idir = os.path.join(mitsuba_launch, 'map')
    files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]

    # shareディレクトリではなく、srcディレクトリのmapフォルダを取得
    if len(files) > 0:
        mitsuba_launch_src = os.path.dirname(os.path.realpath(os.path.join(idir, files[0])))
    map_file = os.path.join(mitsuba_launch_src, 'map')

    select_map_file = LaunchConfiguration('select_map_file', default=map_file)
    namespace = LaunchConfiguration('namespace', default='')

    push_ros_namespace_cmd = PushRosNamespace(namespace=namespace)

    localization_slam_toolbox_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mitsuba_launch, 'launch', 'localization_slam_toolbox_launch.py')
        ),
        launch_arguments={'select_map_file': select_map_file}.items(),
    )

    rviz_cmd = Node(package='rviz2', executable='rviz2', arguments=['-d', rviz])

    ld = LaunchDescription()
    ld.add_action(push_ros_namespace_cmd)
    ld.add_action(localization_slam_toolbox_cmd)
    ld.add_action(rviz_cmd)

    return ld
