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
from launch.substitutions import LaunchConfiguration
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    mitsuba_sim = get_package_share_directory('mitsuba_sim')
    rviz = os.path.join(mitsuba_launch, 'rviz', 'create_map.rviz')
    params_file = os.path.join(mitsuba_launch, 'yaml', 'mapper_params_online_async.yaml')
    sdf_file = os.path.join(mitsuba_sim, 'sdf', 'models/robot_model.sdf')

    namespace = LaunchConfiguration('namespace', default='')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True),
        allow_substs=True)
    
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'),
                  ('/map', 'map'),
                  ('/map_metadata', 'map_metadata'),]

    return LaunchDescription(
        [
            PushRosNamespace(namespace=namespace),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                arguments=[sdf_file],
                remappings=remappings,
            ),
            Node(
                package='slam_toolbox',
                executable='async_slam_toolbox_node',
                name='slam_toolbox',
                parameters=[configured_params],
                remappings=remappings,
                output='screen'
            ),
            Node(package='rviz2', executable='rviz2', remappings=remappings, arguments=['-d', rviz]),
        ]
    )
