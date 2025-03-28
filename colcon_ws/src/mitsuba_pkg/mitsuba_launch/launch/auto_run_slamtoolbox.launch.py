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
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'
    )

    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    mitsuba_sim = get_package_share_directory('mitsuba_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    models_dir = os.path.join(mitsuba_sim, 'sdf', 'models')
    yamls_dir = os.path.join(mitsuba_launch, 'yaml')
    rviz = os.path.join(mitsuba_launch, 'rviz', 'auto_run.rviz')

    idir = os.path.join(mitsuba_launch, 'map')
    files = [f for f in os.listdir(idir) if os.path.isfile(os.path.join(idir, f))]
    if len(files) > 0:
         # shareフォルダではなく、srcフォルダのmapフォルダを取得
        mitsuba_launch_src = os.path.dirname(os.path.realpath(os.path.join(idir, files[0])))
    map_file = os.path.join(mitsuba_launch_src, 'map')
    default_sdf_file = os.path.join(models_dir, 'robot_model.sdf')
    default_params_file = os.path.join(yamls_dir, 'nav2_params.yaml')
    sdf_file = LaunchConfiguration('sdf_file', default=default_sdf_file)
    params_file = LaunchConfiguration('params_file', default=default_params_file)
    select_map_file = LaunchConfiguration('select_map_file', default=map_file)

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static'),
                  ('/scan', 'scan'),
                  ('/map', 'map'),
                  ('/map_metadata', 'map_metadata'),]

    push_ros_namespace_cmd = PushRosNamespace(namespace=namespace)
    navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'mitsuba_navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'params_file': params_file}.items(),
    )
    localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mitsuba_launch, 'launch', 'localization_slam_toolbox_launch.py')
        ),
        launch_arguments={'select_map_file': select_map_file}.items(),
    )
    nav_throuph_poses_csv_cmd = Node(
        package='nav2_simple_commander', executable='nav_throuph_poses_csv', output='screen'
    )
    robot_state_publisher_cmd = Node(
        package='robot_state_publisher', executable='robot_state_publisher', arguments=[sdf_file], remappings=remappings
    )
    rviz2_cmd = Node(package='rviz2', executable='rviz2', remappings=remappings, arguments=['-d', rviz])

    ld = LaunchDescription()
    ld.add_action(push_ros_namespace_cmd)
    ld.add_action(nav_throuph_poses_csv_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(navigation_cmd)
    ld.add_action(localization_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(rviz2_cmd)

    return ld
