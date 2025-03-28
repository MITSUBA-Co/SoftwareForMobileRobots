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

"""
Example for spawing multiple robots in Gazebo.
This is an example on how to create a launch file for spawning multiple robots into Gazebo
and launch multiple instances of the navigation stack, each controlling one robot.
The robots co-exist on a shared environment and are controlled by independent nav stacks
"""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    mitsuba_sim = get_package_share_directory('mitsuba_sim')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_world = os.path.join(mitsuba_sim, 'sdf', 'world.sdf')
    
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {sdf_world}'}.items(),
    )

    bridge_params = os.path.join(mitsuba_sim, 'yaml', 'bridge_params5.yaml')
        
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
    )    
    
    # Names and poses of the robots
    robots = [
        {'name': 'vehicle1', 'x_pose': 0.0, 'y_pose': 0, 'z_pose': 0.0},
        {'name': 'vehicle2', 'x_pose': 0.0, 'y_pose': 1, 'z_pose': 0.0},
        {'name': 'vehicle3', 'x_pose': 0.0, 'y_pose': 2, 'z_pose': 0.0},
        {'name': 'vehicle4', 'x_pose': 0.0, 'y_pose': 3, 'z_pose': 0.0},
        {'name': 'vehicle5', 'x_pose': 0.0, 'y_pose': 4, 'z_pose': 0.0},
#        {'name': 'vehicle6', 'x_pose': 0.0, 'y_pose': 5, 'z_pose': 0.0}
    ]

    log_settings = LaunchConfiguration('log_settings', default='true')

    robot_spawn_cmds = []
    for robot in robots:
        sdf_robot = os.path.join(mitsuba_sim, 'sdf', 'models', robot['name'], 'robot_model.sdf')
        group = GroupAction([
            Node(package='ros_gz_sim',
                 executable='create',
                 arguments=[
                    '-file',sdf_robot,
                    '-world','ros_ign_tutorial',
                    '-x',f"{robot['x_pose']}",
                    '-y',f"{robot['y_pose']}",
                    '-z',f"{robot['z_pose']}",],
                output='screen',
            ),

            LogInfo(condition=IfCondition(log_settings), msg=['Launching ', robot['name']]),
        ])

        robot_spawn_cmds.append(group)

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(ros_gz_bridge)
    for robot_spawn_cmd in robot_spawn_cmds:
        ld.add_action(robot_spawn_cmd)

    return ld
