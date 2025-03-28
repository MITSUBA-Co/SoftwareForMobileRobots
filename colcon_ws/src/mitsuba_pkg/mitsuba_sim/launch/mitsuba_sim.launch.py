import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    mitsuba_launch = get_package_share_directory('mitsuba_launch')
    mitsuba_sim = get_package_share_directory('mitsuba_sim')
    nav2_bringup = get_package_share_directory('nav2_bringup')
    sdf_robot = os.path.join(mitsuba_sim, 'sdf', 'models', 'robot_model.sdf')
    sdf_world = os.path.join(mitsuba_sim, 'sdf', 'world.sdf')
    pkg_ros_ign_gazebo = get_package_share_directory('ros_ign_gazebo')  # add
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_ign_gazebo, 'launch', 'ign_gazebo.launch.py')
        ),
        launch_arguments={'ign_args': '-r ' + sdf_world}.items(),
    )

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='', description='Top-level namespace'
    )

    declare_x_value_cmd = DeclareLaunchArgument(
        'x_value', default_value='0.0', description='Value for x'
    )

    declare_y_value_cmd = DeclareLaunchArgument(
        'y_value', default_value='0.0', description='Value for y'
    )

    declare_z_value_cmd = DeclareLaunchArgument(
        'z_value', default_value='0.0', description='Value for z'
    )

    bridge_params = os.path.join(mitsuba_sim, 'yaml', 'bridge_params.yaml')

    bridge = Node(
        package='ros_ign_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': bridge_params}],
        output='screen'
    )    

    teleop_twist_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mitsuba_launch, 'launch', 'teleop_twist_joy.launch.py')
        ),
    )

    velocity_smoother_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'velocity_smoother_launch.py')
        ),
    )

    run_command_action = ExecuteProcess(
        cmd=[
            'ros2',
            'run',
            'ros_ign_gazebo',
            'create',
            '-file',
            sdf_robot,
            '-world',
            'ros_ign_tutorial',
            '-x',
            LaunchConfiguration('x_value'),
            '-y',
            LaunchConfiguration('y_value'),
            '-z',
            LaunchConfiguration('z_value'),
        ],
        output='screen',
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(ign_gazebo)
    ld.add_action(bridge)
    ld.add_action(teleop_twist_cmd)
    ros_distro = os.getenv('ROS_DISTRO')
    if ros_distro == 'humble':
        ld.add_action(velocity_smoother_cmd)
    ld.add_action(declare_x_value_cmd)
    ld.add_action(declare_y_value_cmd)
    ld.add_action(declare_z_value_cmd)
    ld.add_action(run_command_action)

    return ld
