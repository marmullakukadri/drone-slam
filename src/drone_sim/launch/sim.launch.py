import os
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            ExecuteProcess, OpaqueFunction, Shutdown)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _sdf_world_name(path):
    if not os.path.isfile(path):
        raise RuntimeError(f'World file not found: {path}')
    world = ET.parse(path).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'No <world name="..."> in {path}')
    return world.get('name')


def launch_setup(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    pkg = get_package_share_directory('drone_sim')
    ws = arg('workspace')
    world = arg('world')
    if not os.path.isabs(world):
        world = os.path.join(ws, 'worlds', world)
    world_name = arg('world_name') or _sdf_world_name(world)
    gui = arg('gui').lower() in ('true', '1', 'yes')
    gz_args = f'-r -v 2 {world}' if gui else f'-r -s -v 2 --headless-rendering {world}'
    sim_time = {'use_sim_time': True}

    return [
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(ws, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(ws, 'worlds')),

        ExecuteProcess(
            cmd=['gz', 'sim', *gz_args.split(), '--force-version', '8'],
            name='gazebo', output='screen', on_exit=Shutdown(),
            additional_env={'GZ_SIM_SYSTEM_PLUGIN_PATH': os.pathsep.join(
                [os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', ''),
                 os.environ.get('LD_LIBRARY_PATH', '')])}),

        Node(package='ros_gz_sim', executable='create', name='spawn_drone', output='screen',
             arguments=['-world', world_name,
                        '-file', os.path.join(ws, 'models', 'slam_drone', 'model.sdf'),
                        '-name', 'drone',
                        '-x', arg('spawn_x'), '-y', arg('spawn_y'), '-z', arg('spawn_z'),
                        '-Y', arg('spawn_yaw')]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'),
                          **sim_time}]),

        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_lidar',
             arguments=['--z', '0.10', '--frame-id', 'base_link',
                        '--child-frame-id', 'lidar_link'],
             parameters=[sim_time]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
             parameters=[sim_time]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('workspace', default_value='/workspace'),
        DeclareLaunchArgument('world', default_value='home.sdf'),
        DeclareLaunchArgument('world_name', default_value=''),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
        DeclareLaunchArgument('gui', default_value='true'),
        OpaqueFunction(function=launch_setup),
    ])
