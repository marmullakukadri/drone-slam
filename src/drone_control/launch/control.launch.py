import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory('drone_control'), 'config')
    drone_node = Node(
        package='drone_control', executable='drone_node', name='drone_node', output='screen',
        parameters=[LaunchConfiguration('controller_file'),
                    LaunchConfiguration('mission_file'),
                    {'run_mission': ParameterValue(LaunchConfiguration('run_mission'),
                                                   value_type=bool),
                     'require_localization': ParameterValue(
                         LaunchConfiguration('require_localization'), value_type=bool)}],
        remappings=[('odom', '/odom_gt')])
    return LaunchDescription([
        DeclareLaunchArgument('controller_file',
                              default_value=os.path.join(config, 'controller.yaml')),
        DeclareLaunchArgument('mission_file', default_value=os.path.join(config, 'mission.yaml')),
        DeclareLaunchArgument('run_mission', default_value='false'),
        DeclareLaunchArgument('require_localization', default_value='false'),
        drone_node,
        RegisterEventHandler(OnProcessExit(
            target_action=drone_node,
            on_exit=[EmitEvent(event=Shutdown(reason='drone_node stopped'))])),
    ])
