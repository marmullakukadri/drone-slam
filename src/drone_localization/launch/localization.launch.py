import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('drone_slam'), 'config', 'rtabmap.yaml')
    remap = [('scan_cloud', '/lidar/points'), ('odom', '/odom')]

    def num(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='/workspace/maps/home/rtabmap.db'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true'),
        Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
             output='screen', parameters=[cfg], remappings=remap),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[cfg, {
                 'database_path': LaunchConfiguration('database_path'),
                 'Mem/IncrementalMemory': 'false',
                 'Mem/InitWMWithAllNodes': 'true',
                 'RGBD/StartAtOrigin': 'true'}],
             remappings=remap),
        Node(package='drone_localization', executable='initial_pose', name='initial_pose',
             output='screen', condition=IfCondition(LaunchConfiguration('auto_initial_pose')),
             parameters=[{'x': num('initial_x'), 'y': num('initial_y'),
                          'yaw': num('initial_yaw'), 'use_sim_time': True}]),
    ])
