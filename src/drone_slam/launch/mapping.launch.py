import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory('drone_slam'), 'config', 'rtabmap.yaml')
    remap = [('scan_cloud', '/lidar/points'), ('odom', '/odom')]
    return LaunchDescription([
        DeclareLaunchArgument('database_path', default_value='/workspace/maps/map/rtabmap.db'),
        Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
             output='screen', parameters=[cfg], remappings=remap),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[cfg, {
                 'database_path': LaunchConfiguration('database_path'),
                 'Mem/IncrementalMemory': 'true',
                 'Mem/InitWMWithAllNodes': 'false',
                 'RGBD/StartAtOrigin': 'false'}],
             remappings=remap, arguments=['-d']),
    ])
