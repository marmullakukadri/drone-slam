"""Drone SLAM simulation: Gazebo Harmonic + ros_gz bridge + RTAB-Map + drone controller + RViz2.

Modes (mode:=...):
  demo          automatic waypoint mission, builds a new map      (maps/<map_name>, default "demo")
  mapping       you fly the drone (keyboard / RViz goals), builds a new map in maps/<map_name>
  localization  loads maps/<map_name>/rtabmap.db, no new map; set the start with
                RViz "2D Pose Estimate" and send the drone with "2D Goal Pose"

Examples:
  ros2 launch drone_slam slam_sim.launch.py                                   # demo
  ros2 launch drone_slam slam_sim.launch.py mode:=mapping map_name:=room1
  ros2 launch drone_slam slam_sim.launch.py mode:=localization map_name:=room1 spawn_x:=5 spawn_y:=-4.5
  ...  gui:=false               # no Gazebo window (RViz only)
  ...  gui:=false rviz:=false   # fully headless
Save a map while running:  ros2 run drone_slam map_saver.py --ros-args -p output_dir:=/workspace/maps/room1
"""
import math
import os
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ('demo', 'mapping', 'localization')
DEFAULT_WORLD = 'shtepia.sdf'   # new maps and the demo mission
OLD_WORLD = 'slam_world.sdf'    # world of maps without a 'world' entry in map_info.yaml


def _sdf_world_name(path):
    """<world name="..."> of an SDF file (ros_gz 'create' needs it)."""
    if not os.path.isfile(path):
        raise RuntimeError(f'World file not found: {path}')
    world = ET.parse(path).getroot().find('world')
    if world is None or not world.get('name'):
        raise RuntimeError(f'No <world name="..."> in {path}')
    return world.get('name')


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _truthy(context, name):
    return _arg(context, name).lower() in ('true', '1', 'yes')


def launch_setup(context):
    pkg = get_package_share_directory('drone_slam')
    ws = _arg(context, 'workspace')
    mode = _arg(context, 'mode')
    if mode not in MODES:
        raise RuntimeError(f"mode:='{mode}' is invalid, use one of {MODES}")
    localization = mode == 'localization'
    gui = _truthy(context, 'gui')

    map_name = _arg(context, 'map_name') or ('demo' if mode == 'demo' else 'map')
    map_dir = os.path.join(ws, 'maps', map_name)
    db_path = os.path.join(map_dir, 'rtabmap.db')
    info_path = os.path.join(map_dir, 'map_info.yaml')

    # world:= empty -> new maps use DEFAULT_WORLD; localization reuses the world
    # the map was built in (map_info.yaml), so older maps keep working.
    world = _arg(context, 'world')
    if not world:
        world = DEFAULT_WORLD
        if localization:
            world = OLD_WORLD  # maps made before map_info.yaml recorded the world
            if os.path.isfile(info_path):
                with open(info_path) as f:
                    world = (yaml.safe_load(f) or {}).get('world', OLD_WORLD)
    if not os.path.isabs(world):
        world = os.path.join(ws, 'worlds', world)
    world_name = _arg(context, 'world_name') or _sdf_world_name(world)

    spawn = {k: float(_arg(context, f'spawn_{k}')) for k in ('x', 'y', 'z', 'yaw')}

    # The SLAM 'map' frame is anchored where the drone started while mapping.
    # Remember that pose so localization runs can relate 'map' to Gazebo 'world'.
    if localization:
        if not os.path.isfile(db_path):
            raise RuntimeError(f'No map to localize in: {db_path} (run mode:=mapping first)')
        origin = {'x': 0.0, 'y': 0.0, 'z': spawn['z'], 'yaw': 0.0}
        if os.path.isfile(info_path):
            with open(info_path) as f:
                origin.update(yaml.safe_load(f).get('map_origin_in_world', {}))
    else:
        os.makedirs(map_dir, exist_ok=True)
        origin = dict(spawn)
        with open(info_path, 'w') as f:
            yaml.safe_dump({'map_name': map_name, 'world': os.path.basename(world),
                            'map_origin_in_world': origin}, f, sort_keys=False)

    # map -> world = inverse(origin pose)
    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    mw = {'x': -(c * origin['x'] + s * origin['y']),
          'y': -(-s * origin['x'] + c * origin['y']),
          'z': -origin['z'], 'yaw': -origin['yaw']}

    rtabmap_cfg = os.path.join(pkg, 'config', 'rtabmap.yaml')
    sim_time = {'use_sim_time': True}
    gz_args = f'-r -v 2 {world}' if gui else f'-r -s -v 2 --headless-rendering {world}'

    actions = [
        LogInfo(msg=f'[drone_slam] mode={mode}  world={os.path.basename(world)}  map={map_dir}  '
                    f'spawn=({spawn["x"]}, {spawn["y"]}, yaw {spawn["yaw"]})'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(ws, 'models')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(ws, 'worlds')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items()),

        Node(package='ros_gz_sim', executable='create', name='spawn_drone', output='screen',
             arguments=['-world', world_name,
                        '-file', os.path.join(ws, 'models', 'slam_drone', 'model.sdf'),
                        '-name', 'drone',
                        '-x', str(spawn['x']), '-y', str(spawn['y']), '-z', str(spawn['z']),
                        '-Y', str(spawn['yaw'])]),

        Node(package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge',
             output='screen',
             parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml'),
                          **sim_time}]),

        # Sensor mounting (must match models/slam_drone/model.sdf)
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_lidar',
             arguments=['--z', '0.10', '--frame-id', 'base_link',
                        '--child-frame-id', 'lidar_link'],
             parameters=[sim_time]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_imu',
             arguments=['--frame-id', 'base_link', '--child-frame-id', 'imu_link'],
             parameters=[sim_time]),
        # Gazebo ground-truth 'world' frame expressed in the SLAM 'map' frame
        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_map_world',
             arguments=['--x', f'{mw["x"]:.6f}', '--y', f'{mw["y"]:.6f}',
                        '--z', f'{mw["z"]:.6f}', '--yaw', f'{mw["yaw"]:.6f}',
                        '--frame-id', 'map', '--child-frame-id', 'world'],
             parameters=[sim_time]),

        # ---- RTAB-Map ----
        Node(package='rtabmap_odom', executable='icp_odometry', name='icp_odometry',
             output='screen', parameters=[rtabmap_cfg],
             remappings=[('scan_cloud', '/lidar/points'), ('odom', '/odom')]),
        Node(package='rtabmap_slam', executable='rtabmap', name='rtabmap', output='screen',
             parameters=[rtabmap_cfg, {
                 'database_path': db_path,
                 'Mem/IncrementalMemory': 'false' if localization else 'true',
                 'Mem/InitWMWithAllNodes': 'true' if localization else 'false',
                 # localization: assume start at map origin until "2D Pose Estimate"
                 'RGBD/StartAtOrigin': 'true' if localization else 'false'}],
             remappings=[('scan_cloud', '/lidar/points'), ('odom', '/odom')],
             arguments=[] if localization else ['-d']),   # -d: start a fresh map

        # ---- drone controller (mission / goals / teleop) ----
        Node(package='drone_slam', executable='drone_node', name='drone_node', output='screen',
             parameters=[_arg(context, 'mission_file'), {'run_mission': mode == 'demo'}],
             remappings=[('odom', '/odom_gt')]),

        # the Gazebo drone model (with spinning propellers) as RViz markers
        Node(package='drone_slam', executable='drone_model.py', name='drone_model',
             output='screen', parameters=[{'workspace': ws, **sim_time}]),

        # floor-filtered copies of /cloud_map for RViz (same filter as map_saver.py)
        Node(package='drone_slam', executable='map_filter.py', name='map_filter',
             output='screen', parameters=[sim_time]),

        Node(package='drone_slam', executable='slam_evaluator.py', name='slam_evaluator',
             output='screen',
             parameters=[{'output_dir': os.path.join(ws, 'results'), **sim_time}]),
    ]

    # Localization: the drone starts at a known place -> give RTAB-Map that start
    # pose automatically (spawn pose expressed in the map frame).
    if localization and _truthy(context, 'auto_initial_pose'):
        dx, dy = spawn['x'] - origin['x'], spawn['y'] - origin['y']
        co, so = math.cos(origin['yaw']), math.sin(origin['yaw'])
        actions.append(Node(
            package='drone_slam', executable='initial_pose.py', name='initial_pose',
            output='screen',
            parameters=[{'x': co * dx + so * dy, 'y': -so * dx + co * dy,
                         'yaw': spawn['yaw'] - origin['yaw'], **sim_time}]))

    if _truthy(context, 'rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
            parameters=[sim_time]))
    return actions


def generate_launch_description():
    pkg = get_package_share_directory('drone_slam')
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='demo', choices=list(MODES),
                              description='demo | mapping | localization'),
        DeclareLaunchArgument('map_name', default_value='',
                              description='Map folder under maps/ (default: demo / map)'),
        DeclareLaunchArgument('workspace', default_value='/workspace',
                              description='Folder that contains models/, worlds/, maps/'),
        DeclareLaunchArgument('world', default_value='',
                              description='World file (absolute or relative to worlds/). '
                                          f'Empty: {DEFAULT_WORLD}, or for localization '
                                          'the world saved in the map\'s map_info.yaml'),
        DeclareLaunchArgument('world_name', default_value='',
                              description='<world name="..."> inside the world file '
                                          '(empty: read from the file)'),
        DeclareLaunchArgument('spawn_x', default_value='0.0'),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value='0.1'),
        DeclareLaunchArgument('spawn_yaw', default_value='0.0', description='radians'),
        DeclareLaunchArgument('gui', default_value='true', description='Gazebo GUI'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('auto_initial_pose', default_value='true',
                              description='localization: set the start pose (spawn_*) '
                                          'automatically instead of "2D Pose Estimate"'),
        DeclareLaunchArgument('mission_file',
                              default_value=os.path.join(pkg, 'config', 'mission.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
