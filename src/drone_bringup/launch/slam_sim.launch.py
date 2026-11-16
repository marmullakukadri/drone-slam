import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MODES = ('demo', 'mapping', 'localization')
DEFAULT_WORLD = 'home.sdf'


def _arg(context, name):
    return LaunchConfiguration(name).perform(context)


def _truthy(context, name):
    return _arg(context, name).lower() in ('true', '1', 'yes')


def _include(pkg, launch_file, **args):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(pkg), 'launch', launch_file)),
        launch_arguments={k: str(v) for k, v in args.items()}.items())


def launch_setup(context):
    pkg = get_package_share_directory('drone_bringup')
    ws = _arg(context, 'workspace')
    mode = _arg(context, 'mode')
    if mode not in MODES:
        raise RuntimeError(f"mode:='{mode}' is invalid, use one of {MODES}")
    localization = mode == 'localization'

    map_name = _arg(context, 'map_name') or ('demo' if mode == 'demo' else 'map')
    map_dir = os.path.join(ws, 'maps', map_name)
    db_path = os.path.join(map_dir, 'rtabmap.db')
    info_path = os.path.join(map_dir, 'map_info.yaml')

    world = _arg(context, 'world')
    if not world:
        world = DEFAULT_WORLD
        if localization and os.path.isfile(info_path):
            with open(info_path) as f:
                world = (yaml.safe_load(f) or {}).get('world', DEFAULT_WORLD)
    if not os.path.isabs(world):
        world = os.path.join(ws, 'worlds', world)

    spawn = {k: float(_arg(context, f'spawn_{k}')) for k in ('x', 'y', 'z', 'yaw')}

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

    c, s = math.cos(origin['yaw']), math.sin(origin['yaw'])
    mw = {'x': -(c * origin['x'] + s * origin['y']),
          'y': -(-s * origin['x'] + c * origin['y']),
          'z': -origin['z'], 'yaw': -origin['yaw']}
    sim_time = {'use_sim_time': True}

    actions = [
        LogInfo(msg=f'[drone_bringup] mode={mode}  world={os.path.basename(world)}  '
                    f'map={map_dir}  spawn=({spawn["x"]}, {spawn["y"]}, yaw {spawn["yaw"]})'),

        _include('drone_sim', 'sim.launch.py', workspace=ws, world=world,
                 world_name=_arg(context, 'world_name'), gui=_arg(context, 'gui'),
                 spawn_x=spawn['x'], spawn_y=spawn['y'], spawn_z=spawn['z'],
                 spawn_yaw=spawn['yaw']),

        Node(package='tf2_ros', executable='static_transform_publisher', name='tf_map_world',
             arguments=['--x', f'{mw["x"]:.6f}', '--y', f'{mw["y"]:.6f}',
                        '--z', f'{mw["z"]:.6f}', '--yaw', f'{mw["yaw"]:.6f}',
                        '--frame-id', 'map', '--child-frame-id', 'world'],
             parameters=[sim_time]),
    ]

    if localization:
        dx, dy = spawn['x'] - origin['x'], spawn['y'] - origin['y']
        co, so = math.cos(origin['yaw']), math.sin(origin['yaw'])
        actions.append(_include(
            'drone_localization', 'localization.launch.py', database_path=db_path,
            initial_x=co * dx + so * dy, initial_y=-so * dx + co * dy,
            initial_yaw=spawn['yaw'] - origin['yaw'],
            auto_initial_pose=_arg(context, 'auto_initial_pose')))
    else:
        actions.append(_include('drone_slam', 'mapping.launch.py', database_path=db_path))

    actions += [
        _include('drone_control', 'control.launch.py',
                 controller_file=_arg(context, 'controller_file'),
                 mission_file=_arg(context, 'mission_file'),
                 run_mission='true' if mode == 'demo' else 'false',
                 require_localization='true' if localization else 'false'),

        Node(package='drone_bringup', executable='drone_model', name='drone_model',
             output='screen', parameters=[{'workspace': ws, **sim_time}]),

        Node(package='drone_slam', executable='map_filter.py', name='map_filter',
             output='screen', parameters=[sim_time]),

        Node(package='drone_slam', executable='slam_evaluator', name='slam_evaluator',
             output='screen',
             parameters=[{'output_dir': os.path.join(ws, 'results'), **sim_time}]),
    ]

    if _truthy(context, 'rviz'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='log',
            arguments=['-d', os.path.join(pkg, 'rviz', 'slam.rviz')],
            parameters=[sim_time]))
    return actions


def generate_launch_description():
    config = os.path.join(get_package_share_directory('drone_control'), 'config')
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
        DeclareLaunchArgument('controller_file',
                              default_value=os.path.join(config, 'controller.yaml')),
        DeclareLaunchArgument('mission_file', default_value=os.path.join(config, 'mission.yaml')),
        OpaqueFunction(function=launch_setup),
    ])
