import os
from pathlib import Path
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from solo_bringup.configuration import load_config
from solo_bringup.launching import critical, mavros
from solo_bringup.runtime import create_run
from solo_bringup.world import generate_world


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    config = load_config(get('fleet'))
    profile = get('profile')
    if profile == 'mpc':
        from solo_mpc.settings import load_settings
        scenario, _, _ = load_settings(get('scenario'), get('settings'))
        if len(config['uavs']) != len(scenario['starts']):
            raise ValueError('Fleet/scenario UAV count mismatch')
        config['world_name'] = 'mpc_small50'
        for uav, point in zip(config['uavs'], scenario['starts']):
            uav['spawn'] = [point[0], point[1], 0.195, 0, 0, 90]
    # All ports live inside one container; no host port mapping is needed.
    for uav in config['uavs']:
        for kind, port in ((socket.SOCK_STREAM, uav['tcp_port']),
                           (socket.SOCK_DGRAM, uav['json_port'])):
            with socket.socket(socket.AF_INET, kind) as sock:
                try:
                    sock.bind(('127.0.0.1', port))
                except OSError as error:
                    raise RuntimeError(f"{uav['name']}: port {port} occupied") from error
    run = create_run(config, 'simulation', profile, get('run_id'), get('resume') == 'true')
    if profile == 'mpc':
        from solo_mpc.world import generate_mpc_world
        world, _ = generate_mpc_world(config, '/opt/ardupilot_gazebo', run / 'world.sdf',
                                      get('scenario'), get('settings'))
    else:
        world = generate_world(config, '/opt/ardupilot_gazebo', run / 'world.sdf', profile)
    defaults = ','.join(str(Path('/opt/ardupilot/Tools/autotest/default_params') / name)
                        for name in ('copter.parm', 'gazebo-iris.parm'))
    for path in defaults.split(','):
        if not Path(path).is_file():
            raise RuntimeError(f'Missing defaults: {path}')
    gz_command = ['gz', 'sim', '-s', '-r', '-v', '3', str(world)]
    if profile in ('reference', 'mpc'):
        gz_command.insert(2, '--headless-rendering')
    gazebo = ExecuteProcess(cmd=gz_command, name='gazebo-server', output='both')
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run / 'logs')),
               critical(gazebo), gazebo]
    for uav in config['uavs']:
        cwd = run / 'sitl' / str(uav['instance'])
        cwd.mkdir(parents=True, exist_ok=True)
        sitl = ExecuteProcess(
            cmd=['/opt/ardupilot/build/sitl/bin/arducopter', '-S', '--model', 'JSON',
                 '--speedup', '1', '--sim-address=127.0.0.1', '-I', str(uav['instance']),
                 '--sysid', str(uav['sysid']), '--defaults', defaults],
            cwd=str(cwd), name=f"sitl-{uav['name']}", output='both',
        )
        bridge = mavros(uav, run)
        actions += [critical(sitl), sitl, critical(bridge), bridge]
    monitor = Node(
        package='solo_bringup', executable='readiness', name='fleet_readiness',
        output='both', arguments=['--fleet', str(run / 'fleet.yaml'), '--run-dir', str(run),
                                  '--timeout', get('readiness_timeout')],
    )
    actions += [critical(monitor), monitor]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value=os.environ.get('FLEET_CONFIG', '/workspace/config/fleet.yaml')),
        DeclareLaunchArgument('profile', default_value=os.environ.get('SIM_PROFILE', 'control')),
        DeclareLaunchArgument('scenario', default_value=os.environ.get('MPC_SCENARIO', '')),
        DeclareLaunchArgument('settings', default_value=os.environ.get('MPC_SETTINGS', '')),
        DeclareLaunchArgument('run_id', default_value=os.environ.get('RUN_ID', '')),
        DeclareLaunchArgument('resume', default_value=os.environ.get('SIM_RESUME', 'false'), choices=['true', 'false']),
        DeclareLaunchArgument('readiness_timeout', default_value=os.environ.get('READINESS_TIMEOUT', '600')),
        OpaqueFunction(function=start),
    ])
