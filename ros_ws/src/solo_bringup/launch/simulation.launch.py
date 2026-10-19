import os
from pathlib import Path
import socket

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from solo_bringup.configuration import load_config
from solo_bringup.launching import critical, mavros, observed
from solo_bringup.runtime import create_run
from solo_bringup.startup import StartupProgress
from solo_bringup.world import generate_world


def start(context):
    get = lambda name: LaunchConfiguration(name).perform(context)
    config = load_config(get('fleet'))
    profile = get('profile')
    if profile == 'mpc':
        from solo_mpc.settings import load_settings, scenario_fleet
        scenario, _, _ = load_settings(get('scenario'), get('settings'))
        config = scenario_fleet(config, scenario)
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
    progress = StartupProgress(run)
    # The MPC launch extends this same inventory before any process starts.
    context.solo_startup_progress = progress
    progress.stage('Generating Gazebo world and sensor models')
    if profile == 'mpc':
        from solo_mpc.world import generate_mpc_world
        world, _ = generate_mpc_world(config, '/opt/ardupilot_gazebo', run / 'world.sdf',
                                      get('scenario'), get('settings'),
                                      record_overhead=get('record_overhead') == 'true')
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
    server_home = run / 'gazebo-home'
    server_home.mkdir(exist_ok=True)
    gazebo = ExecuteProcess(cmd=gz_command, name='gazebo-server', output='both',
                            additional_env={'HOME': str(server_home)})
    progress.stage('Preparing SITL and MAVROS configuration')
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run / 'logs')),
               critical(gazebo)] + observed(gazebo, progress, 'gazebo-server')
    for uav in config['uavs']:
        cwd = run / 'sitl' / str(uav['instance'])
        cwd.mkdir(parents=True, exist_ok=True)
        sitl = ExecuteProcess(
            cmd=['/opt/ardupilot/build/sitl/bin/arducopter', '-S', '--model', 'JSON',
                 '--speedup', '1', '--sim-address=127.0.0.1', '-I', str(uav['instance']),
                 '--sysid', str(uav['sysid']), '--defaults', defaults],
            cwd=str(cwd), name=f"sitl-{uav['name']}", output='both',
        )
        bridge = mavros(uav, run, simulation=True)
        actions += [critical(sitl)] + observed(sitl, progress, f"sitl-{uav['name']}")
        actions += [critical(bridge)] + observed(bridge, progress, f"mavros-{uav['name']}")
    monitor = Node(
        package='solo_bringup', executable='readiness', name='fleet_readiness',
        output='both', arguments=['--fleet', str(run / 'fleet.yaml'), '--run-dir', str(run),
                                  '--timeout', get('readiness_timeout'), '--state-timeout', '60'],
    )
    actions += [critical(monitor)] + observed(monitor, progress, 'readiness-monitor')
    progress.stage('Starting processes; waiting for fleet telemetry')
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('fleet', default_value=os.environ.get('FLEET_CONFIG', '/workspace/config/fleet.yaml')),
        DeclareLaunchArgument('profile', default_value=os.environ.get('SIM_PROFILE', 'control')),
        DeclareLaunchArgument('scenario', default_value=os.environ.get('MPC_SCENARIO', '')),
        DeclareLaunchArgument('settings', default_value=os.environ.get('MPC_SETTINGS', '')),
        DeclareLaunchArgument('record_overhead', default_value=os.environ.get('MPC_RECORD_OVERHEAD', 'false'), choices=['true', 'false']),
        DeclareLaunchArgument('run_id', default_value=os.environ.get('RUN_ID', '')),
        DeclareLaunchArgument('resume', default_value=os.environ.get('SIM_RESUME', 'false'), choices=['true', 'false']),
        DeclareLaunchArgument('readiness_timeout', default_value=os.environ.get('READINESS_TIMEOUT', '600')),
        OpaqueFunction(function=start),
    ])
