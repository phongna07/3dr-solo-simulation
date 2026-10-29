"""Gazebo/SITL fleet, sensors, recording and independent MPC mission processes."""

import json
import os
import shutil
import socket
from pathlib import Path

import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from solo_bringup.configuration import load_config
from solo_bringup.launching import critical, mavros, observed
from solo_bringup.runtime import create_run, sitl_parameter_file
from solo_bringup.startup import StartupProgress
from solo_mpc.settings import load_settings
from solo_mpc.core.visual_style import OVERHEAD_FPS
from solo_mpc.world import bridge_config, generate_mpc_world


def start(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    fleet = load_config(get("fleet"))
    scenario, settings, cfg = load_settings(get("scenario"), get("settings"))
    if get("target_mode"):
        settings["target_mode"] = get("target_mode")
    if settings["target_mode"] not in ("camera", "ground_truth"):
        raise ValueError("Invalid target_mode")
    record_video = get("record_video") == "true"
    camera_enabled = settings["target_mode"] == "camera" or record_video
    if len(fleet["uavs"]) != len(scenario["starts"]):
        raise ValueError("Fleet/scenario UAV count mismatch")
    for uav, point in zip(fleet["uavs"], scenario["starts"]):
        uav["spawn"] = [point[0], point[1], 0.195, 0, 0, 90]
    fleet["world_name"] = scenario.get("world_name", "mpc_small50")
    # All FCU ports stay inside the simulation container.
    for uav in fleet['uavs']:
        for kind, port in ((socket.SOCK_STREAM, uav['tcp_port']),
                           (socket.SOCK_DGRAM, uav['json_port'])):
            with socket.socket(socket.AF_INET, kind) as sock:
                try:
                    sock.bind(('127.0.0.1', port))
                except OSError as error:
                    raise RuntimeError(f"{uav['name']}: port {port} occupied") from error
    run = create_run(fleet, get('run_id'))
    progress = StartupProgress(run)
    progress.stage('Generating Gazebo world and sensor models')
    world = generate_mpc_world(
        fleet, '/opt/ardupilot_gazebo', run / 'config/world.sdf',
        scenario, settings, cfg, record_video=record_video,
    )
    defaults = ','.join(str(Path('/opt/ardupilot/Tools/autotest/default_params') / name)
                        for name in ('copter.parm', 'gazebo-iris.parm'))
    for path in defaults.split(','):
        if not Path(path).is_file():
            raise RuntimeError(f'Missing defaults: {path}')
    if settings.get('sitl_params'):
        defaults += ',' + sitl_parameter_file(settings['sitl_params'], run / 'config/sitl-overrides.parm')
    server_home = run / 'gazebo-home'
    server_home.mkdir()
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '--headless-rendering', '-s', '-r', '-v', '3', str(world)],
        name='gazebo-server', output='both', additional_env={'HOME': str(server_home)},
    )
    progress.stage('Preparing SITL and MAVROS configuration')
    actions = [SetEnvironmentVariable('ROS_LOG_DIR', str(run / 'logs')),
               critical(gazebo)] + observed(gazebo, progress, 'gazebo-server')
    for uav in fleet['uavs']:
        cwd = run / 'sitl' / str(uav['instance'])
        cwd.mkdir(parents=True)
        sitl = ExecuteProcess(
            cmd=['/opt/ardupilot/build/sitl/bin/arducopter', '-S', '--model', 'JSON',
                 '--speedup', '1', '--sim-address=127.0.0.1', '-I', str(uav['instance']),
                 '--sysid', str(uav['sysid']), '--defaults', defaults],
            cwd=str(cwd), name=f"sitl-{uav['name']}", output='both',
        )
        fcu_bridge = mavros(uav, run)
        actions += [critical(sitl)] + observed(sitl, progress, f"sitl-{uav['name']}")
        actions += [critical(fcu_bridge)] + observed(fcu_bridge, progress, f"mavros-{uav['name']}")
    monitor = Node(
        package='solo_bringup', executable='readiness', name='fleet_readiness',
        output='both', arguments=['--fleet', str(run / 'config/fleet.yaml'), '--run-dir', str(run),
                                  '--timeout', get('readiness_timeout'), '--state-timeout', '60'],
    )
    actions += [critical(monitor)] + observed(monitor, progress, 'readiness-monitor')
    progress.stage('Preparing MPC nodes and sensor bridges')
    # Snapshot the same resolved inputs used for the world and FCU setup.
    shutil.copyfile(get("scenario"), run / "config/scenario.yaml")
    (run / "config/controller.yaml").write_text(yaml.safe_dump(settings, sort_keys=False))
    (run / "config/mpc-config.json").write_text(json.dumps(cfg, indent=2) + "\n")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="mpc_sensor_bridge",
        parameters=[
            {
                "config_file": bridge_config(
                    [u["name"] for u in fleet["uavs"]],
                    fleet["world_name"],
                    run / "config/bridge.yaml",
                    record_overhead=record_video,
                    camera_enabled=camera_enabled,
                )
            }
        ],
        output="both",
    )
    actions += [critical(bridge)] + observed(bridge, progress, 'sensor-bridge')
    args = [
        "--scenario",
        str(run / "config/scenario.yaml"),
        "--settings",
        str(run / "config/controller.yaml"),
        "--fleet",
        str(run / "config/fleet.yaml"),
        "--run-dir",
        str(run),
        "--world",
        fleet["world_name"],
    ]
    for executable in (
        "frames",
        "target",
        "controller",
        "setpoints",
    ):
        node = Node(
            package="solo_mpc",
            executable=executable,
            arguments=args + ['--startup-component', executable],
            parameters=[{"use_sim_time": True}],
            output="both",
        )
        # Keep SITL and MAVROS alive if a controller/perception process exits.
        # The independent publisher expires references; the supervisor can LAND.
        actions += observed(node, progress, executable, initializes=True)
    supervisor = Node(
        package="solo_mpc", executable="mission_supervisor",
        arguments=['--run-dir', str(run), '--startup-component', 'mission-supervisor'],
        parameters=[{"use_sim_time": True}], output="both",
    )
    actions += observed(supervisor, progress, 'mission-supervisor', initializes=True)
    # Each camera callback runs in its own process, independently of the other
    # vehicles and of all optimization workers. Reuse the installed executable.
    for uav in fleet["uavs"] if settings["target_mode"] == "camera" else []:
        name = f"camera-detector-{uav['name']}"
        node = Node(
            package="solo_mpc", executable="marker_detector",
            arguments=args + ["--vehicle", uav["name"], '--startup-component', name],
            parameters=[{"use_sim_time": True}], output="both",
        )
        actions += observed(node, progress, name, initializes=True)
    if record_video:
        # Run the live module so recording needs no image rebuild in development.
        # Disk writes run independently of camera detection and flight control.
        recorder = ExecuteProcess(
            cmd=[
                "python3", "-m", "solo_mpc.camera_recorder", "--combined",
                "--fleet", str(run / "config/fleet.yaml"),
                "--output-dir", str(run), "--fps", str(OVERHEAD_FPS),
                '--run-dir', str(run), '--startup-component', 'combined-recorder',
                "--ros-args", "-p", "use_sim_time:=true",
            ],
            name="mpc-combined-recorder", output="both",
        )
        actions += observed(recorder, progress, 'combined-recorder', initializes=True)
    progress.stage('Starting processes; waiting for fleet telemetry')
    return actions


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("solo_mpc"))
    live = Path("/workspace/ros_ws/src/solo_mpc")
    if os.environ.get("SOLO_DEV") == "1" and live.exists():
        share = live
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "fleet",
                default_value=os.environ.get(
                    "FLEET_CONFIG", "/workspace/config/fleet.yaml"
                ),
            ),
            DeclareLaunchArgument(
                "scenario",
                default_value=os.environ.get(
                    "MPC_SCENARIO", str(share / "scenarios/small50_3uav.yaml")
                ),
            ),
            DeclareLaunchArgument(
                "settings",
                default_value=os.environ.get(
                    "MPC_SETTINGS", str(share / "config/controller.yaml")
                ),
            ),
            DeclareLaunchArgument(
                "record_video",
                default_value=os.environ.get("MPC_RECORD_VIDEO", "false"),
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "target_mode", default_value=os.environ.get("MPC_TARGET_MODE", "")
            ),
            DeclareLaunchArgument("run_id", default_value=os.environ.get("RUN_ID", "")),
            DeclareLaunchArgument(
                "readiness_timeout",
                default_value=os.environ.get("READINESS_TIMEOUT", "600"),
            ),
            OpaqueFunction(function=start),
        ]
    )
