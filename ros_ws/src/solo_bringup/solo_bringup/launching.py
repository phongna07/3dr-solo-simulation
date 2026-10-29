"""Simulation process lifecycle and pinned MAVROS configuration."""
from pathlib import Path
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from launch.actions import EmitEvent, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.events import Shutdown
from launch_ros.actions import Node

import yaml


def observed(action, progress, name, initializes=False):
    """Register lifecycle callbacks before executing the process action."""
    progress.register(name, initializes)

    def started(event, context):
        progress.running(name, event.pid)

    def exited(event, context):
        progress.exited(name, event.returncode)

    return [
        RegisterEventHandler(OnProcessStart(target_action=action, on_start=started)),
        RegisterEventHandler(OnProcessExit(target_action=action, on_exit=exited)),
        action,
    ]


def critical(action):
    return RegisterEventHandler(OnProcessExit(
        target_action=action,
        on_exit=[LogInfo(msg='Critical process exited; shutting down stack.'),
                 EmitEvent(event=Shutdown(reason='critical process exited'))],
    ))


def mavros(uav, run):
    share = Path(get_package_share_directory('mavros'))
    version = ET.parse(share / 'package.xml').getroot().findtext('version')
    if version != '2.15.1':
        raise RuntimeError(f'MAVROS runtime {version}; expected 2.15.1. Source the pinned overlay via solo-entrypoint.')
    return Node(
        package='mavros', executable='mavros_node', namespace=uav['name'],
        output='both', parameters=[
            str(share / 'launch/apm_pluginlists.yaml'),
            mavros_config(uav, run / "config" / f"{uav['name']}-mavros.yaml"),
            {'fcu_url': uav['fcu_url'], 'gcs_url': '', 'tgt_system': uav['sysid'],
             'base_link_frame': f"{uav['name']}/base_link", 'map_frame': f"{uav['name']}/map",
             'odom_frame': f"{uav['name']}/odom",
             'tgt_component': 1, 'fcu_protocol': 'v2.0', 'use_sim_time': False},
        ],
    )


def mavros_config(uav, destination):
    source = Path(get_package_share_directory('mavros')) / 'launch/apm_config.yaml'
    data = yaml.safe_load(source.read_text())
    # FCU heartbeats run on Gazebo time, but this MAVROS version uses a
    # wall timer for connection loss. Leave room for slow rendering.
    data['/**/sys']['ros__parameters']['conn_timeout'] = 60.0
    # The simulation does not use FCU parameter services: SITL defaults are
    # loaded directly. Avoid background pulls/retries competing with flight
    # telemetry, especially after a heartbeat reconnection.
    plugins = yaml.safe_load((source.parent / 'apm_pluginlists.yaml').read_text())
    denylist = plugins['/**']['ros__parameters']['plugin_denylist']
    data['/**']['ros__parameters']['plugin_denylist'] = list(dict.fromkeys([*denylist, 'param']))
    # A single clock offset cannot synchronize a slow simulated FCU with
    # wall time. Adapters already timestamp measurements on /clock receipt.
    data['/**/time']['ros__parameters'].update(
        timesync_mode='NONE', timesync_rate=0.0, system_time_rate=0.0)
    replacements = {name: f"{uav['name']}/{name}" for name in (
        'map', 'odom', 'earth', 'base_link', 'fix', 'lidar', 'camera_center',
        'target_position', 'target_attitude', 'vision_estimate', 'landing_target',
    )}

    def rewrite(value):
        if isinstance(value, dict):
            return {k: replacements.get(v, v) if 'frame' in k and isinstance(v, str)
                    else rewrite(v) for k, v in value.items()}
        if isinstance(value, list):
            return [rewrite(v) for v in value]
        return value

    Path(destination).write_text(yaml.safe_dump(rewrite(data), sort_keys=False))
    return str(destination)
