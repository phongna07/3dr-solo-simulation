"""Existing fleet bringup plus scenario, sensors and independent ROS MPC nodes."""

import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from solo_bringup.configuration import load_config
from solo_bringup.launching import critical
from solo_mpc.settings import load_settings
from solo_mpc.world import bridge_config


def start(context):
    get = lambda n: LaunchConfiguration(n).perform(context)
    if get("resume") == "true":
        raise ValueError(
            "Start a fresh MPC run; old alignment and sensor state cannot be resumed"
        )
    fleet = load_config(get("fleet"))
    scenario, settings, cfg = load_settings(get("scenario"), get("settings"))
    if len(fleet["uavs"]) != len(scenario["starts"]):
        raise ValueError("Fleet/scenario UAV count mismatch")
    for uav, point in zip(fleet["uavs"], scenario["starts"]):
        uav["spawn"] = [point[0], point[1], 0.195, 0, 0, 90]
    fleet["world_name"] = "mpc_small50"
    # Reuse all existing SITL, readiness, critical-process and parameter behavior.
    import importlib.util

    path = Path("/workspace/ros_ws/src/solo_bringup/launch/simulation.launch.py")
    if not path.exists():
        from ament_index_python.packages import get_package_share_directory

        path = Path(
            get_package_share_directory("solo_bringup"), "launch/simulation.launch.py"
        )
    spec = importlib.util.spec_from_file_location("solo_sim_launch", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context.launch_configurations["profile"] = "mpc"
    actions = module.start(context)
    run = Path("/workspace/runs/latest").resolve()
    # Save the actual resolved experiment input alongside the existing manifests.
    import json
    import shutil

    shutil.copyfile(get("scenario"), run / "scenario.yaml")
    import yaml

    if get("target_mode"):
        settings["target_mode"] = get("target_mode")
    if settings["target_mode"] not in ("camera", "ground_truth"):
        raise ValueError("Invalid target_mode")
    (run / "controller.yaml").write_text(yaml.safe_dump(settings, sort_keys=False))
    (run / "mpc-config.json").write_text(json.dumps(cfg, indent=2) + "\n")
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="mpc_sensor_bridge",
        parameters=[
            {
                "config_file": bridge_config(
                    [u["name"] for u in fleet["uavs"]],
                    fleet["world_name"],
                    run / "bridge.yaml",
                )
            }
        ],
        output="both",
    )
    actions += [critical(bridge), bridge]
    args = [
        "--scenario",
        str(run / "scenario.yaml"),
        "--settings",
        str(run / "controller.yaml"),
        "--fleet",
        str(run / "fleet.yaml"),
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
            arguments=args,
            parameters=[{"use_sim_time": True}],
            output="both",
        )
        # Keep SITL and MAVROS alive if a controller/perception process exits.
        # The independent publisher expires references; mission run can LAND.
        actions.append(node)
    # Each camera callback runs in its own process, independently of the other
    # vehicles and of all optimization workers. Reuse the installed executable.
    for uav in fleet["uavs"]:
        actions.append(Node(
            package="solo_mpc", executable="marker_detector",
            arguments=args + ["--vehicle", uav["name"]],
            parameters=[{"use_sim_time": True}], output="both",
        ))
    return actions


def generate_launch_description():
    from ament_index_python.packages import get_package_share_directory

    share = Path(get_package_share_directory("solo_mpc"))
    live = Path("/workspace/ros_ws/src/solo_mpc")
    if live.exists():
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
            DeclareLaunchArgument("profile", default_value="mpc"),
            DeclareLaunchArgument(
                "target_mode", default_value=os.environ.get("MPC_TARGET_MODE", "")
            ),
            DeclareLaunchArgument("run_id", default_value=os.environ.get("RUN_ID", "")),
            DeclareLaunchArgument("resume", default_value="false"),
            DeclareLaunchArgument(
                "readiness_timeout",
                default_value=os.environ.get("READINESS_TIMEOUT", "600"),
            ),
            OpaqueFunction(function=start),
        ]
    )
