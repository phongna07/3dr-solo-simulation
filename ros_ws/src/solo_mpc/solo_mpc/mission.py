"""Simulation-only mission CLI. Publisher ownership is explicit during takeoff/LAND."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.signals import SignalHandlerOptions
from rclpy.clock import Clock, ClockType
from solo_bringup.flight import Flight
from solo_bringup.startup import startup_blockers
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .ros_utils import pose_arrays, qos_profile_sensor_data, seconds


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["run", "start", "stop", "status", "reset"])
    parser.add_argument("--run-dir", default="/workspace/runs/latest")
    parser.add_argument('--wait-for-startup', action='store_true')
    parser.add_argument('--startup-timeout', type=float, default=600.0)
    parser.add_argument('--cancel-file', type=Path,
                        help='Host cancellation request, polled while ROS is spinning')
    parser.add_argument(
        "--seconds", type=float, default=0.0, help="run only: optional wall-time limit"
    )
    args = parser.parse_args(args)
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")
    if not math.isfinite(args.startup_timeout) or args.startup_timeout <= 0:
        parser.error('--startup-timeout must be finite and positive')
    run = Path(args.run_dir)
    if args.wait_for_startup:
        print('Waiting for all simulation processes and MPC nodes to initialize.', flush=True)
        deadline = time.monotonic() + args.startup_timeout
        while True:
            if args.cancel_file and args.cancel_file.exists():
                print('Mission cancelled before takeoff.', flush=True)
                return
            try:
                startup = json.loads((run / 'startup.json').read_text())
            except (OSError, ValueError):
                startup = None
            try:
                if not startup_blockers(run, startup):
                    break
            except RuntimeError as exc:
                raise SystemExit(str(exc)) from exc
            if time.monotonic() >= deadline:
                raise SystemExit('Timeout: all simulation processes initialized')
            time.sleep(0.1)
    manifest = json.loads((run / "manifest.json").read_text())
    if manifest["mode"] != "simulation" or manifest["profile"] != "mpc":
        raise SystemExit("Mission CLI requires the MPC simulation profile")
    import yaml

    settings = yaml.safe_load((run / "controller.yaml").read_text())
    scenario = yaml.safe_load((run / "scenario.yaml").read_text())
    uavs = manifest["config"]["uavs"]
    names = [u["name"] for u in uavs]
    # Keep ROS alive while KeyboardInterrupt cleanup requests LAND.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Flight(uavs, timeout=240.0)
    clients = {
        name: node.create_client(typ, name)
        for name, typ in [
            ("/mpc/enable", SetBool),
            ("/target/run", SetBool),
            ("/target/reset", Trigger),
        ]
    }
    snapshots = {}
    target = {}
    delivery = {}
    subscriptions = [
        node.create_subscription(
            String,
            "/mpc/status",
            lambda msg: snapshots.update(json.loads(msg.data)),
            10,
        ),
        node.create_subscription(
            PoseStamped,
            "/target/ground_truth",
            lambda msg: target.update(pose=msg, wall=time.monotonic()),
            10,
        ),
        node.create_subscription(
            String,
            "/mpc/setpoint_status",
            lambda msg: delivery.update(json.loads(msg.data), wall=time.monotonic()),
            10,
        ),
    ]
    truth = {}
    truth_log = (
        open(run / "truth.jsonl", "a", buffering=1) if args.command == "run" else None
    )

    def truth_pose(name, msg):
        truth[name] = msg
        if name == names[0] and truth_log and all(n in truth for n in names):
            truth_log.write(
                json.dumps(
                    {
                        "stamp": seconds(msg.header.stamp),
                        "vehicles": {
                            n: {"position": pose_arrays(truth[n])[0].tolist()}
                            for n in names
                        },
                        "target": pose_arrays(target["pose"])[0].tolist()
                        if target
                        else None,
                    }
                )
                + "\n"
            )

    for n in names:
        subscriptions.append(
            node.create_subscription(
                PoseStamped,
                f"/{n}/ground_truth/pose",
                lambda msg, name=n: truth_pose(name, msg),
                qos_profile_sensor_data,
            )
        )

    def call(name, value=None):
        client = clients[name]
        node.wait(client.service_is_ready, f"{name} service", 30.0)
        req = Trigger.Request() if value is None else SetBool.Request(data=value)
        future = client.call_async(req)
        node.wait(future.done, f"{name} reply", 30.0)
        result = future.result()
        if not result or not result.success:
            raise RuntimeError(f"{name}: rejected")

    def stop():
        errors = []
        for service in ("/mpc/enable", "/target/run"):
            try:
                call(service, False)
            except RuntimeError as exc:
                errors.append(str(exc))
        node.land(names)
        if errors:
            raise RuntimeError("; ".join(errors))

    started = False
    cancel_timer = None
    if args.cancel_file:
        def check_cancel():
            if args.cancel_file.exists():
                # Cleanup also spins ROS; do not interrupt LAND a second time.
                cancel_timer.cancel()
                raise KeyboardInterrupt

        cancel_timer = node.create_timer(0.1, check_cancel, clock=Clock(clock_type=ClockType.STEADY_TIME))
    try:
        if args.command == "status":
            node.wait(lambda: bool(snapshots), "controller status", 30.0)
            print(json.dumps(snapshots, indent=2))
            return
        if args.command == "reset":
            call("/target/reset")
            return
        if args.command == "stop":
            stop()
            return
        # Freeze target and flight publisher during takeoff so velocity references
        # cannot override ArduPilot's takeoff controller.
        call("/mpc/enable", False)
        call("/target/reset")

        def fleet_ready():
            try:
                health = json.loads((run / "health.json").read_text())
                return (
                    health["ready"] and time.time() - health["updated_wall_unix"] < 10
                )
            except (FileNotFoundError, json.JSONDecodeError, KeyError):
                return False

        node.wait(fleet_ready, "fleet preflight readiness", 240.0)
        node.wait(
            lambda: (
                (run / "alignment.json").exists()
                and len(json.loads((run / "alignment.json").read_text())) == len(names)
            ),
            "common world alignment",
            90.0,
        )
        started = True
        node.takeoff(names, settings["flight_altitude"])
        node.wait(
            lambda: (
                snapshots.get("reason") == "ready"
                and all(
                    v.get("result") in ("tracking", "search")
                    for v in snapshots.get("vehicles", {}).values()
                )
                and len(snapshots.get("vehicles", {})) == len(names)
            ),
            "MPC with valid sensors and target",
            90.0,
        )
        call("/mpc/enable", True)
        call("/target/run", True)
        print(
            "MPC mission started: measured-state control, "
            + settings["target_mode"]
            + " target observations.",
            flush=True,
        )
        if args.command == "start":
            started = False
            return
        goal = np.asarray(scenario["target"]["waypoints"][-1])[:2]
        begin = time.monotonic()
        target_paused = False
        failure_since = None
        arrival_since = None
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if (
                target
                and np.linalg.norm(pose_arrays(target["pose"])[0][:2] - goal) < 0.05
            ):
                positions = [
                    np.asarray(v["position"])[:2]
                    for v in snapshots.get("vehicles", {}).values()
                ]
                distances = [np.linalg.norm(p - goal) for p in positions]
                camera_radius = settings["flight_altitude"] * np.tan(
                    settings["camera_hfov"] / 2
                )
                if (
                    len(distances) == len(names)
                    and max(distances) < settings.get("arrival_fleet_radius", 6.0)
                    and min(distances) < camera_radius
                ):
                    break
                arrival_since = (
                    snapshots.get("stamp", 0.0)
                    if arrival_since is None
                    else arrival_since
                )
                if snapshots.get("stamp", 0.0) - arrival_since > settings.get(
                    "arrival_timeout_sim", 30.0
                ):
                    raise RuntimeError(
                        "Target reached its goal but the fleet did not arrive"
                    )
            if args.seconds > 0 and time.monotonic() - begin >= args.seconds:
                break
            if (
                time.monotonic() - target.get("wall", begin)
                > settings["pose_timeout_wall"]
            ):
                raise RuntimeError("Target simulator stopped reporting")
            if (
                time.monotonic() - delivery.get("wall", begin)
                > settings["command_timeout_wall"]
            ):
                raise RuntimeError("Independent setpoint publisher stopped reporting")
            if (
                time.time() - snapshots.get("wall", 0.0)
                > settings["command_timeout_wall"] + 1.0
            ):
                raise RuntimeError("Controller status stopped updating")
            if snapshots.get("reason") == "ready":
                failure_since = None
            else:
                failure_since = (
                    snapshots.get("stamp", 0.0)
                    if failure_since is None
                    else failure_since
                )
                if snapshots.get("stamp", 0.0) - failure_since > settings.get(
                    "planner_failure_timeout_sim", 3.0
                ):
                    raise RuntimeError("Planner inputs or solves remained unavailable")
            # Pause the simulated target during long occlusion while UAVs SEARCH
            # from their last observation. This experiment policy is explicit.
            age = snapshots.get("target_observation_age")
            if age is not None:
                lost = age > settings["target_timeout_sim"]
                if lost != target_paused:
                    call("/target/run", not lost)
                    target_paused = lost
                if age > settings["target_search_timeout_sim"]:
                    raise RuntimeError(
                        "Target could not be reacquired within SEARCH timeout"
                    )
            if not all(
                node.connected(n)
                and node.states[n].armed
                and node.states[n].mode == "GUIDED"
                for n in names
            ):
                raise RuntimeError("Mission lost GUIDED/armed/connection")
            if any(
                node.poses[n].pose.position.z < settings["flight_altitude"] - 0.7
                for n in names
            ):
                raise RuntimeError("Vehicle left the fixed-altitude flight envelope")
        stop()
        started = False
        print("Mission completed and fleet landed.", flush=True)
    except KeyboardInterrupt:
        print(
            "Interrupted; stopping target, releasing publisher and landing.", flush=True
        )
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if cancel_timer:
            cancel_timer.cancel()
        if started:
            try:
                stop()
            except RuntimeError as exc:
                print(f"Mission cleanup failed: {exc}", flush=True)
        node.destroy_node()
        if truth_log:
            truth_log.close()
        if rclpy.ok():
            rclpy.shutdown()
