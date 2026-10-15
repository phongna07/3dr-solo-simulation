"""Send simulation mission requests to the persistent supervisor."""

import argparse
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from solo_bringup.startup import startup_blockers
from solo_mpc_interfaces.srv import StartMission
from std_srvs.srv import Trigger


TERMINAL = {"idle", "completed", "stopped", "failed"}


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=["run", "start", "stop", "status", "reset"])
    parser.add_argument("--run-dir", default="/workspace/runs/latest")
    parser.add_argument('--wait-for-startup', action='store_true')
    parser.add_argument('--startup-timeout', type=float, default=600.0)
    parser.add_argument('--cancel-file', type=Path,
                        help='Host cancellation request, polled while ROS is spinning')
    parser.add_argument(
        "--seconds", type=float, default=0.0, help="optional tracking wall-time limit for run/start"
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
    run = run.resolve()
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node("simulation_mission_client")
    clients = {
        action: node.create_client(StartMission if action == "start" else Trigger, f"/mission/{action}")
        for action in ("start", "stop", "reset", "status")
    }
    cancel_timer = None
    start_future = None
    mission_id = None

    def wait(condition, description, timeout=30.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if condition():
                return
        raise RuntimeError(f"Timeout: {description}")

    def call(action, request=None):
        nonlocal start_future
        client = clients[action]
        wait(client.service_is_ready, f"mission supervisor {action} service")
        future = client.call_async(request if request is not None else Trigger.Request())
        if action == "start":
            start_future = future
        wait(future.done, f"mission supervisor {action} reply")
        result = future.result()
        if not result or not result.success:
            raise RuntimeError(f"Mission {action}: {result.message if result else 'no reply'}")
        return result

    def status():
        snapshot = json.loads(call("status").message)
        if snapshot["run_dir"] != str(run):
            raise RuntimeError("Mission supervisor belongs to a different simulation run")
        if mission_id is not None and snapshot["mission_id"] != mission_id:
            raise RuntimeError("Mission was replaced by another request")
        return snapshot

    def await_result(return_on_tracking=False):
        while True:
            snapshot = status()
            if snapshot["state"] == "failed":
                raise RuntimeError(snapshot["error"] or "Mission failed")
            if snapshot["state"] in TERMINAL:
                return snapshot
            if return_on_tracking and snapshot["state"] == "tracking":
                return snapshot

    if args.cancel_file:
        def check_cancel():
            if args.cancel_file.exists():
                cancel_timer.cancel()
                raise KeyboardInterrupt

        cancel_timer = node.create_timer(
            0.1, check_cancel, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
    try:
        if args.command == "status":
            print(json.dumps(status(), indent=2))
            return
        if args.command in ("stop", "reset"):
            status()  # Validate the run before sending a mutating request.
            call(args.command)
            await_result()
            return
        result = call("start", StartMission.Request(seconds=args.seconds, run_dir=str(run)))
        mission_id = result.mission_id
        snapshot = await_result(return_on_tracking=args.command == "start")
        if snapshot["state"] == "tracking":
            print("MPC mission started; persistent supervisor is monitoring flight.", flush=True)
        else:
            print("Mission completed and fleet landed.", flush=True)
    except KeyboardInterrupt:
        print("Interrupted; stopping target, releasing publisher and landing.", flush=True)
        if cancel_timer:
            cancel_timer.cancel()
        if start_future is not None:
            try:
                # Cancellation can arrive before the start reply. Only stop a
                # mission this client actually owns; another start may be busy.
                wait(start_future.done, "mission supervisor start reply")
                result = start_future.result()
                if not result or not result.success:
                    return
                mission_id = result.mission_id
                status()
                call("stop")
                await_result()
            except RuntimeError as exc:
                raise SystemExit(f"Mission cleanup failed: {exc}") from exc
    except RuntimeError as exc:
        raise SystemExit(str(exc)) from exc
    finally:
        if cancel_timer:
            cancel_timer.cancel()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
