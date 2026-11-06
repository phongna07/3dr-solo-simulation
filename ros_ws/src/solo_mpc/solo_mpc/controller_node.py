"""ROS adapter around the independent measured-state TeamController."""

import hashlib
import json
import threading
import time
from array import array
from dataclasses import replace
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from solo_mpc_interfaces.msg import FlightState
from nav_msgs.msg import OccupancyGrid, Path as RosPath
from rclpy.exceptions import InvalidHandle
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

from .core.algorithm import CycleInput, VehicleState, create_algorithm
from .core.cycles import CycleRunner, result_is_current
from .core.occupancy_video import OCCUPANCY_TOPIC, fuse_occupancy
from .core.sensors import PoseHistory, project_scan
from .core.tracking import ExactTargetTracker
from .core.upstream import provenance
from .ros_utils import (
    pose_arrays,
    pose_message,
    qos_profile_sensor_data,
    run_node,
    seconds,
)


class Controller(Node):
    def __init__(self, args):
        super().__init__("mpc_controller")
        self.args = args
        self.execution = args.settings_data.get("controller_execution", "parallel")
        self.poses = {}
        self.velocities = {}
        self.accelerations = {}
        self.scans = {}
        self.flight = {}
        self.histories = {
            n: PoseHistory(args.settings_data["frame_history_tolerance"])
            for n in args.names
        }
        self.observed = {}
        self.last_observation = -np.inf
        self.subscriptions_keep = []
        self.outputs = {}
        self.trajectories = {}
        self.status_pub = self.create_publisher(String, "/mpc/status", 10)
        self.occupancy_pub = self.create_publisher(
            OccupancyGrid, OCCUPANCY_TOPIC, qos_profile_sensor_data)
        self.log = None
        if args.run_dir:
            for directory in ('telemetry', 'metadata'):
                Path(args.run_dir, directory).mkdir(parents=True, exist_ok=True)
            self.log = open(Path(args.run_dir, "telemetry/mpc.jsonl"), "a", buffering=1)
            package = Path(__file__).parent
            sources = {
                str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(package.rglob("*.py"))
            }
            with open(
                Path(args.run_dir, "metadata/controller_sources.jsonl"), "a"
            ) as source_log:
                source_log.write(
                    json.dumps(
                        {"wall": time.time(), "sha256": sources, "config": args.cfg,
                         "upstream": provenance()}
                    )
                    + "\n"
                )
        for n in args.names:
            self.trajectories[n] = self.create_publisher(JointTrajectory, f"/{n}/mpc/trajectory", 10)
            self.outputs[n] = (
                self.trajectories[n],
                self.create_publisher(RosPath, f"/{n}/mpc/prediction", 10),
            )
            subscriptions = [
                (PoseStamped, "mpc/pose", self.pose),
                (TwistStamped, "mpc/velocity", self.velocity),
                (LaserScan, "scan", self.scan),
                (FlightState, "flight/state", self.state),
            ]
            if args.settings_data["target_mode"] == "camera":
                subscriptions.append((PoseStamped, "target/observation", self.observation))
            for typ, suffix, cb in subscriptions:
                self.subscriptions_keep.append(
                    self.create_subscription(
                        typ,
                        f"/{n}/{suffix}",
                        lambda msg, name=n, fn=cb: fn(name, msg),
                        qos_profile_sensor_data,
                    )
                )
        if args.settings_data["target_mode"] == "ground_truth":
            self.subscriptions_keep.append(
                self.create_subscription(
                    PoseStamped,
                    "/target/ground_truth",
                    lambda msg: self.observation("ground_truth", msg),
                    10,
                )
            )
        cfg = dict(args.cfg)
        if args.run_dir:
            cfg['DIAGNOSTIC_DIR'] = str(Path(args.run_dir, 'optimizer-inputs'))
            cfg['DIAGNOSTIC_CAPTURE_CYCLES'] = args.settings_data.get('diagnostic_capture_cycles', 128)
        self.team = create_algorithm(args.settings_data.get('algorithm', 'mpc'),
                                     args.names, cfg, args.settings_data)
        if args.settings_data["target_mode"] == "ground_truth":
            self.team.track = ExactTargetTracker(
                cfg['KF_PROCESS_NOISE_STD'], cfg['KF_OBS_NOISE_STD'])
        self.cycles = CycleRunner()
        self.cycle_context = None
        self.last_cycle_stamp = -np.inf
        self._completion_lock = threading.Lock()
        self._closing = False
        self.completion_guard = None
        try:
            # Both callbacks use the default mutually exclusive callback group.
            # The timer only schedules starts against simulation time; completed
            # work wakes the executor without waiting for another /clock tick.
            self.completion_guard = self.create_guard_condition(self._collect_cycle)
            self.create_timer(min(0.02, args.settings_data["control_period"]), self.tick)
        except BaseException:
            with self._completion_lock:
                self._closing = True
            self.cycles.close()
            if self.completion_guard is not None:
                self.destroy_guard_condition(self.completion_guard)
            self.team.close()
            raise

    def pose(self, n, msg):
        if msg.header.frame_id != "world":
            return
        p, q = pose_arrays(msg)
        stamp = seconds(msg.header.stamp)
        if (
            not np.isfinite(p).all()
            or not np.isfinite(q).all()
            or np.linalg.norm(q) < 0.9
        ):
            return
        if n in self.poses and stamp <= self.poses[n][2]:
            return
        self.poses[n] = (p, q, stamp)
        self.histories[n].add(stamp, p, q)

    def velocity(self, n, msg):
        stamp = seconds(msg.header.stamp)
        v = msg.twist.linear
        v = np.array([v.x, v.y, v.z])
        if msg.header.frame_id != 'world' or not np.isfinite(v).all():
            return
        old = self.velocities.get(n)
        if old and stamp <= old[1]:
            return
        if old:
            estimate = (v - old[0]) / (stamp - old[1])
            prev = self.accelerations.get(n, np.zeros(3))
            self.accelerations[n] = (
                0.3 * np.clip(estimate, -self.args.cfg["UMAX"], self.args.cfg["UMAX"])
                + 0.7 * prev
            )
        self.velocities[n] = (v, stamp)

    def scan(self, n, msg):
        self.scans[n] = msg

    def state(self, n, msg):
        self.flight[n] = msg

    def observation(self, n, msg):
        sources = ("ground_truth",) if self.args.settings_data["target_mode"] == "ground_truth" else self.args.names
        if n not in sources:
            return
        stamp = seconds(msg.header.stamp)
        now = self.get_clock().now().nanoseconds * 1e-9
        position = pose_arrays(msg)[0]
        if (msg.header.frame_id != 'world' or not np.isfinite(position).all()
                or not 0 <= now - stamp <= self.args.settings_data['target_timeout_sim']):
            return
        if stamp > self.observed.get(n, -np.inf):
            self.observed[n] = stamp
        if self.team.track.observe(position, stamp):
            self.last_observation = stamp

    def _notify_cycle_done(self, _future):
        # Future callbacks can run on the fleet thread (or inline when already
        # done). Never collect results, read live inputs or publish from here.
        with self._completion_lock:
            if self._closing or not self.context.ok():
                return
            try:
                self.completion_guard.trigger()
            except (InvalidHandle, RuntimeError):
                # SIGINT may invalidate the ROS context before destroy_node.
                # Keep unexpected trigger failures visible while ROS is alive.
                if self.context.ok():
                    raise

    def _collect_cycle(self):
        # Guard notifications can coalesce or be stale. The outstanding future
        # and context, consumed before publishing, are the single source of truth.
        if (self._closing or self.cycles.future is None
                or not self.cycles.future.done() or self.cycle_context is None):
            return
        start, now, stamp = self.cycle_context
        self.cycle_context = None
        try:
            cycle = self.cycles.take()
            results = cycle.vehicles
            coordination = cycle.coordination
            reason = ('partial_solver_failure' if any(
                not out.nominal_success for out in results.values()) else 'ready')
        except Exception as exc:
            results, reason = {}, f'controller_error: {exc}'
            coordination = self.team.diagnostics()
            self.get_logger().error(reason)
        current = self.get_clock().now().nanoseconds * 1e-9
        if not result_is_current(stamp, current, self.cycles.started, self.args.settings_data):
            results = {n: replace(out, publication_eligible=False) for n, out in results.items()}
            reason = 'obsolete_cycle'
        elif not all(n in self.flight and self.flight[n].connected and self.flight[n].armed
                     and self.flight[n].mode in ('TAKEOFF', 'TRACK') for n in self.args.names):
            results = {n: replace(out, publication_eligible=False) for n, out in results.items()}
            reason = 'flight_state_changed'
        for n, out in results.items():
            if out.publication_eligible and out.fallback_used:
                p, q, at = self.poses[n]
                v, vt = self.velocities[n]
                measured = VehicleState(p, v, self.accelerations.get(n, np.zeros(3)), q, at, vt)
                results[n] = self.team.revalidate(out, measured)
                if not results[n].publication_eligible:
                    reason = 'partial_solver_failure'
        self._publish_cycle(start, now, stamp, reason, results, self.cycles.timing(), coordination)
        self._publish_occupancy(now)
        if reason == 'ready':
            self.tick()

    def tick(self):
        if self._closing or self.cycles.future is not None:
            return
        start = time.perf_counter()
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
        if 0 <= stamp - self.last_cycle_stamp < self.args.settings_data['control_period'] - 1e-9:
            return
        self.last_cycle_stamp = stamp
        cfg = self.args.cfg
        settings = self.args.settings_data
        names = self.args.names
        reason = "ready"
        states = {}
        scans = {}
        results = {}
        if not all(
            n in self.poses and n in self.velocities and n in self.scans for n in names
        ):
            reason = "waiting_for_measured_state_and_scans"
        elif not all(
            n in self.flight
            and self.flight[n].connected
            and self.flight[n].armed
            and self.flight[n].mode in ('TAKEOFF', 'TRACK')
            for n in names
        ):
            reason = "waiting_for_owned_airborne_fleet"
        elif any(abs(self.poses[name][0][2] - settings['flight_altitude']) > 0.5
                 for name in names):
            reason = 'waiting_for_flight_altitude'
        elif (
            self.team.track.observed_at is None
            or not 0 <= stamp - self.team.track.observed_at <= settings[
                "target_timeout_sim" if settings["target_mode"] == "ground_truth"
                else "target_search_timeout_sim"
            ]
        ):
            reason = "target_unavailable"
        else:
            for n in names:
                p, q, at = self.poses[n]
                v, vt = self.velocities[n]
                scan = self.scans[n]
                if not all(0 <= stamp - value <= settings['state_timeout_sim'] for value in (at, vt)):
                    reason = "stale_state"
                    break
                if not 0 <= stamp - seconds(scan.header.stamp) <= settings['sensor_timeout_sim']:
                    reason = "stale_scan"
                    break
                states[n] = VehicleState(
                    p, v, self.accelerations.get(n, np.zeros(3)), q, at, vt
                )
                try:
                    sp, sq = self.histories[n].at(seconds(scan.header.stamp))
                    from scipy.spatial.transform import Rotation

                    sp = sp + Rotation.from_quat(sq).apply([0.0, 0.0, 0.08])
                    scans[n] = project_scan(
                        scan.ranges,
                        scan.angle_min,
                        scan.angle_increment,
                        scan.range_min,
                        scan.range_max,
                        sp,
                        sq,
                        seconds(scan.header.stamp),
                        neighbors=[self.histories[k].at(seconds(scan.header.stamp))[0]
                                   for k in names if k != n],
                        neighbor_radius=cfg["ROBOT_RADIUS"] + 0.15,
                    )
                except ValueError:
                    reason = "scan_pose_unavailable"
                    break
            if reason == "ready":
                seers = {
                    n
                    for n, at in self.observed.items()
                    if 0 <= stamp - at < settings["sensor_timeout_sim"]
                }
                if settings["target_mode"] == "ground_truth":
                    # Geometric tracking/formation gates need no camera visibility.
                    target = self.team.track.position(stamp)
                    seers = {
                        n
                        for n in names
                        if max(abs(self.poses[n][0][:2] - target))
                        <= cfg["VIEWING_RADIUS"]
                    }
                self.cycle_context = start, now, stamp
                try:
                    self.cycles.submit(self.team.step, CycleInput(states, scans, stamp, seers, self.team.track))
                except Exception as exc:
                    self.cycle_context = None
                    reason = f'controller_error: {exc}'
                    self.get_logger().error(reason)
                else:
                    # Register after submit has stored the future and its frozen
                    # context, including when computation finishes immediately.
                    self.cycles.future.add_done_callback(self._notify_cycle_done)
                    return
        self._publish_cycle(start, now, stamp, reason, results)

    def _publish_occupancy(self, now):
        # Read only after the background fleet cycle completes. Publishing is
        # optional and never changes the workers' independent planning maps.
        if not self.occupancy_pub.get_subscription_count():
            return
        maps = self.team.occupancy_maps()
        if not maps:
            return
        cells = fuse_occupancy(maps)
        msg = OccupancyGrid()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'world'
        msg.info.resolution = float(self.args.cfg['GRID_RESOLUTION'])
        msg.info.height, msg.info.width = cells.shape
        msg.info.origin.position.x, msg.info.origin.position.y = map(
            float, self.args.cfg['WORLD_BOUNDS'][:2])
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', cells.tobytes())
        self.occupancy_pub.publish(msg)

    def _publish_cycle(self, start, now, stamp, reason, results, timing=None, coordination=None):
        settings = self.args.settings_data
        # A completed solve may already be obsolete. Publisher checks age too.
        for n, out in results.items():
            if not out.publication_eligible:
                continue
            trajectory = JointTrajectory()
            trajectory.header.stamp = now.to_msg()
            trajectory.header.frame_id = 'world'
            trajectory.joint_names = ['x', 'y']
            reference = out.trajectory
            if reference is not None:
                controls = reference.accelerations
                for k, position in enumerate(reference.positions):
                    at = int(round(k * reference.timestep * 1e9))
                    trajectory.points.append(JointTrajectoryPoint(
                        positions=list(map(float, position)),
                        velocities=list(map(float, reference.velocities[k])),
                        accelerations=list(map(float, controls[min(k, len(controls) - 1)])),
                        time_from_start=Duration(sec=at // 10**9, nanosec=at % 10**9)))
                self.trajectories[n].publish(trajectory)
            path = RosPath()
            path.header = trajectory.header
            path.poses = [
                pose_message([x[0], x[1], settings["flight_altitude"]], now.to_msg())
                for x in out.prediction.states
            ]
            self.outputs[n][1].publish(path)
        # Callbacks can receive a newer target while this cycle is solving.
        # Freshness describes the live input when reporting, not the solve's
        # earlier timestamp (which could produce a negative observation age).
        reported_at = self.get_clock().now().nanoseconds * 1e-9
        target_age = (None if self.team.track.observed_at is None
                      else reported_at - self.team.track.observed_at)
        statuses = {n: dict(out.diagnostics, reason=out.reason,
                            nominal_ok=out.nominal_success, fallback_used=out.fallback_used,
                            publication_eligible=out.publication_eligible)
                    for n, out in results.items()}
        record = {
            "stamp": stamp,
            "reported_stamp": reported_at,
            "wall": time.time(),
            "compute_ms": 1000 * (time.perf_counter() - start),
            **(timing or {}),
            "execution": self.execution,
            "reason": reason,
            **(self.team.diagnostics() if coordination is None else coordination),
            "target_mode": settings["target_mode"],
            "target_input_fresh": target_age is not None and 0 <= target_age <= settings['target_timeout_sim'],
            "target_observation_age": target_age,
            "target_estimate": None
            if self.team.track.x is None
            else self.team.track.x.tolist(),
            "vehicles": {
                n: {
                    "position": p.tolist(),
                    "velocity": self.velocities.get(n, (np.zeros(3), 0))[0].tolist(),
                    "result": statuses.get(n, {}).get("reason", reason),
                    "nominal_ok": statuses.get(n, {}).get("nominal_ok", False),
                    "fallback_used": statuses.get(n, {}).get("fallback_used", False),
                    "publication_eligible": statuses.get(n, {}).get("publication_eligible", False),
                    "solver_status": statuses.get(n, {}).get("solver_status"),
                    "solver_ms": statuses.get(n, {}).get("solver_ms"),
                    "solver_cpu_ms": statuses.get(n, {}).get("solver_cpu_ms"),
                    **{
                        key: statuses.get(n, {}).get(key)
                        for key in (
                            "worker_pid", "cycle", "mapping_ms", "path_ms", "corridor_ms",
                            "solver_setup_ms", "input_ms", "capture_ms",
                            "scan_stamp", "scan_usable_rays", "scan_hits", "map_updated",
                            "solver_iterations", "planning_and_solve_ms",
                            "map_started_wall", "map_finished_wall",
                            "solve_started_wall", "solve_finished_wall", "corridor_faces",
                        )
                    },
                    "armed": bool(n in self.flight and self.flight[n].armed),
                    "mode": self.flight[n].mode if n in self.flight else None,
                }
                for n, (p, q, at) in self.poses.items()
            },
        }
        text = json.dumps(record)
        self.status_pub.publish(String(data=text))
        if self.log:
            self.log.write(text + "\n")

    def destroy_node(self):
        # Suppress notification before joining the bounded fleet computation.
        # Its callback has finished before super() destroys the guard condition.
        with self._completion_lock:
            self._closing = True
        self.cycles.close()
        self.cycle_context = None
        self.team.close()
        if self.log:
            self.log.close()
        return super().destroy_node()


def main():
    run_node(
        Controller, "Multi-UAV MPC using measured ROS states and sensor observations"
    )
