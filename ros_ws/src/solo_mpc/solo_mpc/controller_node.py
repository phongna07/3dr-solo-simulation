"""ROS adapter around the independent measured-state TeamController."""

import hashlib
import json
import time
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import PositionTarget, State
from nav_msgs.msg import Path as RosPath
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from .core.controller import TeamController, VehicleState
from .core.parallel_controller import ParallelTeamController
from .core.sensors import PoseHistory, project_scan
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
        self.fcu = {}
        self.histories = {
            n: PoseHistory(args.settings_data["frame_history_tolerance"])
            for n in args.names
        }
        self.observed = {}
        self.last_observation = -np.inf
        self.subscriptions_keep = []
        self.outputs = {}
        self.status_pub = self.create_publisher(String, "/mpc/status", 10)
        self.log = None
        if args.run_dir:
            self.log = open(Path(args.run_dir, "mpc.jsonl"), "a", buffering=1)
            package = Path(__file__).parent
            sources = {
                str(p.relative_to(package)): hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(package.rglob("*.py"))
            }
            with open(
                Path(args.run_dir, "controller_sources.jsonl"), "a"
            ) as source_log:
                source_log.write(
                    json.dumps(
                        {"wall": time.time(), "sha256": sources, "config": args.cfg}
                    )
                    + "\n"
                )
        for n in args.names:
            self.outputs[n] = (
                self.create_publisher(PositionTarget, f"/{n}/mpc/reference", 10),
                self.create_publisher(RosPath, f"/{n}/mpc/prediction", 10),
            )
            for typ, suffix, cb in [
                (PoseStamped, "mpc/pose", self.pose),
                (TwistStamped, "mpc/velocity", self.velocity),
                (LaserScan, "scan", self.scan),
                (State, "mavros/state", self.state),
                (PoseStamped, "target/observation", self.observation),
            ]:
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
        if self.execution == "parallel":
            self.team = ParallelTeamController(
                args.names, args.cfg,
                timeout=args.settings_data.get("controller_worker_timeout_wall", 1.0),
                startup_timeout=args.settings_data.get("controller_worker_startup_timeout_wall", 30.0),
            )
        else:
            self.team = TeamController(args.names, args.cfg)
        try:
            self.create_timer(args.settings_data["control_period"], self.tick)
        except BaseException:
            if isinstance(self.team, ParallelTeamController):
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
        self.poses[n] = (p, q, stamp)
        self.histories[n].add(stamp, p, q)

    def velocity(self, n, msg):
        stamp = seconds(msg.header.stamp)
        v = msg.twist.linear
        v = np.array([v.x, v.y, v.z])
        if not np.isfinite(v).all():
            return
        old = self.velocities.get(n)
        if old and stamp > old[1]:
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
        self.fcu[n] = msg

    def observation(self, n, msg):
        stamp = seconds(msg.header.stamp)
        self.observed[n] = stamp
        if stamp > self.last_observation:
            self.team.track.observe(pose_arrays(msg)[0], stamp)
            self.last_observation = stamp

    def tick(self):
        start = time.perf_counter()
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1e-9
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
            n in self.fcu
            and self.fcu[n].connected
            and self.fcu[n].armed
            and self.fcu[n].mode == "GUIDED"
            for n in names
        ):
            reason = "waiting_for_armed_guided_fleet"
        elif (
            self.team.track.observed_at is None
            or stamp - self.team.track.observed_at
            > settings["target_search_timeout_sim"]
        ):
            reason = "target_unavailable"
        else:
            for n in names:
                p, q, at = self.poses[n]
                v, vt = self.velocities[n]
                scan = self.scans[n]
                if stamp - min(at, vt) > settings["state_timeout_sim"]:
                    reason = "stale_state"
                    break
                if stamp - seconds(scan.header.stamp) > settings["sensor_timeout_sim"]:
                    reason = "stale_scan"
                    break
                states[n] = VehicleState(
                    p, v, self.accelerations.get(n, np.zeros(3)), q, at
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
                        neighbors=[self.poses[k][0] for k in names if k != n],
                        neighbor_radius=cfg["ROBOT_RADIUS"] + 0.15,
                        height_tolerance=settings["scan_height_tolerance"],
                    )
                except ValueError:
                    reason = "scan_pose_unavailable"
                    break
                if len(scans[n].endpoints) < len(scan.ranges) * 0.7:
                    reason = "insufficient_valid_scan"
                    break
            if reason == "ready":
                seers = {
                    n
                    for n, at in self.observed.items()
                    if stamp - at < settings["sensor_timeout_sim"]
                }
                if settings["target_mode"] == "ground_truth":
                    target = self.team.track.x[:2]
                    seers = {
                        n
                        for n in names
                        if max(abs(self.poses[n][0][:2] - target))
                        < cfg["VIEWING_RADIUS"]
                    }
                try:
                    results = self.team.step(states, scans, stamp, seers)
                    if any(not result.get("nominal_ok", result["ok"]) for result in results.values()):
                        reason = "partial_solver_failure"
                except Exception as exc:
                    reason = f"controller_error: {exc}"
                    self.get_logger().error(reason)
        # A completed solve may already be obsolete. Publisher checks age too.
        for n, out in results.items():
            if not out["ok"]:
                continue
            msg = PositionTarget()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = "world"
            msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
            msg.type_mask = PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
            msg.position.x, msg.position.y = map(float, out["position"])
            msg.position.z = settings["flight_altitude"]
            msg.velocity.x, msg.velocity.y = map(float, out["velocity"])
            msg.acceleration_or_force.x, msg.acceleration_or_force.y = map(
                float, out["acceleration"]
            )
            self.outputs[n][0].publish(msg)
            path = RosPath()
            path.header = msg.header
            path.poses = [
                pose_message([x[0], x[1], settings["flight_altitude"]], now.to_msg())
                for x in out["prediction"]
            ]
            self.outputs[n][1].publish(path)
        record = {
            "stamp": stamp,
            "wall": time.time(),
            "compute_ms": 1000 * (time.perf_counter() - start),
            "execution": self.execution,
            "reason": reason,
            "leader": self.team.leader,
            "target_mode": settings["target_mode"],
            "target_observation_age": None
            if self.team.track.observed_at is None
            else stamp - self.team.track.observed_at,
            "target_estimate": None
            if self.team.track.x is None
            else self.team.track.x.tolist(),
            "vehicles": {
                n: {
                    "position": p.tolist(),
                    "velocity": self.velocities.get(n, (np.zeros(3), 0))[0].tolist(),
                    "result": results.get(n, {}).get("reason", reason),
                    "nominal_ok": results.get(n, {}).get("nominal_ok", False),
                    "solver_status": results.get(n, {}).get("solver_status"),
                    "solver_ms": results.get(n, {}).get("solver_ms"),
                    **{
                        key: results.get(n, {}).get(key)
                        for key in (
                            "worker_pid", "cycle", "mapping_ms", "path_ms", "corridor_ms",
                            "solver_iterations", "planning_and_solve_ms",
                            "map_started_wall", "map_finished_wall",
                            "solve_started_wall", "solve_finished_wall", "corridor_faces",
                        )
                    },
                    "armed": bool(n in self.fcu and self.fcu[n].armed),
                    "mode": self.fcu[n].mode if n in self.fcu else None,
                }
                for n, (p, q, at) in self.poses.items()
            },
        }
        text = json.dumps(record)
        self.status_pub.publish(String(data=text))
        if self.log:
            self.log.write(text + "\n")

    def destroy_node(self):
        if isinstance(self.team, ParallelTeamController):
            self.team.close()
        if self.log:
            self.log.close()
        return super().destroy_node()


def main():
    run_node(
        Controller, "Multi-UAV MPC using measured ROS states and sensor observations"
    )
