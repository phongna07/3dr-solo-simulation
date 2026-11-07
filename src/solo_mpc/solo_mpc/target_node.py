"""Simulation-only target motion, evaluated at Gazebo clock time."""

import json

import numpy as np
from geometry_msgs.msg import PoseStamped
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.pose_pb2 import Pose
from gz.transport13 import Node as GazeboNode
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from .core.planning.target_motion import GOAL_TOLERANCE, startup_route_time, target_route
from .ros_utils import pose_message, run_node


class MovingTarget(Node):
    def __init__(self, args):
        super().__init__("mpc_target")
        self.args = args
        self.running = False
        self.started = 0.0
        self.elapsed = 0.0
        self.gz = GazeboNode()
        self.path, self.times, self.waypoint_times = target_route(args.scenario_data, args.cfg)
        self.publisher = self.create_publisher(PoseStamped, "/target/ground_truth", 10)
        self.progress = self.create_publisher(String, "/target/progress", 10)
        self.create_service(SetBool, "/target/run", self.set_running)
        self.create_service(Trigger, "/target/reset", self.reset)
        self.create_timer(0.05, self.tick)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def set_running(self, req, res):
        if req.data and not self.running:
            self.started = self.now() - self.elapsed
        elif not req.data and self.running:
            self.elapsed = self.now() - self.started
        self.running = req.data
        res.success = True
        res.message = "running" if req.data else "paused"
        return res

    def reset(self, req, res):
        self.running = False
        self.elapsed = 0.0
        self.started = self.now()
        res.success = True
        return res

    def tick(self):
        sampled_at = self.get_clock().now()
        elapsed = sampled_at.nanoseconds * 1e-9 - self.started if self.running else self.elapsed
        route_elapsed = startup_route_time(
            elapsed, self.args.settings_data['target_startup_ramp_sim'])
        point = np.array(
            [
                np.interp(route_elapsed, self.times, self.path[:, 0]),
                np.interp(route_elapsed, self.times, self.path[:, 1]),
                self.args.settings_data["target_height"],
            ]
        )
        request = Pose()
        request.name = "tracking_target"
        request.position.x, request.position.y, request.position.z = map(float, point)
        request.orientation.w = 1.0
        ok, response = self.gz.request(
            f"/world/{self.args.world}/set_pose", request, Pose, Boolean, 200
        )
        if ok and response.data:
            self.publisher.publish(pose_message(point, sampled_at.to_msg()))
        # The start waypoint counts as reached; the mission goal counts as N/N.
        reached = int(np.searchsorted(self.waypoint_times, route_elapsed, side="right"))
        if np.linalg.norm(point[:2] - self.path[-1]) < GOAL_TOLERANCE:
            reached = len(self.waypoint_times)
        self.progress.publish(String(data=json.dumps({
            "waypoints_reached": reached, "waypoints": len(self.waypoint_times),
        })))
        if route_elapsed >= self.times[-1]:
            self.running = False
            self.elapsed = elapsed


def main():
    run_node(MovingTarget, "Move the visible Gazebo target along the scenario spline")
