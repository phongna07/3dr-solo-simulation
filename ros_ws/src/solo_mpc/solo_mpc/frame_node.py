"""Publish Gazebo measured state in common-world ENU at its source timestamp."""

import json
from pathlib import Path

import numpy as np
from geometry_msgs.msg import PoseStamped, TransformStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster

from .core.sensors import stabilized_camera_pose
from .ros_utils import pose_message, qos_profile_sensor_data, run_node


class Frames(Node):
    def __init__(self, args):
        super().__init__('mpc_frames')
        self.args = args
        self.publishers_keep = {}
        self.subscriptions_keep = []
        self.static = StaticTransformBroadcaster(self)
        self.dynamic = TransformBroadcaster(self)
        mounts = []
        for name in args.names:
            mount = TransformStamped()
            mount.header.frame_id = f'{name}/base_link'
            mount.child_frame_id = f'{name}/laser'
            mount.transform.translation.z = 0.08
            mount.transform.rotation.w = 1.0
            mounts.append(mount)
            self.publishers_keep[name] = (
                self.create_publisher(PoseStamped, f'/{name}/mpc/pose', 10),
                self.create_publisher(TwistStamped, f'/{name}/mpc/velocity', 10),
            )
            self.subscriptions_keep.append(self.create_subscription(
                Odometry, f'/{name}/ground_truth/odometry',
                lambda msg, vehicle=name: self.odometry(vehicle, msg), qos_profile_sensor_data))
        self.static.sendTransform(mounts)
        if args.run_dir:
            path = Path(args.run_dir, 'status/alignment.json')
            path.write_text(json.dumps({name: dict(yaw=0.0, translation=[0.0, 0.0, 0.0],
                                                  source='gazebo_world_enu')
                                        for name in args.names}, indent=2) + '\n')

    def odometry(self, name, msg):
        position, quaternion = msg.pose.pose.position, msg.pose.pose.orientation
        coordinates = [position.x, position.y, position.z]
        orientation = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        if (msg.header.frame_id != 'world' or not np.isfinite(coordinates + orientation).all()
                or abs(np.linalg.norm(orientation) - 1.0) > 0.01):
            return
        rotation = Rotation.from_quat(orientation)
        pose = pose_message(coordinates, msg.header.stamp, orientation)
        self.publishers_keep[name][0].publish(pose)
        linear, angular = msg.twist.twist.linear, msg.twist.twist.angular
        velocity = TwistStamped()
        velocity.header = pose.header
        values = rotation.apply([linear.x, linear.y, linear.z])
        velocity.twist.linear.x, velocity.twist.linear.y, velocity.twist.linear.z = map(float, values)
        values = rotation.apply([angular.x, angular.y, angular.z])
        velocity.twist.angular.x, velocity.twist.angular.y, velocity.twist.angular.z = map(float, values)
        self.publishers_keep[name][1].publish(velocity)
        transform = TransformStamped()
        transform.header = pose.header
        transform.child_frame_id = f'{name}/base_link'
        transform.transform.translation.x = position.x
        transform.transform.translation.y = position.y
        transform.transform.translation.z = position.z
        transform.transform.rotation = quaternion
        camera_position, camera_quaternion = stabilized_camera_pose(coordinates)
        camera = TransformStamped()
        camera.header = pose.header
        camera.child_frame_id = f'{name}/camera_optical_frame'
        (camera.transform.translation.x, camera.transform.translation.y,
         camera.transform.translation.z) = map(float, camera_position)
        (camera.transform.rotation.x, camera.transform.rotation.y,
         camera.transform.rotation.z, camera.transform.rotation.w) = map(float, camera_quaternion)
        self.dynamic.sendTransform([transform, camera])


def main():
    run_node(Frames, 'Adapt Gazebo ground truth to common-world measured states')
