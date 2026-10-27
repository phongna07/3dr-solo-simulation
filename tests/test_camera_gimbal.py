"""The visibility box and rendered camera must share a centered world frame."""

from collections import deque
import os
from pathlib import Path
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from solo_mpc.core.sensors import pixel_to_ground, stabilized_camera_pose
from solo_mpc.settings import load_settings
from solo_mpc.world import element, sensors, solid

PKG = Path(__file__).resolve().parents[1] / 'ros_ws/src/solo_mpc'


def configuration():
    return load_settings(PKG / 'scenarios/small50_3uav.yaml', PKG / 'config/controller.yaml')


def test_cbf_box_projects_inside_stabilized_camera_with_mount_and_target_plane():
    _, settings, cfg = configuration()
    position = np.array([5., 2., settings['flight_altitude']])
    origin, quaternion = stabilized_camera_pose(position)
    ground_z = settings['target_height'] + .02
    focal = settings['camera_width'] / (2 * np.tan(settings['camera_hfov'] / 2))
    intrinsic = np.array([[focal, 0, 80], [0, focal, 80], [0, 0, 1.]])
    half_box = cfg['CBF_BOX_RATIO'] * cfg['VIEWING_RADIUS']
    # Corners check both axes and all four faces, including the user's (.8, 0)
    # example that was outside the rigidly mounted camera at 15 degrees pitch.
    for offset in [(x, y) for x in (-half_box, half_box) for y in (-half_box, half_box)] + [(.8, 0)]:
        target = np.r_[position[:2] + offset, ground_z]
        optical = Rotation.from_quat(quaternion).inv().apply(target - origin)
        normalized = optical[:2] / optical[2] / (2 * np.tan(settings['camera_hfov'] / 2))
        assert np.max(np.abs(normalized)) < .5
        pixel = (intrinsic @ (optical / optical[2]))[:2]
        np.testing.assert_allclose(pixel_to_ground(pixel, intrinsic, origin, quaternion, ground_z), target)


@pytest.mark.skipif(not Path('/opt/ardupilot_gazebo').exists(),
                    reason='Requires the simulation image and its gimbal plugin')
def test_rendered_camera_cancels_roll_pitch_yaw_and_follows_translation(tmp_path, monkeypatch):
    """Real rendered pixels catch missing plugins and exposure/update-order bugs."""
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node
    import cv2

    _, settings, cfg = configuration()
    monkeypatch.setenv('GZ_PARTITION', f'gimbal-test-{uuid.uuid4().hex}')
    root = ET.Element('sdf', version='1.9')
    world = element(root, 'world', name='gimbal_test')
    # Teleport a freely moving body without falling or starting flight control.
    element(world, 'gravity', '0 0 0')
    physics = element(world, 'physics', name='1ms', type='ignore')
    element(physics, 'max_step_size', .001)
    element(physics, 'real_time_factor', 1)
    for filename, name in [('physics', 'Physics'), ('user-commands', 'UserCommands'), ('sensors', 'Sensors')]:
        plugin = element(world, 'plugin', filename=f'gz-sim-{filename}-system',
                         name=f'gz::sim::systems::{name}')
        if name == 'Sensors':
            element(plugin, 'render_engine', 'ogre2')
    scene = element(world, 'scene')
    element(scene, 'ambient', '1 1 1 1')
    model = element(world, 'model', name='uav1')
    element(model, 'pose', '0 0 3 0 0 0')
    body = element(model, 'model', name='iris_with_standoffs')
    element(body, 'link', name='base_link')
    sensors(model, 'uav1', settings, cfg)
    # No collision occlusion, FCU, ROS bridge, or marker detector is involved.
    # Two asymmetric markers expose yaw rotation as well as pitch/roll drift.
    for name, xy, color in [('tracking_target', (.8, 0), '1 0 1 1'),
                            ('reference', (-.5, .6), '0 1 0 1')]:
        solid(world, name, [*xy, settings['target_height']], 'box', [.12, .12, .04], color)
    path = tmp_path / 'gimbal.sdf'
    ET.ElementTree(root).write(path)
    samples = deque(maxlen=100)
    latest_image = []
    condition = threading.Condition()

    def image(message):
        rgb = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
        rgb = rgb[:, :3 * message.width].reshape(message.height, message.width, 3)
        masks = [(rgb[:, :, 0] > 180) & (rgb[:, :, 1] < 80) & (rgb[:, :, 2] > 180),
                 (rgb[:, :, 0] < 80) & (rgb[:, :, 1] > 100) & (rgb[:, :, 2] < 80)]
        centers = [np.argwhere(mask).mean(axis=0)[::-1] if mask.any() else np.full(2, np.nan)
                   for mask in masks]
        with condition:
            latest_image[:] = [rgb.copy()]
            stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            samples.append((stamp, np.array(centers)))
            condition.notify_all()

    node = Node()
    assert node.subscribe(Image, '/uav1/camera/image_raw', image)
    log = tmp_path / 'gazebo.log'
    with log.open('w') as output:
        process = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', str(path)],
                                   env=os.environ.copy(), stdout=output, stderr=subprocess.STDOUT)
        try:
            def frames_after(stamp, count=5):
                deadline = time.monotonic() + 45
                with condition:
                    while True:
                        frames = [centers for t, centers in samples if t > stamp]
                        if len(frames) >= count:
                            return np.array(frames[:count])
                        assert process.poll() is None, log.read_text()
                        assert time.monotonic() < deadline, log.read_text()
                        condition.wait(timeout=.2)

            baseline = frames_after(-1)[-1]
            cv2.imwrite(str(tmp_path / 'initial.png'), latest_image[0][:, :, ::-1])
            assert np.isfinite(baseline).all(), log.read_text()
            focal = settings['camera_width'] / (2 * np.tan(settings['camera_hfov'] / 2))
            intrinsic = np.array([[focal, 0, 80], [0, focal, 80], [0, 0, 1.]])
            # Include the reported pitch failure, roll, yaw, and combined large
            # rotations. No rendered frames after the pose settles may drift.
            for angles, position in [((0, 15, 0), (0, 0, 3)),
                                     ((-20, 0, 0), (0, 0, 3)),
                                     ((0, 0, 73), (0, 0, 3)),
                                     ((25, -20, 137), (0, 0, 3)),
                                     ((-15, 25, -60), (.2, -.1, 3.4))]:
                request = Pose(name='uav1')
                request.position.x, request.position.y, request.position.z = position
                q = Rotation.from_euler('xyz', angles, degrees=True).as_quat()
                (request.orientation.x, request.orientation.y,
                 request.orientation.z, request.orientation.w) = map(float, q)
                ok, response = node.request('/world/gimbal_test/set_pose', request, Pose, Boolean, 3000)
                assert ok and response.data, log.read_text()
                with condition:
                    # Skip only the render/transport queue at service completion.
                    after = samples[-1][0] + .5
                frames = frames_after(after)
                assert np.isfinite(frames).all(), (angles, frames, log.read_text())
                assert np.max(np.ptp(frames, axis=0)) < .1, (angles, frames)
                if position == (0, 0, 3):
                    np.testing.assert_allclose(frames, np.broadcast_to(baseline, frames.shape), atol=.1)
                origin, optical_q = stabilized_camera_pose(position)
                for pixel, xy in zip(frames[-1], [(.8, 0), (-.5, .6)]):
                    observed = pixel_to_ground(pixel, intrinsic, origin, optical_q,
                                               settings['target_height'] + .02)
                    np.testing.assert_allclose(observed[:2], xy, atol=.025)
        finally:
            node.unsubscribe('/uav1/camera/image_raw')
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=5)
