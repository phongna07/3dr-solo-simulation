from glob import glob
import json
from pathlib import Path
from setuptools import find_packages, setup

upstream = Path('../../../external/motion_planning_MPC')
# Unchanged upstream modules executed at runtime; the pin helper hashes the same list.
runtime_sources = json.loads(Path('UPSTREAM.json').read_text())['runtime_sources']
upstream_files = [upstream / name for name in (*runtime_sources, 'LICENSE.md')]
if any(not path.is_file() for path in upstream_files):
    raise RuntimeError('Initialize the MPC submodule: git submodule update --init --recursive')

model_files = {}
for path in sorted(Path('models').rglob('*')):
    if path.is_file():
        model_files.setdefault(str(Path('share/solo_mpc') / path.parent), []).append(str(path))

setup(
    name='solo_mpc', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/solo_mpc']),
        ('share/solo_mpc', ['package.xml', 'LICENSE-MPC', 'UPSTREAM.json']),
        ('share/solo_mpc/upstream', [str(path) for path in upstream_files]),
        ('share/solo_mpc/launch', glob('launch/*.launch.py')),
        ('share/solo_mpc/config', glob('config/*.yaml')),
        ('share/solo_mpc/scenarios', glob('scenarios/*.yaml')),
        *model_files.items(),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Solo project team', maintainer_email='maintainer@example.invalid',
    description='Modular multi-UAV target tracking MPC with Gazebo sensors',
    license='MIT',
    entry_points={'console_scripts': [
        'controller = solo_mpc.controller_node:main',
        'setpoints = solo_mpc.setpoint_node:main',
        'frames = solo_mpc.frame_node:main',
        'flight = solo_mpc.flight_node:main',
        'target = solo_mpc.target_node:main',
        'mission = solo_mpc.mission:main',
        'mission_supervisor = solo_mpc.mission_node:main',
        'evaluate = solo_mpc.evaluate:main',
    ]},
)
