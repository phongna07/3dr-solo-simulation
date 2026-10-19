import copy
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest
import yaml

from solo_bringup.configuration import load_config
from solo_bringup.world import generate_world

ROOT = Path(__file__).resolve().parents[1]


def test_fleet_port_mapping():
    fleet = load_config(ROOT / 'config/fleet.yaml')
    assert [(u['tcp_port'], u['json_port'], u['sysid']) for u in fleet['uavs']] == [
        (5760, 9002, 1), (5770, 9012, 2), (5780, 9022, 3)]


@pytest.mark.parametrize('field,value', [('name', 'uav1'), ('sysid', 1), ('instance', 0),
                                        ('name', '../outside'), ('spawn', [float('nan')] * 6)])
def test_reject_conflicting_or_invalid_fleet(tmp_path, field, value):
    fleet = yaml.safe_load((ROOT / 'config/fleet.yaml').read_text())
    fleet['uavs'][1][field] = value
    path = tmp_path / 'fleet.yaml'
    path.write_text(yaml.safe_dump(fleet))
    with pytest.raises(ValueError):
        load_config(path)


@pytest.fixture
def upstream(tmp_path):
    # Small fixture covers nested IMU names and the gimbal removal/isolation contract.
    source = tmp_path / 'official'
    (source / 'worlds').mkdir(parents=True)
    (source / 'models/iris_with_gimbal').mkdir(parents=True)
    (source / 'models/gimbal_small_3d').mkdir(parents=True)
    (source / 'worlds/iris_runway.sdf').write_text('''<sdf version="1.9"><world name="iris_runway">
      <plugin name="gz::sim::systems::Sensors"/><plugin name="gz::sim::systems::Imu"/>
      <include><uri>model://iris_with_gimbal</uri></include></world></sdf>''')
    (source / 'models/iris_with_gimbal/model.sdf').write_text('''<sdf><model name="iris_with_gimbal">
      <include><uri>model://iris_with_standoffs</uri></include>
      <include><uri>model://gimbal_small_3d</uri><name>gimbal</name><pose>0 0 0 0 0 0</pose></include>
      <joint name="gimbal_joint"/>
      <plugin name="ArduPilotPlugin"><fdm_port_in>9002</fdm_port_in>
        <imuName>iris_with_standoffs::imu_link::imu_sensor</imuName>
        <control channel="0"/><control channel="8"><cmd_topic>/gimbal/cmd_roll</cmd_topic></control>
      </plugin><plugin name="gz::sim::systems::JointPositionController"><topic>/gimbal/cmd_roll</topic></plugin>
      </model></sdf>''')
    (source / 'models/gimbal_small_3d/model.sdf').write_text(
        '<sdf><model name="gimbal"><link name="pitch"><sensor><plugin><udp_port>5600</udp_port>'
        '</plugin><plugin name="CameraZoomPlugin"/></sensor></link></model></sdf>')
    return source


@pytest.mark.parametrize('profile', ['control', 'reference'])
def test_world_preserves_imu_and_matches_fleet(tmp_path, upstream, profile):
    fleet = load_config(ROOT / 'config/fleet.yaml')
    original = copy.deepcopy(fleet)
    output = generate_world(fleet, upstream, tmp_path / 'world.sdf', profile)
    assert fleet == original
    world = ET.parse(output).getroot().find('world')
    models = world.findall('model')
    assert [m.get('name') for m in models] == ['uav1', 'uav2', 'uav3']
    assert [m.findtext("plugin[@name='ArduPilotPlugin']/fdm_port_in") for m in models] == ['9002', '9012', '9022']
    for model in models:
        assert model.findtext("plugin[@name='ArduPilotPlugin']/imuName") == 'iris_with_standoffs::imu_link::imu_sensor'
        if profile == 'control':
            assert model.find('joint') is None
            assert model.find('model') is None
            assert len(model.find("plugin[@name='ArduPilotPlugin']").findall('control')) == 1
        else:
            assert model.find("model[@name='gimbal']") is not None
    if profile == 'reference':
        assert [m.findtext('.//udp_port') for m in models] == ['5600', '5610', '5620']
        assert [m.findtext(".//plugin[@name='CameraZoomPlugin']/topic") for m in models] == [
            '/uav1/camera/cmd_zoom', '/uav2/camera/cmd_zoom', '/uav3/camera/cmd_zoom']


@pytest.mark.parametrize("count", [2, 3, 4])
def test_mission_world_and_bridges_follow_configured_fleet(tmp_path, upstream, count):
    from solo_mpc.settings import load_settings
    from solo_mpc.world import bridge_config, generate_mpc_world

    pkg = ROOT / "ros_ws/src/solo_mpc"
    scenario = yaml.safe_load((pkg / "scenarios/small50_3uav.yaml").read_text())
    scenario["starts"] = (scenario["starts"] + [[0.6, 1.8, 3]])[:count]
    scenario["params"]["GRID_CELLS"] = [[0, 1], [0, -1], [-1, 0]][:count - 1]
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    fleet = yaml.safe_load((ROOT / "config/fleet.yaml").read_text())
    fleet["uavs"] = (fleet["uavs"] + [dict(
        name="uav4", instance=3, sysid=4, spawn=[18, 0, 0.195, 0, 0, 90]
    )])[:count]
    fleet_path = tmp_path / "fleet.yaml"
    fleet_path.write_text(yaml.safe_dump(fleet))
    fleet = load_config(fleet_path)
    settings_path = pkg / "config/controller.yaml"
    load_settings(scenario_path, settings_path)
    output, resolved = generate_mpc_world(
        fleet, upstream, tmp_path / "world.sdf", scenario_path, settings_path
    )
    world = ET.parse(output).getroot().find("world")
    sensors = [s.findtext("topic") for s in world.iter("sensor")]
    assert len(sensors) == len(set(sensors)) == 2 * count
    assert [u["spawn"][:2] for u in resolved["uavs"]] == [
        point[:2] for point in scenario["starts"]
    ]
    bridge = tmp_path / "bridge.yaml"
    bridge_config([u["name"] for u in fleet["uavs"]], fleet["world_name"], bridge)
    rows = yaml.safe_load(bridge.read_text())
    assert len(rows) == 1 + 4 * count
    for u in fleet["uavs"]:
        assert f"/{u['name']}/scan" in [r["ros_topic_name"] for r in rows]
        assert f"/{u['name']}/camera/image_raw" in [r["ros_topic_name"] for r in rows]


def test_mission_world_rejects_mismatched_fleet(tmp_path, upstream):
    from solo_mpc.world import generate_mpc_world

    fleet = load_config(ROOT / "config/fleet.yaml")
    fleet["uavs"].pop()
    pkg = ROOT / "ros_ws/src/solo_mpc"
    with pytest.raises(ValueError, match="same number of UAVs"):
        generate_mpc_world(fleet, upstream, tmp_path / "world.sdf",
                           pkg / "scenarios/small50_3uav.yaml",
                           pkg / "config/controller.yaml")
