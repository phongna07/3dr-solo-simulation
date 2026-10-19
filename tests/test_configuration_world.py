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


@pytest.mark.parametrize("count", [2, 3, 4, 5])
def test_mission_world_and_bridges_follow_configured_fleet(tmp_path, upstream, count):
    from solo_mpc.settings import load_settings
    from solo_mpc.world import bridge_config, generate_mpc_world

    pkg = ROOT / "ros_ws/src/solo_mpc"
    scenario = yaml.safe_load((pkg / "scenarios/small50_3uav.yaml").read_text())
    scenario["starts"] = (scenario["starts"] + [[0.6, 1.8, 3], [10, 2, 3]])[:count]
    scenario["params"]["GRID_CELLS"] = [[0, 1], [0, -1], [-1, 0], [1, 0]][:count - 1]
    scenario_path = tmp_path / "scenario.yaml"
    scenario_path.write_text(yaml.safe_dump(scenario))
    fleet = yaml.safe_load((ROOT / "config/fleet.yaml").read_text())
    fleet["uavs"] = (fleet["uavs"] + [dict(
        name=f"uav{i + 1}", instance=i, sysid=i + 1, spawn=[6 * i, 0, 0.195, 0, 0, 90]
    ) for i in (3, 4)])[:count]
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
    bridge_config([u["name"] for u in resolved["uavs"]], resolved["world_name"], bridge)
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


@pytest.mark.parametrize("record_overhead", [False, True])
def test_big1000_world_has_ground_obstacles_and_sensors_at_field_scale(tmp_path, upstream, record_overhead):
    from solo_mpc.settings import load_settings
    from solo_mpc.world import bridge_config, generate_mpc_world

    pkg = ROOT / "ros_ws/src/solo_mpc"
    scenario_path = pkg / "scenarios/big1000_5uav.yaml"
    settings_path = pkg / "config/controller_big1000.yaml"
    scenario, settings, cfg = load_settings(scenario_path, settings_path)
    fleet = load_config(ROOT / "config/fleet_5uav.yaml")
    original = copy.deepcopy(fleet)
    output, resolved = generate_mpc_world(fleet, upstream, tmp_path / "world.sdf",
                                          scenario_path, settings_path, record_overhead=record_overhead)
    assert fleet == original
    world = ET.parse(output).getroot().find("world")
    assert world.get("name") == resolved["world_name"] == "mpc_big1000"
    ground = world.find("model[@name='field_ground']")
    assert ground.findtext("pose") == "500.0 500.0 0 0 0 0"
    assert list(map(float, ground.findtext("link/collision/geometry/plane/size").split())) == [1100, 1100]
    assert len([m for m in world.findall("model") if m.get("name").startswith("obstacle_")]) == 108
    assert world.findtext("model[@name='obstacle_rect_0']/link/collision/geometry/box/size").endswith("70.0")
    marker = world.find("model[@name='tracking_target']")
    assert marker.findtext("link/visual/geometry/box/size") == "2.0 2.0 0.04"
    overhead = marker.find("link/sensor[@name='overhead_camera']")
    if record_overhead:
        from solo_mpc.scene_view import follow_offset

        pose = list(map(float, overhead.findtext("pose").split()))
        assert tuple(pose[:3]) == follow_offset(settings["flight_altitude"]) == (-100, -100, 240)
        assert float(overhead.findtext("camera/clip/far")) > 300
    else:
        assert overhead is None
    for i, uav in enumerate(resolved["uavs"]):
        model = world.find(f"model[@name='{uav['name']}']")
        assert uav["spawn"][:2] == scenario["starts"][i][:2]
        assert uav["spawn"][2] == 0.195  # actual FCU takeoff, never airborne spawning
        assert model.findtext("plugin[@name='ArduPilotPlugin']/fdm_port_in") == str(9002 + 10 * i)
        laser = model.find("link/sensor[@name='laser']")
        assert float(laser.findtext("ray/range/max")) == cfg["SENSING_RADIUS"] == 50
        assert float(laser.findtext("ray/range/min")) == 0.5
        assert float(laser.findtext("ray/noise/stddev")) == 0.05
        assert float(model.findtext("link/sensor[@name='camera']/camera/horizontal_fov")) == settings["camera_hfov"]
    bridge = tmp_path / "bridge.yaml"
    bridge_config([u["name"] for u in resolved["uavs"]], resolved["world_name"], bridge)
    rows = yaml.safe_load(bridge.read_text())
    assert rows[0]["gz_topic_name"] == "/world/mpc_big1000/clock"
    assert len(rows) == len({r["ros_topic_name"] for r in rows}) == 21


def test_five_uav_launch_inventory_shares_coordination_and_isolates_fcus(tmp_path, upstream, monkeypatch):
    """Construct launch actions and manifests only; never execute those actions."""
    import importlib.util
    import json

    pytest.importorskip("launch_ros")
    import ament_index_python.packages as packages
    from launch import LaunchContext
    from launch.actions import ExecuteProcess
    from launch.utilities import perform_substitutions
    from launch_ros.actions import Node
    from solo_bringup import launching
    from solo_mpc import world as mpc_world

    mavros = tmp_path / "mavros"
    (mavros / "launch").mkdir(parents=True)
    (mavros / "package.xml").write_text("<package><version>2.15.1</version></package>")
    (mavros / "launch/apm_pluginlists.yaml").write_text("'/**':\n  ros__parameters:\n    plugin_denylist: []\n")
    (mavros / "launch/apm_config.yaml").write_text(yaml.safe_dump({
        "/**": {"ros__parameters": {}},
        "/**/sys": {"ros__parameters": {}},
        "/**/time": {"ros__parameters": {}},
        "/**/local_position": {"ros__parameters": {"frame_id": "map"}},
    }))
    shares = {"solo_bringup": ROOT / "ros_ws/src/solo_bringup", "mavros": mavros}
    monkeypatch.setattr(packages, "get_package_share_directory", lambda name: str(shares[name]))
    monkeypatch.setattr(launching, "get_package_share_directory", lambda name: str(shares[name]))
    original_world = mpc_world.generate_mpc_world
    monkeypatch.setattr(mpc_world, "generate_mpc_world",
                        lambda fleet, source, *args, **kwargs: original_world(fleet, upstream, *args, **kwargs))
    original_is_file = Path.is_file
    monkeypatch.setattr(Path, "is_file", lambda path:
                        str(path).startswith("/opt/ardupilot/Tools/autotest/default_params/") or original_is_file(path))
    monkeypatch.setenv("SOLO_DEV", "0")
    monkeypatch.setenv("RUNS_ROOT", str(tmp_path / "runs"))
    pkg = ROOT / "ros_ws/src/solo_mpc"
    context = LaunchContext()
    context.launch_configurations.update({
        "fleet": str(ROOT / "config/fleet_5uav.yaml"),
        "scenario": str(pkg / "scenarios/big1000_5uav.yaml"),
        "settings": str(pkg / "config/controller_big1000.yaml"),
        "resume": "false", "profile": "mpc", "run_id": "offline-five",
        "target_mode": "", "record_cameras": "false", "record_overhead": "false",
        "readiness_timeout": "600",
    })
    spec = importlib.util.spec_from_file_location("test_mpc_launch", pkg / "launch/mpc.launch.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    actions = module.start(context)
    progress = context.solo_startup_progress
    inventory = progress.processes
    assert all(p["state"] == "pending" for p in inventory.values())
    shared = {"gazebo-server", "readiness-monitor", "sensor-bridge", "frames",
              "target", "controller", "setpoints", "mission-supervisor"}
    private = {f"{kind}-uav{i}" for kind in ("sitl", "mavros", "camera-detector") for i in range(1, 6)}
    assert set(inventory) == shared | private
    nodes = [a for a in actions if isinstance(a, Node)]
    assert sum(n.node_executable == "controller" for n in nodes) == 1
    assert sum(n.node_executable == "marker_detector" for n in nodes) == 5
    assert sum(n.node_executable == "mavros_node" for n in nodes) == 5
    for node in nodes:
        if node.node_package != "solo_mpc" or node.node_executable == "mission_supervisor":
            continue
        argv = [perform_substitutions(context, arg) for arg in node.cmd[1:]]
        assert argv[argv.index("--world") + 1] == "mpc_big1000"
    sitls = [a for a in actions if isinstance(a, ExecuteProcess) and not isinstance(a, Node)
             and perform_substitutions(context, a.cmd[0]).endswith("/arducopter")]
    assert len(sitls) == 5
    run = progress.path.parent
    for i, action in enumerate(sitls):
        argv = [perform_substitutions(context, arg) for arg in action.cmd]
        assert argv[argv.index("-I") + 1] == str(i)
        assert argv[argv.index("--sysid") + 1] == str(i + 1)
        assert perform_substitutions(context, action.cwd) == str(run / "sitl" / str(i))
        data = yaml.safe_load((run / f"uav{i + 1}-mavros.yaml").read_text())
        assert data["/**/local_position"]["ros__parameters"]["frame_id"] == f"uav{i + 1}/map"
    manifest = json.loads((run / "manifest.json").read_text())
    assert manifest["config"]["world_name"] == "mpc_big1000"
    assert len(manifest["config"]["uavs"]) == 5
