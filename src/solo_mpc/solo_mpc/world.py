"""Generate original scan/collision primitives with separate city appearances."""

import copy
import math
import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from .core.perception.sensors import CAMERA_WORLD_OFFSET
from .obstacle_visuals import (
    CAMERA_VISIBILITY_MASK, CITY_VISUAL_FLAGS, LIDAR_PROXY_FLAGS,
    LIDAR_VISIBILITY_MASK, building_mesh, srgb_material, street_mesh, tree_mesh,
)
from .core.video.style import (
    OVERHEAD_FPS, OVERHEAD_HEIGHT, OVERHEAD_TOPIC, OVERHEAD_WIDTH, follow_offset, vehicle_color,
)


# Keep in sync with CameraCoverage::visibility in gazebo/CameraCoverage.hh.
COVERAGE_VISUAL_FLAGS = 0x20


def fleet_world(config, source):
    source = Path(source)
    tree = ET.parse(source / 'scenery.sdf')
    world = tree.getroot().find('world')
    world.set('name', config['world_name'])
    template = ET.parse(source / 'iris_with_standoffs/model.sdf').getroot().find('model')
    for uav in config['uavs']:
        model = copy.deepcopy(template)
        model.set('name', uav['name'])
        pose = model.find('pose')
        if pose is None:
            pose = ET.SubElement(model, 'pose')
        pose.set('degrees', 'true')
        pose.text = ' '.join(map(str, uav['spawn']))
        native_multicopter(model, uav['name'])
        world.append(model)
    return tree


def element(parent, tag, text=None, **attrs):
    node = ET.SubElement(parent, tag, attrs)
    if text is not None:
        node.text = str(text)
    return node


def native_multicopter(model, name):
    safety = element(model, 'plugin', filename='SoloFlightSafety', name='solo::FlightSafety')
    element(safety, 'namespace', name)
    control = element(model, 'plugin', filename='gz-sim-multicopter-control-system',
                      name='gz::sim::systems::MulticopterVelocityControl')
    for tag, value in [('robotNamespace', name), ('commandSubTopic', 'control/twist'),
                       ('enableSubTopic', 'control/enable'), ('comLinkName', 'base_link'),
                       ('velocityGain', '8 8 10'), ('attitudeGain', '0.8 0.8 0.4'),
                       ('angularRateGain', '0.15 0.15 0.1'),
                       ('maximumLinearAcceleration', '4 4 3'),
                       ('maximumLinearVelocity', '12 12 5'),
                       ('maximumAngularVelocity', '2 2 1')]:
        element(control, tag, value)
    rotors = element(control, 'rotorConfiguration')
    for index, direction in enumerate([-1, -1, 1, 1]):
        rotor = element(rotors, 'rotor')
        for tag, value in [('jointName', f'rotor_{index}_joint'),
                           ('forceConstant', 8.54858e-6), ('momentConstant', 0.016),
                           ('direction', direction)]:
            element(rotor, tag, value)
        motor = element(model, 'plugin', filename='gz-sim-multicopter-motor-model-system',
                        name='gz::sim::systems::MulticopterMotorModel')
        for tag, value in [('robotNamespace', name), ('jointName', f'rotor_{index}_joint'),
                           ('linkName', f'rotor_{index}'),
                           ('turningDirection', 'ccw' if direction == 1 else 'cw'),
                           ('timeConstantUp', 0.0125), ('timeConstantDown', 0.025),
                           ('maxRotVelocity', 1100), ('motorConstant', 8.54858e-6),
                           ('momentConstant', 0.016), ('commandSubTopic', 'motor_command'),
                           ('actuator_number', index), ('rotorDragCoefficient', 8.06428e-5),
                           ('rollingMomentCoefficient', 1e-6), ('rotorVelocitySlowdownSim', 10),
                           ('motorType', 'velocity')]:
            element(motor, tag, value)
    odometry = element(model, 'plugin', filename='gz-sim-odometry-publisher-system',
                       name='gz::sim::systems::OdometryPublisher')
    for tag, value in [('dimensions', 3), ('odom_frame', 'world'),
                       ('robot_base_frame', f'{name}/base_link'),
                       ('odom_publish_frequency', 50),
                       ('odom_topic', f'/{name}/ground_truth/odometry')]:
        element(odometry, tag, value)


def solid(world, name, position, shape, size, color):
    model = element(world, "model", name=name)
    element(model, "static", "true")
    element(model, "pose", " ".join(map(str, [*position, 0, 0, 0])))
    link = element(model, "link", name="body")
    for kind in ("visual", "collision"):
        part = element(link, kind, name=kind)
        geometry = element(part, "geometry")
        geom = element(geometry, shape)
        if shape == "box":
            element(geom, "size", " ".join(map(str, size)))
        elif shape == "plane":
            element(geom, "normal", "0 0 1")
            element(geom, "size", " ".join(map(str, size)))
        else:
            element(geom, "radius", size[0])
            element(geom, "length", size[1])
        if kind == "visual":
            material = element(part, "material")
            element(material, "ambient", color)
            element(material, "diffuse", color)
            if name == "tracking_target":
                element(material, "emissive", color)
    return model


def city_visual(model, mesh_path, obstacle_height):
    """Retain the original primitive for physics and GPU LiDAR only."""
    link = model.find("link[@name='body']")
    proxy = link.find("visual")
    element(proxy, "visibility_flags", LIDAR_PROXY_FLAGS)
    element(proxy, "cast_shadows", "false")
    visual = element(link, "visual", name="city_appearance")
    # The model origin stays at the ORIGINAL collision primitive's midpoint.
    element(visual, "pose", f"0 0 {-obstacle_height / 2} 0 0 0")
    element(visual, "visibility_flags", CITY_VISUAL_FLAGS)
    mesh = element(element(visual, "geometry"), "mesh")
    element(mesh, "uri", Path(mesh_path).resolve().as_uri())


def sensors(model, name, settings, cfg, camera_enabled=True, color_index=0):
    link = element(model, "link", name="perception_link")
    element(link, "pose", "0 0 0 0 0 0", relative_to="base_link")
    inertial = element(link, "inertial")
    element(inertial, "mass", 0.01)
    inertia = element(inertial, "inertia")
    for axis in ("ixx", "iyy", "izz"):
        element(inertia, axis, 0.00001)
    joint = element(model, "joint", name="perception_joint", type="fixed")
    element(joint, "parent", "base_link")
    element(joint, "child", "perception_link")
    laser = element(link, "sensor", name="laser", type="gpu_lidar")
    element(laser, "pose", "0 0 0.08 0 0 0")
    element(laser, "gz_frame_id", f"{name}/laser")
    element(laser, "topic", f"/{name}/scan")
    element(laser, "update_rate", settings["lidar_rate"])
    element(laser, "always_on", "true")
    element(laser, "visualize", "false")
    ray = element(laser, "ray")
    element(ray, "visibility_mask", LIDAR_VISIBILITY_MASK & ~COVERAGE_VISUAL_FLAGS)
    scan = element(ray, "scan")
    horizontal = element(scan, "horizontal")
    count = settings["lidar_samples"]
    element(horizontal, "samples", count)
    element(horizontal, "resolution", 1)
    element(horizontal, "min_angle", -3.141592653589793)
    element(horizontal, "max_angle", 3.141592653589793 - 2 * 3.141592653589793 / count)
    vertical = element(scan, "vertical")
    element(vertical, "samples", 1)
    element(vertical, "resolution", 1)
    element(vertical, "min_angle", 0)
    element(vertical, "max_angle", 0)
    ran = element(ray, "range")
    element(ran, "min", cfg.get("LIDAR_RANGE_MIN", 0.15))
    element(ran, "max", cfg["SENSING_RADIUS"])
    element(ran, "resolution", 0.01)
    noise = element(ray, "noise", type="gaussian")
    element(noise, "mean", 0)
    element(noise, "stddev", settings.get("lidar_noise_stddev", 0.01))
    # UAV truth feeds frame alignment and evaluation independently of FPV.
    publisher = element(
        model,
        "plugin",
        filename="gz-sim-pose-publisher-system",
        name="gz::sim::systems::PosePublisher",
    )
    for tag, value in [
        ("publish_model_pose", "true"),
        ("publish_link_pose", "false"),
        ("publish_nested_model_pose", "false"),
        ("use_pose_vector_msg", "false"),
        ("update_frequency", "20"),
        ("topic", f"/{name}/ground_truth/pose"),
    ]:
        element(publisher, tag, value)
    if not camera_enabled:
        return
    gimbal = element(model, "plugin", filename="SoloWorldStabilizedCamera",
                     name="solo::WorldStabilizedCamera")
    element(gimbal, "camera_name", f"{name}::perception_link::camera")
    element(gimbal, "world_offset", " ".join(map(str, CAMERA_WORLD_OFFSET)))
    element(gimbal, "coverage_parent", f"{name}::perception_link")
    element(gimbal, "coverage_hfov", settings["camera_hfov"])
    element(gimbal, "coverage_aspect", settings["camera_width"] / settings["camera_height"])
    element(gimbal, "coverage_color", coverage_color(color_index))
    element(gimbal, "coverage_ray_diameter", settings['coverage_ray_diameter'])
    element(gimbal, "coverage_outline_diameter", settings['coverage_outline_diameter'])
    camera = element(link, "sensor", name="camera", type="camera")
    element(camera, "pose", "0 0 -0.04 0 1.5707963267948966 0")
    element(camera, "gz_frame_id", f"{name}/camera_optical_frame")
    element(camera, "topic", f"/{name}/camera/image_raw")
    element(camera, "update_rate", settings["camera_rate"])
    element(camera, "always_on", "true")
    optics = element(camera, "camera")
    element(optics, "visibility_mask", CAMERA_VISIBILITY_MASK & ~COVERAGE_VISUAL_FLAGS)
    element(optics, "horizontal_fov", settings["camera_hfov"])
    element(optics, "camera_info_topic", f"/{name}/camera/camera_info")
    image = element(optics, "image")
    element(image, "width", settings["camera_width"])
    element(image, "height", settings["camera_height"])
    element(image, "format", "R8G8B8")
    clip = element(optics, "clip")
    element(clip, "near", 0.05)
    element(clip, "far", 100)


def coverage_color(index):
    return " ".join(str(c / 255) for c in reversed(vehicle_color(index))) + " 1"


def coverage_gui(world, names, settings):
    """Load the coverage helper in GUI clients, including viewers attached later."""
    gui = world.find("gui")
    if gui is None:
        gui = element(world, "gui")
        # A custom SDF GUI replaces Gazebo's fallback plugin list. Retain the
        # installed Harmonic defaults (scene, navigation, tools, world controls).
        defaults = Path("/usr/share/gz/gz-sim8/gui/gui.config").read_text()
        defaults = ET.fromstring("<gui>" + defaults.split("?>", 1)[-1] + "</gui>")
        gui.extend(defaults.findall("plugin"))
    plugin = element(gui, "plugin", filename="SoloCameraCoverageGui",
                     name="Camera coverage")
    properties = element(plugin, "gz-gui")
    for key, kind, value in (("state", "string", "floating"),
                             ("width", "double", "0"), ("height", "double", "0"),
                             ("showTitleBar", "bool", "false"),
                             ("visible", "bool", "false"),
                             ("resizable", "bool", "false")):
        element(properties, "property", value, key=key, type=kind)
    for index, name in enumerate(names):
        element(plugin, "uav", link=f"{name}::perception_link",
                hfov=str(settings["camera_hfov"]),
                aspect=str(settings["camera_width"] / settings["camera_height"]),
                offset=" ".join(map(str, CAMERA_WORLD_OFFSET)),
                color=coverage_color(index),
                ray_diameter=str(settings['coverage_ray_diameter']),
                outline_diameter=str(settings['coverage_outline_diameter']))


def overhead_camera(target, settings):
    # Attach to the target's existing link: set_pose then moves both together,
    # without a second tracking process, collision geometry or physical mass.
    camera = element(target.find("link[@name='body']"), "sensor",
                     name="overhead_camera", type="camera")
    x, y, z = follow_offset(settings)
    pitch = math.atan2(z, math.hypot(x, y))
    yaw = math.atan2(-y, -x)
    element(camera, "pose", f"{x} {y} {z} 0 {pitch} {yaw}")
    element(camera, "topic", OVERHEAD_TOPIC)
    element(camera, "update_rate", OVERHEAD_FPS)
    element(camera, "always_on", "true")
    element(camera, "visualize", "false")
    optics = element(camera, "camera")
    element(optics, "visibility_mask", CAMERA_VISIBILITY_MASK)
    element(optics, "horizontal_fov", math.pi / 3)
    image = element(optics, "image")
    element(image, "width", OVERHEAD_WIDTH)
    element(image, "height", OVERHEAD_HEIGHT)
    element(image, "format", "R8G8B8")
    clip = element(optics, "clip")
    element(clip, "near", 0.1)
    element(clip, "far", max(200, 2 * math.sqrt(x * x + y * y + z * z)))


def generate_mpc_world(fleet, source, destination, scenario, settings, cfg,
                       record_video=False):
    # UAV cameras feed only the recorded FOV panels and coverage overlays.
    camera_enabled = record_video
    if camera_enabled and settings["camera_width"] != settings["camera_height"]:
        raise ValueError("World-stabilized visibility model requires a square camera image")
    output = Path(destination)
    output.parent.mkdir(parents=True, exist_ok=True)
    tree = fleet_world(fleet, source)
    world = tree.getroot().find("world")
    # Throttle wall pacing, retaining the original integration step and all
    # simulation-time rates. This gives MPC more CPU time per physical second.
    physics = world.find('physics')
    if physics is None:
        physics = element(world, 'physics', name='1ms', type='ignore')
        element(physics, 'max_step_size', 0.001)
    factor = physics.find('real_time_factor')
    if factor is None:
        factor = element(physics, 'real_time_factor')
    factor.text = str(settings['simulation_real_time_factor'])
    for include in list(world.findall("include")):
        world.remove(include)
    render = element(
        world,
        "plugin",
        filename="gz-sim-sensors-system",
        name="gz::sim::systems::Sensors",
    )
    element(render, "render_engine", "ogre2")
    xmin, ymin, xmax, ymax = cfg["WORLD_BOUNDS"]
    padding = 2 * cfg["SENSING_RADIUS"]
    ground_width = max(150, xmax - xmin + padding)
    ground_depth = max(150, ymax - ymin + padding)
    cx, cy = (xmin + xmax) / 2, (ymin + ymax) / 2
    ground = solid(world, "field_ground", [cx, cy, 0],
                   "plane", [ground_width, ground_depth],
                   ' '.join(map(str, (*srgb_material(64, 67, 65), 1))))
    height = settings["obstacle_height"]
    assets = Path(output).parent / "city_assets"
    paint = element(ground.find('link'), 'visual', name='street_markings')
    element(paint, 'pose', f'{-cx} {-cy} 0 0 0 0')
    element(paint, 'visibility_flags', CITY_VISUAL_FLAGS)
    element(paint, 'cast_shadows', 'false')
    mesh = element(element(paint, 'geometry'), 'mesh')
    element(mesh, 'uri', street_mesh(assets / 'street_markings.dae',
            (cx - ground_width / 2, cy - ground_depth / 2,
             cx + ground_width / 2, cy + ground_depth / 2)).resolve().as_uri())
    for i, (x, y, w, h) in enumerate(scenario["obstacles"]["rects"]):
        model = solid(
            world,
            f"obstacle_rect_{i}",
            [x + w / 2, y + h / 2, height / 2],
            "box",
            [w, h, height],
            "0.1 0.45 0.15 1",
        )
        city_visual(model, building_mesh(assets / f"building_{i}.dae", w, h, height, i), height)
    for i, (x, y, r) in enumerate(scenario["obstacles"]["circles"]):
        model = solid(
            world,
            f"obstacle_circle_{i}",
            [x, y, height / 2],
            "cylinder",
            [r, height],
            "0.15 0.35 0.65 1",
        )
        city_visual(model, tree_mesh(assets / f"tree_{i}.dae", r, height, i), height)
    first = scenario["target"]["waypoints"][0]
    target = solid(
        world,
        "tracking_target",
        [first[0], first[1], settings["target_height"]],
        "box",
        [2 * cfg.get("TAR_RADIUS", 0.15), 2 * cfg.get("TAR_RADIUS", 0.15), 0.04],
        "1 0 1 1",
    )
    if record_video:
        overhead_camera(target, settings)
    for index, uav in enumerate(fleet["uavs"]):
        model = world.find(f"model[@name='{uav['name']}']")
        sensors(model, uav["name"], settings, cfg, camera_enabled=camera_enabled,
                color_index=index)
    if camera_enabled:
        coverage_gui(world, [uav["name"] for uav in fleet["uavs"]], settings)
    ET.indent(tree, space="  ")
    tree.write(output, encoding="utf-8", xml_declaration=True)
    return output


def bridge_config(names, world, destination, record_overhead=False, *, camera_enabled=True):
    rows = [
        dict(
            ros_topic_name="/clock",
            gz_topic_name=f"/world/{world}/clock",
            ros_type_name="rosgraph_msgs/msg/Clock",
            gz_type_name="gz.msgs.Clock",
            direction="GZ_TO_ROS",
        )
    ]
    for n in names:
        for suffix, ros, gz in [
            ("scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan"),
            ("camera/image_raw", "sensor_msgs/msg/Image", "gz.msgs.Image"),
            ("camera/camera_info", "sensor_msgs/msg/CameraInfo", "gz.msgs.CameraInfo"),
            ("ground_truth/pose", "geometry_msgs/msg/PoseStamped", "gz.msgs.Pose"),
            ("ground_truth/odometry", "nav_msgs/msg/Odometry", "gz.msgs.Odometry"),
            ("motor_speed", "actuator_msgs/msg/Actuators", "gz.msgs.Actuators"),
            ("safety_active", "std_msgs/msg/Bool", "gz.msgs.Boolean"),
        ]:
            if suffix.startswith("camera/") and not camera_enabled:
                continue
            rows.append(
                dict(
                    ros_topic_name=f"/{n}/{suffix}",
                    gz_topic_name=f"/{n}/{suffix}",
                    ros_type_name=ros,
                    gz_type_name=gz,
                    direction="GZ_TO_ROS",
                )
            )
        for suffix, ros, gz in [('cmd_vel', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist'),
                                ('enable', 'std_msgs/msg/Bool', 'gz.msgs.Boolean')]:
            rows.append(dict(ros_topic_name=f'/{n}/{suffix}', gz_topic_name=f'/{n}/{suffix}',
                             ros_type_name=ros, gz_type_name=gz, direction='ROS_TO_GZ'))
    if record_overhead:
        rows.append(dict(
            ros_topic_name=OVERHEAD_TOPIC,
            gz_topic_name=OVERHEAD_TOPIC,
            ros_type_name="sensor_msgs/msg/Image",
            gz_type_name="gz.msgs.Image",
            direction="GZ_TO_ROS",
            # Full HD images span thousands of DDS fragments. Best-effort
            # delivery loses whole images on a fragment drop, producing holds
            # in the constant-rate recording. Omit the sensor-data profile to
            # use the bridge's default reliable QoS, matching the recorder.
            publisher_queue=20,
            # Gazebo skips rendering without transport subscribers. The
            # recorder subscribes on arming, before its takeoff recording gate.
            lazy=True,
        ))
    Path(destination).write_text(yaml.safe_dump(rows, sort_keys=False))
    return str(destination)
