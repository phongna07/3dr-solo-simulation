"""Local, deterministic city meshes; independent of collision and scan geometry.

COLLADA carries the materials with each mesh, so both Gazebo clients and sensor
cameras render the same scene without Fuel downloads or texture search paths.
"""

import math
from pathlib import Path
import xml.etree.ElementTree as ET


# Gazebo's normal camera / GUI mask is GZ_VISIBILITY_ALL = 0x0fffffff.
# The extra bit is visible only to the explicitly configured GPU LiDAR. Keep
# proxy shadows off too: camera masks alone do not exclude shadow casters.
LIDAR_PROXY_FLAGS = 0x10000000
CITY_VISUAL_FLAGS = 0x10
CAMERA_VISIBILITY_MASK = 0x0FFFFFFF
LIDAR_VISIBILITY_MASK = (CAMERA_VISIBILITY_MASK | LIDAR_PROXY_FLAGS) & ~CITY_VISUAL_FLAGS


class Mesh:
    """Small flat-shaded mesh writer, batched by material rather than window."""

    def __init__(self, palette):
        self.palette = palette
        self.faces = {name: [] for name in palette}

    def face(self, material, points):
        a, b, c = points[:3]
        u = [b[i] - a[i] for i in range(3)]
        v = [c[i] - a[i] for i in range(3)]
        normal = (u[1] * v[2] - u[2] * v[1],
                  u[2] * v[0] - u[0] * v[2],
                  u[0] * v[1] - u[1] * v[0])
        length = math.sqrt(sum(n * n for n in normal))
        if length < 1e-12:
            return
        normal = tuple(n / length for n in normal)
        for i in range(1, len(points) - 1):
            self.faces[material].append(((points[0], points[i], points[i + 1]), normal))

    def box(self, material, center, size):
        x, y, z = center
        w, d, h = (s / 2 for s in size)
        for points in (
            [(x-w, y-d, z-h), (x-w, y-d, z+h), (x-w, y+d, z+h), (x-w, y+d, z-h)],
            [(x+w, y-d, z-h), (x+w, y+d, z-h), (x+w, y+d, z+h), (x+w, y-d, z+h)],
            [(x-w, y-d, z-h), (x+w, y-d, z-h), (x+w, y-d, z+h), (x-w, y-d, z+h)],
            [(x-w, y+d, z-h), (x-w, y+d, z+h), (x+w, y+d, z+h), (x+w, y+d, z-h)],
            [(x-w, y-d, z+h), (x+w, y-d, z+h), (x+w, y+d, z+h), (x-w, y+d, z+h)],
            [(x-w, y-d, z-h), (x-w, y+d, z-h), (x+w, y+d, z-h), (x+w, y-d, z-h)],
        ):
            self.face(material, points)

    def crown(self, center, radii, variant):
        # Staggered rings create a faceted broadleaf crown with no alpha planes.
        rings, segments = 7, 12
        def vertex(row, col):
            phi = math.pi * row / rings
            theta = 2 * math.pi * col / segments + (row % 2) * 0.16
            return (center[0] + radii[0] * math.sin(phi) * math.cos(theta),
                    center[1] + radii[1] * math.sin(phi) * math.sin(theta),
                    center[2] + radii[2] * math.cos(phi))
        for row in range(rings):
            for col in range(segments):
                material = f"leaf{(row + col * 3 + variant) % 4}"
                a, b = vertex(row, col), vertex(row, col + 1)
                c, d = vertex(row + 1, col + 1), vertex(row + 1, col)
                self.face(material, [a, d, c])
                self.face(material, [a, c, b])

    def trunk(self, radius, height):
        for i in range(12):
            a, b = (2 * math.pi * n / 12 for n in (i, i + 1))
            self.face("bark", [(radius * math.cos(a), radius * math.sin(a), 0),
                               (radius * math.cos(b), radius * math.sin(b), 0),
                               (0.65 * radius * math.cos(b), 0.65 * radius * math.sin(b), height),
                               (0.65 * radius * math.cos(a), 0.65 * radius * math.sin(a), height)])

    def write(self, path):
        root = ET.Element("COLLADA", xmlns="http://www.collada.org/2005/11/COLLADASchema",
                          version="1.4.1")
        asset = ET.SubElement(root, "asset")
        ET.SubElement(asset, "created").text = "2026-01-01T00:00:00Z"
        ET.SubElement(asset, "modified").text = "2026-01-01T00:00:00Z"
        ET.SubElement(asset, "unit", name="meter", meter="1")
        ET.SubElement(asset, "up_axis").text = "Z_UP"
        effects = ET.SubElement(root, "library_effects")
        materials = ET.SubElement(root, "library_materials")
        geometries = ET.SubElement(root, "library_geometries")
        scenes = ET.SubElement(root, "library_visual_scenes")
        scene = ET.SubElement(scenes, "visual_scene", id="city_scene")
        node = ET.SubElement(scene, "node", id="obstacle")
        for name, color in self.palette.items():
            faces = self.faces[name]
            if not faces:
                continue
            effect = ET.SubElement(effects, "effect", id=f"{name}_fx")
            profile = ET.SubElement(effect, "profile_COMMON")
            technique = ET.SubElement(profile, "technique", sid="common")
            phong = ET.SubElement(technique, "phong")
            for channel, value in (("ambient", (*color, 1)), ("diffuse", (*color, 1)),
                                   ("specular", (0.12, 0.12, 0.12, 1))):
                ET.SubElement(ET.SubElement(phong, channel), "color").text = " ".join(map(str, value))
            # Subtle self illumination keeps window grids legible in shade.
            if name in ("glass", "lit"):
                emission = ET.SubElement(phong, "emission")
                ET.SubElement(emission, "color").text = " ".join(map(str, (*[c * 0.3 for c in color], 1)))
            material = ET.SubElement(materials, "material", id=name)
            ET.SubElement(material, "instance_effect", url=f"#{name}_fx")
            geometry = ET.SubElement(geometries, "geometry", id=f"{name}_mesh")
            mesh = ET.SubElement(geometry, "mesh")
            count = len(faces) * 3
            for suffix, values in (
                ("positions", [v for points, _ in faces for p in points for v in p]),
                ("normals", [v for _, normal in faces for _ in range(3) for v in normal]),
            ):
                source = ET.SubElement(mesh, "source", id=f"{name}_{suffix}")
                array = ET.SubElement(source, "float_array", id=f"{name}_{suffix}_array", count=str(count * 3))
                array.text = " ".join(f"{v:.6f}" for v in values)
                accessor = ET.SubElement(ET.SubElement(source, "technique_common"), "accessor",
                                         source=f"#{name}_{suffix}_array", count=str(count), stride="3")
                for axis in "XYZ":
                    ET.SubElement(accessor, "param", name=axis, type="float")
            vertices = ET.SubElement(mesh, "vertices", id=f"{name}_vertices")
            ET.SubElement(vertices, "input", semantic="POSITION", source=f"#{name}_positions")
            triangles = ET.SubElement(mesh, "triangles", material=name, count=str(len(faces)))
            ET.SubElement(triangles, "input", semantic="VERTEX", source=f"#{name}_vertices", offset="0")
            ET.SubElement(triangles, "input", semantic="NORMAL", source=f"#{name}_normals", offset="1")
            ET.SubElement(triangles, "p").text = " ".join(f"{i} {i}" for i in range(count))
            instance = ET.SubElement(node, "instance_geometry", url=f"#{name}_mesh")
            binding = ET.SubElement(ET.SubElement(instance, "bind_material"), "technique_common")
            ET.SubElement(binding, "instance_material", symbol=name, target=f"#{name}")
        ET.SubElement(ET.SubElement(root, "scene"), "instance_visual_scene", url="#city_scene")
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
        return path


def building_mesh(path, width, depth, obstacle_height, index):
    height = obstacle_height * (1.8 + 0.18 * (index % 5))
    facades = [(0.48, 0.52, 0.56), (0.61, 0.56, 0.47), (0.36, 0.43, 0.50)]
    mesh = Mesh(dict(wall=facades[index % 3], glass=(0.08, 0.22, 0.32),
                     lit=(0.86, 0.72, 0.40), trim=(0.25, 0.30, 0.34), roof=(0.30, 0.33, 0.35)))
    mesh.box("wall", (0, 0, height / 2), (width, depth, height))
    # Windows are flat quads with a small offset, batched into two submeshes.
    # No thousands of SDF entities, textures, or overlapping coplanar surfaces.
    floors = max(4, round(height / 0.85))
    pitch = height / floors
    for face in range(4):
        span = width if face < 2 else depth
        columns = max(2, round(span / 0.55))
        spacing = span / columns
        for floor in range(floors):
            z = (floor + 0.5) * pitch
            for col in range(columns):
                u = -span / 2 + (col + 0.5) * spacing
                du, dz = spacing * 0.32, pitch * 0.31
                points = [(u-du, z-dz), (u+du, z-dz), (u+du, z+dz), (u-du, z+dz)]
                if face < 2:
                    sign = -1 if face == 0 else 1
                    vertices = [(a, sign * (depth / 2 + 0.003), b) for a, b in points]
                else:
                    sign = 1 if face == 2 else -1
                    vertices = [(sign * (width / 2 + 0.003), a, b) for a, b in points]
                if face in (1, 3):
                    vertices.reverse()
                material = "lit" if (floor * 7 + col * 11 + face * 3 + index) % 9 == 0 else "glass"
                mesh.face(material, vertices)
        for floor in range(1, floors):
            z = floor * pitch
            if face < 2:
                mesh.box("trim", (0, (-1 if face == 0 else 1) * depth / 2, z),
                         (width, 0.012, 0.025))
            else:
                mesh.box("trim", ((1 if face == 2 else -1) * width / 2, 0, z),
                         (0.012, depth, 0.025))
    mesh.box("roof", (0, 0, height + 0.04), (width, depth, 0.08))
    for x, y, w, d in ((0, -depth / 2 + 0.04, width, 0.08),
                       (0, depth / 2 - 0.04, width, 0.08),
                       (-width / 2 + 0.04, 0, 0.08, depth),
                       (width / 2 - 0.04, 0, 0.08, depth)):
        mesh.box("wall", (x, y, height + 0.15), (w, d, 0.3))
    mesh.box("trim", (0.12 * width, 0.1 * depth, height + 0.24),
             (0.3 * width, 0.3 * depth, 0.4))
    return mesh.write(path)


def tree_mesh(path, radius, obstacle_height, index):
    height = obstacle_height * (1.25 + 0.08 * (index % 4))
    mesh = Mesh(dict(bark=(0.28, 0.16, 0.08), leaf0=(0.12, 0.30, 0.08),
                     leaf1=(0.20, 0.40, 0.10), leaf2=(0.28, 0.46, 0.13),
                     leaf3=(0.16, 0.35, 0.07)))
    mesh.trunk(radius * 0.18, height * 0.73)
    mesh.crown((0, 0, height * 0.70), (radius * 0.78, radius * 0.78, height * 0.30), index)
    for branch in range(3):
        angle = 2 * math.pi * branch / 3 + index * 0.7
        mesh.crown((radius * 0.38 * math.cos(angle), radius * 0.38 * math.sin(angle), height * 0.58),
                   (radius * 0.60, radius * 0.60, height * 0.21), index + branch)
    return mesh.write(path)
