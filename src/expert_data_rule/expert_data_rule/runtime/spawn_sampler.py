"""Random collision-free spawn sampling from a Stonefish .scn file."""

from dataclasses import dataclass
import math
from pathlib import Path
import random
import re
import xml.etree.ElementTree as ET

import numpy as np

from expert_data_rule.perception.geometry import transform_from_pose, transform_points


def _numbers(text, count):
    values = tuple(float(item) for item in str(text).split())
    if len(values) == 1 and count == 3:
        values *= 3
    if len(values) != count:
        raise ValueError("expected %d values, got %r" % (count, text))
    return values


def _transform(element, tag):
    child = element.find(tag)
    if child is None:
        return np.eye(4, dtype=np.float32)
    return transform_from_pose(_numbers(child.get("xyz", "0 0 0"), 3),
                               _numbers(child.get("rpy", "0 0 0"), 3))


@dataclass(frozen=True)
class Bounds2D:
    min_x: float
    max_x: float
    min_y: float
    max_y: float

    def inflated(self, margin):
        return Bounds2D(self.min_x-margin, self.max_x+margin,
                        self.min_y-margin, self.max_y+margin)

    def contains(self, x, y):
        return self.min_x <= x <= self.max_x and self.min_y <= y <= self.max_y


@dataclass(frozen=True)
class SpawnPose:
    x: float
    y: float
    z: float
    yaw: float


class SceneSpawnSampler:
    def __init__(self, scenario_file, config, seed=42, package_resolver=None):
        self.path = Path(scenario_file).expanduser().resolve()
        if not self.path.is_file():
            raise FileNotFoundError("scenario file not found: %s" % self.path)
        self.config = config
        self.rng = random.Random(int(seed))
        self.package_resolver = package_resolver
        root = ET.parse(str(self.path)).getroot()
        self.pool = self._pool_bounds(root)
        pattern = re.compile(str(config["rock_name_regex"]))
        self.rocks = [self._static_bounds(item).inflated(
            float(config["rock_clearance_m"]))
            for item in root.findall("static") if pattern.search(item.get("name", ""))]
        if not self.rocks:
            raise ValueError("no active rock statics matched %r" % pattern.pattern)

    def _pool_bounds(self, root):
        name = str(self.config["pool_static_name"])
        pool = next((item for item in root.findall("static")
                     if item.get("name") == name), None)
        if pool is None or pool.get("type") != "box":
            raise ValueError("pool static %r must be an active box" % name)
        dimensions = _numbers(pool.find("dimensions").get("xyz"), 3)
        vertices = np.asarray([[x, y, 0] for x in (-dimensions[0]/2, dimensions[0]/2)
                               for y in (-dimensions[1]/2, dimensions[1]/2)], np.float32)
        vertices = transform_points(vertices, _transform(pool, "world_transform"))
        margin = float(self.config["boundary_margin_m"])
        return Bounds2D(float(vertices[:, 0].min()+margin),
                        float(vertices[:, 0].max()-margin),
                        float(vertices[:, 1].min()+margin),
                        float(vertices[:, 1].max()-margin))

    def _resolve_mesh(self, filename):
        match = re.match(r"\$\(find\s+([^\)]+)\)(.*)", filename)
        if match:
            if self.package_resolver is None:
                raise ValueError("cannot resolve ROS package path %r" % filename)
            return Path(self.package_resolver(match.group(1))) / match.group(2).lstrip("/\\")
        path = Path(filename)
        return path if path.is_absolute() else self.path.parent/path

    @staticmethod
    def _obj_vertices(path):
        vertices = []
        with Path(path).open(encoding="utf-8", errors="ignore") as stream:
            for line in stream:
                if line.startswith("v "):
                    values = line.split()
                    if len(values) >= 4:
                        vertices.append(tuple(map(float, values[1:4])))
        if not vertices:
            raise ValueError("mesh has no OBJ vertices: %s" % path)
        return np.asarray(vertices, np.float32)

    def _static_bounds(self, item):
        if item.get("type") == "box":
            size = _numbers(item.find("dimensions").get("xyz"), 3)
            vertices = np.asarray([[x, y, z]
                for x in (-size[0]/2, size[0]/2)
                for y in (-size[1]/2, size[1]/2)
                for z in (-size[2]/2, size[2]/2)], np.float32)
            transform = _transform(item, "world_transform")
        elif item.get("type") == "model":
            physical = item.find("physical")
            mesh = physical.find("mesh") if physical is not None else None
            if mesh is None:
                raise ValueError("rock model %s has no physical mesh" % item.get("name"))
            vertices = self._obj_vertices(self._resolve_mesh(mesh.get("filename")))
            scale = _numbers(mesh.get("scale", "1"), 3)
            vertices *= np.asarray(scale, np.float32)
            transform = _transform(item, "world_transform") @ _transform(physical, "origin")
        else:
            raise ValueError("unsupported rock type %r" % item.get("type"))
        points = transform_points(vertices, transform)
        return Bounds2D(float(points[:, 0].min()), float(points[:, 0].max()),
                        float(points[:, 1].min()), float(points[:, 1].max()))

    def sample(self):
        for _ in range(int(self.config["max_sampling_attempts"])):
            x = self.rng.uniform(self.pool.min_x, self.pool.max_x)
            y = self.rng.uniform(self.pool.min_y, self.pool.max_y)
            if any(rock.contains(x, y) for rock in self.rocks):
                continue
            return SpawnPose(x, y, float(self.config["fixed_z_m"]),
                             self.rng.uniform(-math.pi, math.pi))
        raise RuntimeError("could not sample a collision-free spawn pose")
