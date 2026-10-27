"""Vectorized RGB-D projection utilities."""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int

    @classmethod
    def from_config(cls, config):
        return cls(float(config["fx"]), float(config["fy"]),
                   float(config["cx"]), float(config["cy"]),
                   int(config["width"]), int(config["height"]))

    def validate(self, image, name):
        if tuple(np.asarray(image).shape[:2]) != (self.height, self.width):
            raise ValueError("%s shape %r != (%d, %d)" % (
                name, np.asarray(image).shape, self.height, self.width))


def rotation_matrix_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.asarray([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float32)


def transform_from_pose(translation, rpy):
    result = np.eye(4, dtype=np.float32)
    result[:3, :3] = rotation_matrix_from_rpy(*map(float, rpy))
    result[:3, 3] = np.asarray(translation, dtype=np.float32)
    return result


def transform_points(points, transform):
    points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
    if not len(points):
        return points
    homogeneous = np.column_stack((points, np.ones(len(points), np.float32)))
    return (np.asarray(transform, np.float32) @ homogeneous.T).T[:, :3]


def valid_depth(values, minimum, maximum):
    values = np.asarray(values)
    return np.isfinite(values) & (values >= minimum) & (values <= maximum)


def pixels_to_world(pixels, depths, intrinsics, camera_to_world):
    pixels = np.asarray(pixels, np.float32).reshape(-1, 2)
    depths = np.asarray(depths, np.float32).reshape(-1)
    u, v = pixels[:, 0], pixels[:, 1]
    camera = np.column_stack(((u - intrinsics.cx) * depths / intrinsics.fx,
                              (v - intrinsics.cy) * depths / intrinsics.fy,
                              depths)).astype(np.float32)
    return transform_points(camera, camera_to_world)


def sample_depth(depth, stride, minimum, maximum):
    depth = np.asarray(depth, np.float32)
    rows, cols = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    values = depth[::stride, ::stride]
    mask = valid_depth(values, minimum, maximum)
    return np.column_stack((cols[mask], rows[mask])).astype(np.float32), values[mask]
