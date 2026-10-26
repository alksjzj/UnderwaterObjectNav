"""Extract collision-height geometry and explored endpoints from depth."""

import numpy as np

from .geometry import pixels_to_world, sample_depth


def extract_depth_geometry(depth, intrinsics, camera_to_world, config):
    pixels, depths = sample_depth(
        depth, int(config["obstacle_depth_stride"]),
        float(config["min_depth_m"]), float(config["max_depth_m"]))
    points = pixels_to_world(pixels, depths, intrinsics, camera_to_world)
    sensor = np.asarray(camera_to_world, np.float32)[:3, 3]
    minimum = sensor[2] + float(config["obstacle_z_min_offset_m"])
    maximum = sensor[2] + float(config["obstacle_z_max_offset_m"])
    collision = (points[:, 2] >= minimum) & (points[:, 2] <= maximum)
    return points[collision], points, pixels, sensor
