"""Persistent world-coordinate semantic map."""

from dataclasses import dataclass

import numpy as np


GLOBAL_CHANNELS = ("obstacle", "explored", "visited", "robot", "echinus", "rock")
GLOBAL_INDEX = {name: index for index, name in enumerate(GLOBAL_CHANNELS)}


@dataclass(frozen=True)
class WorldMapSpec:
    origin_x: float
    origin_y: float
    width_m: float
    height_m: float
    resolution_m: float

    @classmethod
    def from_config(cls, config):
        return cls(*(float(config[name]) for name in
                     ("origin_x", "origin_y", "width_m", "height_m",
                      "resolution_m")))

    @property
    def width_cells(self):
        return int(round(self.width_m / self.resolution_m))

    @property
    def height_cells(self):
        return int(round(self.height_m / self.resolution_m))


class GridMap:
    def __init__(self, channels, spec):
        self.channels = tuple(channels)
        self.index = {name: i for i, name in enumerate(self.channels)}
        self.spec = spec
        self.reset()

    def reset(self):
        self.map = np.zeros((len(self.channels), self.spec.height_cells,
                             self.spec.width_cells), np.float32)

    def indices(self, points):
        points = np.asarray(points, np.float32).reshape(-1, 2)
        x = np.floor((points[:, 0]-self.spec.origin_x) /
                     self.spec.resolution_m).astype(np.int32)
        y = np.floor((points[:, 1]-self.spec.origin_y) /
                     self.spec.resolution_m).astype(np.int32)
        valid = ((x >= 0) & (x < self.spec.width_cells) &
                 (y >= 0) & (y < self.spec.height_cells))
        return x, y, valid

    def mark_points(self, channel, points, values=1.0):
        points = np.asarray(points, np.float32).reshape(-1, 2)
        if not len(points):
            return
        x, y, valid = self.indices(points)
        if np.isscalar(values):
            selected = np.full(valid.sum(), float(values), np.float32)
        else:
            selected = np.asarray(values, np.float32).reshape(-1)[valid]
        np.maximum.at(self.map[self.index[channel]], (y[valid], x[valid]), selected)

    def mark_disks(self, channel, centers, radius, values=1.0):
        centers = np.asarray(centers, np.float32).reshape(-1, 2)
        if not len(centers):
            return
        values = (np.full(len(centers), float(values), np.float32)
                  if np.isscalar(values) else np.asarray(values, np.float32))
        x, y, center_valid = self.indices(centers)
        cells = int(np.ceil(radius/self.spec.resolution_m))
        layer = self.map[self.index[channel]]
        for dy in range(-cells, cells+1):
            for dx in range(-cells, cells+1):
                if np.hypot(dx, dy)*self.spec.resolution_m > radius+1e-6:
                    continue
                xx, yy = x+dx, y+dy
                valid = (center_valid & (xx >= 0) & (xx < self.spec.width_cells) &
                         (yy >= 0) & (yy < self.spec.height_cells))
                np.maximum.at(layer, (yy[valid], xx[valid]), values[valid])


def explored_rays(sensor_xy, endpoints, pixels, resolution):
    sensor_xy = np.asarray(sensor_xy, np.float32).reshape(2)
    endpoints = np.asarray(endpoints, np.float32).reshape(-1, 3)
    pixels = np.asarray(pixels, np.float32).reshape(-1, 2)
    if not len(endpoints):
        return np.empty((0, 2), np.float32)
    columns = pixels[:, 0].astype(np.int32)
    distances = np.linalg.norm(endpoints[:, :2]-sensor_xy, axis=1)
    selected = [indices[np.argmax(distances[indices])]
                for column in np.unique(columns)
                for indices in [np.flatnonzero(columns == column)]]
    rays = endpoints[np.asarray(selected), :2]-sensor_xy
    counts = np.maximum(1, np.ceil(np.linalg.norm(rays, axis=1)/resolution)
                        .astype(np.int32))
    sample = np.arange(int(counts.max())+1, dtype=np.float32)
    mask = sample[None, :] <= counts[:, None]
    fraction = sample[None, :]/counts[:, None]
    return (sensor_xy[None, None, :] + fraction[:, :, None]*rays[:, None, :])[mask]


class GlobalSemanticMap(GridMap):
    def __init__(self, spec):
        super().__init__(GLOBAL_CHANNELS, spec)

    def update(self, obstacle, endpoints, pixels, sensor_xy, robot_xy,
               echinus, echinus_scores, rocks, rock_scores, config):
        self.map[GLOBAL_INDEX["robot"]].fill(0)
        self.mark_points("explored", explored_rays(
            sensor_xy, endpoints, pixels, self.spec.resolution_m))
        self.mark_points("obstacle", np.asarray(obstacle).reshape(-1, 3)[:, :2])
        self.mark_disks("visited", [robot_xy], float(config["visited_radius_m"]))
        self.mark_disks("robot", [robot_xy], float(config["robot_radius_m"]))
        self.mark_disks("echinus", np.asarray(echinus).reshape(-1, 3)[:, :2],
                        float(config["echinus_radius_m"]), echinus_scores)
        self.mark_points("rock", np.asarray(rocks).reshape(-1, 3)[:, :2], rock_scores)
        return self.map.copy()
