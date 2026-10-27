"""Persistent evidence rendered into a robot-centred local map."""

from dataclasses import dataclass
import math

import numpy as np

from .global_map import GridMap, WorldMapSpec


LOCAL_CHANNELS = ("obstacle", "rock", "echinus", "visited")


@dataclass(frozen=True)
class LocalMapSpec:
    width_m: float
    height_m: float
    resolution_m: float

    @classmethod
    def from_config(cls, config):
        return cls(float(config["width_m"]), float(config["height_m"]),
                   float(config["resolution_m"]))

    @property
    def width_cells(self):
        return int(round(self.width_m/self.resolution_m))

    @property
    def height_cells(self):
        return int(round(self.height_m/self.resolution_m))


class LocalEvidenceMap(GridMap):
    def __init__(self, world_spec, resolution):
        super().__init__(LOCAL_CHANNELS, WorldMapSpec(
            world_spec.origin_x, world_spec.origin_y, world_spec.width_m,
            world_spec.height_m, float(resolution)))

    def update(self, obstacle, robot_xy, echinus, echinus_scores,
               rocks, rock_scores, map_config):
        self.mark_points("obstacle", np.asarray(obstacle).reshape(-1, 3)[:, :2])
        self.mark_points("rock", np.asarray(rocks).reshape(-1, 3)[:, :2], rock_scores)
        self.mark_disks("echinus", np.asarray(echinus).reshape(-1, 3)[:, :2],
                        float(map_config["echinus_radius_m"]), echinus_scores)
        self.mark_disks("visited", [robot_xy], float(map_config["visited_radius_m"]))

    def render(self, robot_xy, yaw, spec):
        rows, cols = np.mgrid[0:spec.height_cells, 0:spec.width_cells]
        forward = (spec.height_cells/2-(rows+0.5))*spec.resolution_m
        right = ((cols+0.5)-spec.width_cells/2)*spec.resolution_m
        cosine, sine = math.cos(yaw), math.sin(yaw)
        world_x = robot_xy[0]+cosine*forward-sine*right
        world_y = robot_xy[1]+sine*forward+cosine*right
        x, y, valid = self.indices(np.column_stack((world_x.ravel(), world_y.ravel())))
        result = np.zeros((len(LOCAL_CHANNELS), spec.height_cells,
                           spec.width_cells), np.float32)
        result.reshape(len(LOCAL_CHANNELS), -1)[:, valid] = self.map[:, y[valid], x[valid]]
        return result
