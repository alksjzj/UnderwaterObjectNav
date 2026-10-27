"""Independent semantic mapping pipeline used by the expert worker."""

import numpy as np

from expert_data_rule.perception.depth_fusion import DepthFusion
from expert_data_rule.perception.depth_obstacles import extract_depth_geometry
from expert_data_rule.perception.geometry import CameraIntrinsics, transform_from_pose
from expert_data_rule.types import PerceptionOutput
from .global_map import GlobalSemanticMap, WorldMapSpec
from .local_map import LocalEvidenceMap, LocalMapSpec


class SemanticMappingPipeline:
    def __init__(self, config):
        self.config = config
        self.intrinsics = CameraIntrinsics.from_config(config["camera"])
        extrinsic = config["camera"]["T_base_camera"]
        self.base_to_camera = transform_from_pose(
            extrinsic["translation_m"], extrinsic["rpy_rad"])
        self.world_spec = WorldMapSpec.from_config(config["map"]["global"])
        self.local_spec = LocalMapSpec.from_config(config["map"]["local"])
        self.global_map = GlobalSemanticMap(self.world_spec)
        self.local_evidence = LocalEvidenceMap(
            self.world_spec, self.local_spec.resolution_m)
        self.fusion = DepthFusion(self.intrinsics, config["depth"])

    def reset(self):
        self.global_map.reset()
        self.local_evidence.reset()

    def update(self, rgb, depth, pose, detections):
        self.intrinsics.validate(rgb, "rgb")
        self.intrinsics.validate(depth, "depth")
        world_to_base = transform_from_pose(
            (pose["x"], pose["y"], pose["z"]),
            (pose["roll"], pose["pitch"], pose["yaw"]))
        camera_to_world = world_to_base @ self.base_to_camera
        fused = self.fusion.process(depth, detections, camera_to_world)
        obstacle, endpoints, pixels, sensor = extract_depth_geometry(
            depth, self.intrinsics, camera_to_world, self.config["depth"])
        robot_xy = (float(pose["x"]), float(pose["y"]))
        global_map = self.global_map.update(
            obstacle, endpoints, pixels, sensor[:2], robot_xy,
            fused["echinus_points"], fused["echinus_scores"],
            fused["rock_points"], fused["rock_scores"], self.config["map"])
        self.local_evidence.update(
            obstacle, robot_xy, fused["echinus_points"], fused["echinus_scores"],
            fused["rock_points"], fused["rock_scores"], self.config["map"])
        local_map = self.local_evidence.render(robot_xy, float(pose["yaw"]),
                                               self.local_spec)
        return PerceptionOutput(tuple(detections), fused["targets"], global_map,
                                local_map, float(sensor[2]))
