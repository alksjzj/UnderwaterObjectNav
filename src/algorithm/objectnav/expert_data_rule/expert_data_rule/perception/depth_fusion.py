"""Fuse echinus/rock detections with aligned depth."""

import math

import numpy as np

from expert_data_rule.types import TargetObservation
from .geometry import pixels_to_world, valid_depth


def _bounds(box, width, height):
    x1, y1, x2, y2 = map(float, box)
    return (max(0, int(math.floor(min(x1, x2)))),
            max(0, int(math.floor(min(y1, y2)))),
            min(width, int(math.ceil(max(x1, x2)))),
            min(height, int(math.ceil(max(y1, y2)))))


class DepthFusion:
    def __init__(self, intrinsics, config):
        self.intrinsics = intrinsics
        self.config = config
        self.minimum = float(config["min_depth_m"])
        self.maximum = float(config["max_depth_m"])

    def _median(self, depth, box, ratio, minimum_pixels):
        x1, y1, x2, y2 = map(float, box)
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        width = max(float(self.config["min_roi_size_px"]), abs(x2 - x1) * ratio)
        height = max(float(self.config["min_roi_size_px"]), abs(y2 - y1) * ratio)
        left, top, right, bottom = _bounds(
            (cx-width/2, cy-height/2, cx+width/2, cy+height/2),
            depth.shape[1], depth.shape[0])
        values = depth[top:bottom, left:right]
        values = values[valid_depth(values, self.minimum, self.maximum)]
        return None if values.size < minimum_pixels else float(np.median(values))

    def process(self, depth, detections, camera_to_world):
        targets, echinus_points, echinus_scores = [], [], []
        rock_points, rock_scores = [], []
        for detection in detections:
            if detection.class_name == "echinus":
                distance = self._median(
                    depth, detection.bbox,
                    float(self.config["echinus_center_roi_ratio"]),
                    int(self.config["min_valid_depth_pixels"]))
                if distance is None:
                    continue
                x1, y1, x2, y2 = detection.bbox
                u = float(np.clip((x1+x2)/2, 0, depth.shape[1]-1))
                v = float(np.clip((y1+y2)/2, 0, depth.shape[0]-1))
                point = pixels_to_world([[u, v]], [distance], self.intrinsics,
                                        camera_to_world)[0]
                bearing = math.atan((u-self.intrinsics.cx)/self.intrinsics.fx)
                targets.append(TargetObservation(
                    True, bearing, distance, detection.bbox, detection.score))
                echinus_points.append(point)
                echinus_scores.append(detection.score)
            elif detection.class_name == "rock":
                reference = self._median(
                    depth, detection.bbox,
                    float(self.config["rock_center_roi_ratio"]),
                    int(self.config["min_valid_depth_pixels"]))
                if reference is None:
                    continue
                left, top, right, bottom = _bounds(
                    detection.bbox, depth.shape[1], depth.shape[0])
                stride = int(self.config["rock_depth_stride"])
                rows, cols = np.mgrid[top:bottom:stride, left:right:stride]
                values = depth[top:bottom:stride, left:right:stride]
                mask = valid_depth(values, self.minimum, self.maximum)
                mask &= np.abs(values-reference) <= float(
                    self.config["rock_depth_gate_m"])
                if mask.any():
                    points = pixels_to_world(
                        np.column_stack((cols[mask], rows[mask])), values[mask],
                        self.intrinsics, camera_to_world)
                    rock_points.append(points)
                    rock_scores.append(np.full(len(points), detection.score,
                                               dtype=np.float32))
        empty_points = np.empty((0, 3), np.float32)
        empty_scores = np.empty(0, np.float32)
        return {
            "targets": tuple(targets),
            "echinus_points": np.asarray(echinus_points, np.float32).reshape(-1, 3),
            "echinus_scores": np.asarray(echinus_scores, np.float32),
            "rock_points": np.concatenate(rock_points) if rock_points else empty_points,
            "rock_scores": np.concatenate(rock_scores) if rock_scores else empty_scores,
        }
