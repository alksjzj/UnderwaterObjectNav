"""Stable nearest-echinus tracking without a learned tracker."""

import math


def _iou(first, second):
    ax1, ay1, ax2, ay2 = first
    bx1, by1, bx2, by2 = second
    width = max(0.0, min(ax2, bx2)-max(ax1, bx1))
    height = max(0.0, min(ay2, by2)-max(ay1, by1))
    intersection = width*height
    first_area = max(0.0, ax2-ax1)*max(0.0, ay2-ay1)
    second_area = max(0.0, bx2-bx1)*max(0.0, by2-by1)
    union = first_area+second_area-intersection
    return intersection/union if union > 0 else 0.0


def _center_distance(first, second):
    ac = ((first[0]+first[2])/2, (first[1]+first[3])/2)
    bc = ((second[0]+second[2])/2, (second[1]+second[3])/2)
    return math.hypot(ac[0]-bc[0], ac[1]-bc[1])


class TargetTracker:
    def __init__(self, config, image_width):
        self.lost_limit = int(config["target_lost_steps"])
        self.switch_ratio = float(config["target_switch_distance_ratio"])
        self.match_iou = float(config["target_match_iou"])
        self.match_center_px = float(config.get(
            "target_match_center_ratio", 0.15))*float(image_width)
        self.reset()

    def reset(self):
        self.next_id = 1
        self._clear()

    def _clear(self):
        self.bbox = None
        self.distance = None
        self.last_bearing = 0.0
        self.lost_steps = 0
        self.target_id = None

    def update(self, targets):
        targets = [item for item in targets if item.visible and item.bbox]
        if not targets:
            if self.bbox is not None:
                self.lost_steps += 1
                if self.lost_steps > self.lost_limit:
                    self._clear()
            return None

        nearest = min(targets, key=lambda item: item.distance_m)
        selected = None
        new_identity = self.bbox is None
        if self.bbox is not None:
            matches = [item for item in targets
                       if (_iou(self.bbox, item.bbox) >= self.match_iou or
                           _center_distance(self.bbox, item.bbox) <= self.match_center_px)]
            if matches:
                selected = max(matches, key=lambda item: _iou(self.bbox, item.bbox))
                if nearest.distance_m < selected.distance_m*self.switch_ratio:
                    selected = nearest
                    new_identity = True
            elif (self.distance is not None and
                  nearest.distance_m < self.distance*self.switch_ratio):
                selected = nearest
                new_identity = True
            else:
                self.lost_steps += 1
                if self.lost_steps <= self.lost_limit:
                    return None
                self._clear()
                new_identity = True
        if selected is None:
            selected = nearest
        if new_identity:
            self.target_id = self.next_id
            self.next_id += 1
        self.bbox = selected.bbox
        self.distance = selected.distance_m
        self.last_bearing = selected.bearing_rad
        self.lost_steps = 0
        return selected

    @property
    def recovering(self):
        return self.bbox is not None and 0 < self.lost_steps <= self.lost_limit
