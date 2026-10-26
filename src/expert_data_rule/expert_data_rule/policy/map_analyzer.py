"""Local safety checks and global search-direction scoring."""

import math

import numpy as np

from expert_data_rule.mapping.global_map import GLOBAL_INDEX


class MapAnalyzer:
    def __init__(self, policy_config, map_config):
        self.config = policy_config
        self.global_config = map_config["global"]
        self.local_config = map_config["local"]

    def local_occupancy(self, local_map):
        obstacle = np.asarray(local_map[0], np.float32)
        height, width = obstacle.shape
        forward_rows = max(1, int(height*float(self.config["forward_region_ratio"])))
        corridor_width = max(1, int(width*float(self.config["forward_corridor_ratio"])))
        center = width//2
        left = max(0, center-corridor_width//2)
        right = min(width, left+corridor_width)
        front = obstacle[:forward_rows, left:right]
        left_front = obstacle[:forward_rows, :center]
        right_front = obstacle[:forward_rows, center:]
        return (float(np.mean(front > 0)), float(np.mean(left_front > 0)),
                float(np.mean(right_front > 0)))

    def front_clear(self, local_map):
        front, _, _ = self.local_occupancy(local_map)
        obstacle = np.asarray(local_map[0], np.float32)
        height, width = obstacle.shape
        corridor_width = max(1, int(width*float(
            self.config["forward_corridor_ratio"])))
        center = width//2
        left = max(0, center-corridor_width//2)
        right = min(width, left+corridor_width)
        rows = np.argwhere(obstacle[:height//2, left:right] > 0)[:, 0]
        nearest = (None if not len(rows) else
                   float(np.min((height/2-(rows+0.5))*float(
                       self.local_config["resolution_m"]))))
        distance_safe = (nearest is None or nearest >= float(
            self.config["safe_obstacle_distance_m"]))
        return (front < float(self.config["obstacle_occupancy_threshold"]) and
                distance_safe)

    def avoidance_direction(self, local_map, previous, rng):
        _, left, right = self.local_occupancy(local_map)
        margin = float(self.config["side_clearance_margin"])
        if left+margin < right:
            return "TURN_LEFT"
        if right+margin < left:
            return "TURN_RIGHT"
        return previous or rng.choice(("TURN_LEFT", "TURN_RIGHT"))

    def best_global_direction(self, global_map, yaw):
        robot_cells = np.argwhere(global_map[GLOBAL_INDEX["robot"]] > 0)
        if not len(robot_cells):
            return None
        robot_row, robot_col = robot_cells.mean(axis=0)
        rows, cols = np.indices(global_map.shape[1:])
        dx = (cols-robot_col)*float(self.global_config["resolution_m"])
        dy = (rows-robot_row)*float(self.global_config["resolution_m"])
        distance = np.hypot(dx, dy)
        valid = ((distance >= float(self.config["search_min_radius_m"])) &
                 (distance <= float(self.config["search_max_radius_m"])))
        relative = np.arctan2(dy, dx)-float(yaw)
        relative = np.arctan2(np.sin(relative), np.cos(relative))
        centers = np.asarray([-math.pi/2, -math.pi/4, 0.0,
                              math.pi/4, math.pi/2], np.float32)
        half_width = math.pi/8
        semantic_scores, exploration_scores, rock_amounts = [], [], []
        rock = global_map[GLOBAL_INDEX["rock"]]
        visited = global_map[GLOBAL_INDEX["visited"]]
        explored = global_map[GLOBAL_INDEX["explored"]]
        obstacle = global_map[GLOBAL_INDEX["obstacle"]]
        for center in centers:
            delta = np.arctan2(np.sin(relative-center), np.cos(relative-center))
            mask = valid & (np.abs(delta) <= half_width)
            if not mask.any():
                semantic_scores.append(-float("inf"))
                exploration_scores.append(-float("inf"))
                rock_amounts.append(0.0)
                continue
            rock_evidence = float(np.sum(rock[mask]))
            base = (-float(self.config["visited_penalty_weight"])*np.mean(visited[mask]) -
                    float(self.config["obstacle_penalty_weight"])*np.mean(obstacle[mask]) +
                    float(self.config["unexplored_score_weight"])*(1-np.mean(explored[mask])) -
                    float(self.config["turn_cost_weight"])*abs(float(center)))
            semantic_scores.append(
                base+float(self.config["rock_score_weight"])*rock_evidence)
            exploration_scores.append(base)
            rock_amounts.append(rock_evidence)
        rock_ranked = sorted(range(len(rock_amounts)),
                             key=lambda index: rock_amounts[index], reverse=True)
        best, second = rock_ranked[0], rock_ranked[1]
        if (rock_amounts[best] >= float(self.config["rock_min_evidence"]) and
                rock_amounts[best]-rock_amounts[second] >= float(
                    self.config["rock_direction_margin"])):
            best = max(range(len(semantic_scores)),
                       key=lambda index: semantic_scores[index])
            return float(centers[best])
        # With no clear semantic clue, ignore weak rock noise and explicitly
        # favour unexplored, unvisited, obstacle-free space.
        best = max(range(len(exploration_scores)),
                   key=lambda index: exploration_scores[index])
        return (float(centers[best])
                if np.isfinite(exploration_scores[best]) else None)


def image_rock_turn(detections, image_width, margin, rng):
    left = sum(item.score for item in detections
               if item.class_name == "rock" and (item.bbox[0]+item.bbox[2])/2 < image_width/2)
    right = sum(item.score for item in detections
                if item.class_name == "rock" and (item.bbox[0]+item.bbox[2])/2 >= image_width/2)
    if left > right+margin:
        return "TURN_LEFT"
    if right > left+margin:
        return "TURN_RIGHT"
    return rng.choice(("TURN_LEFT", "TURN_RIGHT"))
