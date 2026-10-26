"""Deterministic rule hierarchy from the expert policy document."""

from dataclasses import dataclass
import math
import random

from expert_data_rule.types import Action, ExpertState, Mode
from .map_analyzer import MapAnalyzer, image_rock_turn
from .target_tracker import TargetTracker


@dataclass(frozen=True)
class Decision:
    action: Action = None
    failure_reason: str = ""
    reason: str = ""

    @property
    def failed(self):
        return bool(self.failure_reason)


class RuleBasedExpertPolicy:
    MOTION_ACTIONS = (Action.FORWARD, Action.TURN_LEFT, Action.TURN_RIGHT)

    def __init__(self, config):
        self.config = config
        self.policy_config = config["policy"]
        self.rng = random.Random(int(config["episode"]["random_seed"]))
        self.analyzer = MapAnalyzer(self.policy_config, config["map"])
        self.tracker = TargetTracker(
            self.policy_config, int(config["camera"]["width"]))
        self.reset()

    def reset(self):
        self.state = ExpertState()
        self.tracker.reset()

    def record_completed_action(self, action):
        action = Action(action)
        if action in self.MOTION_ACTIONS:
            self.state.step_count += 1

    def _turn_for_bearing(self, bearing):
        tolerance = float(self.policy_config["center_tolerance_rad"])
        if bearing < -tolerance:
            return Action.TURN_LEFT
        if bearing > tolerance:
            return Action.TURN_RIGHT
        return None

    def _avoid(self, local_map):
        direction = self.analyzer.avoidance_direction(
            local_map,
            self.state.avoid_direction.value if self.state.avoid_direction else None,
            self.rng)
        self.state.mode = Mode.AVOID
        self.state.avoid_direction = Action(direction)
        if self.state.avoid_steps_remaining <= 0:
            self.state.avoid_steps_remaining = int(
                self.policy_config["avoid_turn_steps"])
        self.state.avoid_steps_remaining -= 1
        return Decision(self.state.avoid_direction, reason="front obstacle")

    def decide(self, perception, yaw):
        target = self.tracker.update(perception.targets)
        self.state.target_id = self.tracker.target_id
        self.state.target_bbox = self.tracker.bbox
        self.state.target_distance_m = self.tracker.distance
        self.state.target_last_bearing_rad = self.tracker.last_bearing
        self.state.target_lost_steps = self.tracker.lost_steps
        front_clear = self.analyzer.front_clear(perception.local_map)

        # Success is deliberately checked before the max-step failure.
        if target and target.distance_m <= float(
                self.policy_config["success_distance_m"]):
            stop_clear = (front_clear or not bool(
                self.policy_config.get("stop_requires_front_clear", True)))
            self.state.success_frames = self.state.success_frames+1 if stop_clear else 0
            if self.state.success_frames >= int(
                    self.policy_config["success_confirmation_frames"]):
                return Decision(Action.STOP, reason="target reached")
            if stop_clear:
                # Confirm depth on another sensor frame without moving closer.
                # This also lets a target reached exactly after the final
                # allowed motion action win over max_steps_exceeded.
                return Decision(reason="confirm target distance")
        else:
            self.state.success_frames = 0

        if self.state.step_count >= int(self.policy_config["max_steps"]):
            return Decision(failure_reason="max_steps_exceeded")

        if target:
            self.state.mode = Mode.APPROACH
            self.state.observe_steps = 0
            turn = self._turn_for_bearing(target.bearing_rad)
            if turn:
                return Decision(turn, reason="align target")
            if not front_clear:
                return self._avoid(perception.local_map)
            return Decision(Action.FORWARD, reason="approach target")

        if self.tracker.recovering:
            self.state.mode = Mode.RECOVER_TARGET
            turn = self._turn_for_bearing(self.tracker.last_bearing)
            if turn is None:
                turn = self.state.observe_direction or Action.TURN_LEFT
            return Decision(turn, reason="recover lost target")

        if self.state.observe_steps < int(self.policy_config["observe_turn_steps"]):
            self.state.mode = Mode.OBSERVE
            if self.state.observe_direction is None:
                self.state.observe_direction = Action(image_rock_turn(
                    perception.detections, int(self.config["camera"]["width"]),
                    float(self.policy_config["image_rock_margin"]), self.rng))
            self.state.observe_steps += 1
            return Decision(self.state.observe_direction, reason="panoramic observation")

        self.state.mode = Mode.SEARCH
        if (self.state.search_direction is None or
                self.state.search_forward_steps >= int(
                    self.policy_config["search_forward_steps"])):
            bearing = self.analyzer.best_global_direction(perception.global_map, yaw)
            if bearing is None:
                bearing = self.rng.choice((-math.pi/2, math.pi/2))
            self.state.search_direction = (Action.TURN_LEFT if bearing < 0
                                           else Action.TURN_RIGHT)
            self.state.search_turn_steps = max(
                0, int(math.ceil(abs(bearing)/float(
                    self.policy_config["turn_primitive_rad"]))))
            self.state.search_forward_steps = 0

        if self.state.search_turn_steps > 0:
            self.state.search_turn_steps -= 1
            return Decision(self.state.search_direction, reason="align search direction")
        if not front_clear:
            return self._avoid(perception.local_map)
        self.state.search_forward_steps += 1
        if self.state.search_forward_steps >= int(
                self.policy_config["search_forward_steps"]):
            self.state.observe_steps = 0
            self.state.observe_direction = None
            self.state.search_direction = None
        return Decision(Action.FORWARD, reason="search forward")
