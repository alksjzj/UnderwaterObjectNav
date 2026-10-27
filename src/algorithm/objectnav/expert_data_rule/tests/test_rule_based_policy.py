import numpy as np

from expert_data_rule.policy import RuleBasedExpertPolicy
from expert_data_rule.types import Detection, PerceptionOutput, TargetObservation


def config():
    return {
        "camera": {"width": 100},
        "episode": {"random_seed": 1},
        "map": {"global": {"resolution_m": 0.1},
                "local": {"resolution_m": 0.1}},
        "policy": {
            "target_lost_steps": 2, "target_switch_distance_ratio": 0.7,
            "target_match_iou": 0.1, "target_match_center_ratio": 0.2,
            "forward_region_ratio": 0.5, "forward_corridor_ratio": 0.3,
            "obstacle_occupancy_threshold": 0.1, "side_clearance_margin": 0.01,
            "safe_obstacle_distance_m": 0.45,
            "center_tolerance_rad": 0.1, "success_distance_m": 0.6,
            "success_confirmation_frames": 1, "stop_requires_front_clear": True,
            "max_steps": 2, "observe_turn_steps": 2, "avoid_turn_steps": 2,
            "search_forward_steps": 2, "turn_primitive_rad": 0.25,
            "image_rock_margin": 0.2, "search_min_radius_m": 0.1,
            "search_max_radius_m": 5.0, "rock_score_weight": 1.0,
            "visited_penalty_weight": 1.0, "obstacle_penalty_weight": 1.0,
            "unexplored_score_weight": 1.0, "turn_cost_weight": 0.1,
            "rock_min_evidence": 1.0, "rock_direction_margin": 0.1,
        },
    }


def output(targets=(), obstacle=False):
    local = np.zeros((4, 10, 10), np.float32)
    if obstacle:
        local[0, :5, 4:7] = 1
    global_map = np.zeros((6, 20, 20), np.float32)
    global_map[3, 10, 10] = 1
    return PerceptionOutput((), tuple(targets), global_map, local, 3.3)


def target(distance, bearing=0.0):
    return TargetObservation(True, bearing, distance, (20, 20, 40, 40), 0.9)


def test_success_precedes_max_steps():
    policy = RuleBasedExpertPolicy(config())
    policy.state.step_count = 2
    decision = policy.decide(output([target(0.5)]), 0.0)
    assert decision.action.value == "STOP"
    assert not decision.failed


def test_success_confirmation_does_not_create_extra_motion_at_limit():
    settings = config()
    settings["policy"]["success_confirmation_frames"] = 2
    policy = RuleBasedExpertPolicy(settings)
    policy.state.step_count = 2
    first = policy.decide(output([target(0.5)]), 0.0)
    second = policy.decide(output([target(0.5)]), 0.0)
    assert first.action is None and not first.failed
    assert second.action.value == "STOP"


def test_max_steps_fails_without_a_101st_motion_action():
    policy = RuleBasedExpertPolicy(config())
    policy.state.step_count = 2
    decision = policy.decide(output(), 0.0)
    assert decision.action is None
    assert decision.failure_reason == "max_steps_exceeded"


def test_target_alignment_and_obstacle_avoidance():
    policy = RuleBasedExpertPolicy(config())
    assert policy.decide(output([target(2.0, -0.3)]), 0.0).action.value == "TURN_LEFT"
    assert policy.decide(output([target(2.0)], obstacle=True), 0.0).action.value in (
        "TURN_LEFT", "TURN_RIGHT")
