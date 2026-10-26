from expert_data_rule.policy.target_tracker import TargetTracker
from expert_data_rule.types import TargetObservation


def cue(distance, box):
    return TargetObservation(True, 0.0, distance, box, 0.9)


def test_tracker_keeps_target_until_another_is_clearly_closer():
    tracker = TargetTracker({"target_lost_steps": 2,
                             "target_switch_distance_ratio": 0.7,
                             "target_match_iou": 0.1,
                             "target_match_center_ratio": 0.1}, 100)
    original = cue(2.0, (10, 10, 30, 30))
    tracker.update([original])
    kept = tracker.update([cue(2.1, (11, 10, 31, 30)),
                           cue(1.7, (60, 10, 80, 30))])
    assert kept.bbox[0] == 11
    switched = tracker.update([cue(2.0, (12, 10, 32, 30)),
                               cue(1.0, (60, 10, 80, 30))])
    assert switched.bbox[0] == 60


def test_tracker_recovers_for_configured_lost_steps():
    tracker = TargetTracker({"target_lost_steps": 2,
                             "target_switch_distance_ratio": 0.7,
                             "target_match_iou": 0.1,
                             "target_match_center_ratio": 0.1}, 100)
    tracker.update([cue(2.0, (10, 10, 30, 30))])
    tracker.update([])
    assert tracker.recovering
    tracker.update([])
    assert tracker.recovering
    tracker.update([])
    assert not tracker.recovering
