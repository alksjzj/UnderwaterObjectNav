"""Data contracts shared by perception, mapping, and policy."""

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional, Tuple

import numpy as np


class Action(str, Enum):
    FORWARD = "FORWARD"
    TURN_LEFT = "TURN_LEFT"
    TURN_RIGHT = "TURN_RIGHT"
    STOP = "STOP"


class Mode(str, Enum):
    OBSERVE = "OBSERVE"
    APPROACH = "APPROACH"
    RECOVER_TARGET = "RECOVER_TARGET"
    AVOID = "AVOID"
    SEARCH = "SEARCH"


@dataclass(frozen=True)
class Detection:
    bbox: Tuple[float, float, float, float]
    score: float
    class_name: str


@dataclass(frozen=True)
class TargetObservation:
    visible: bool = False
    bearing_rad: float = 0.0
    distance_m: float = 0.0
    bbox: Optional[Tuple[float, float, float, float]] = None
    score: float = 0.0


@dataclass(frozen=True)
class PerceptionOutput:
    detections: Tuple[Detection, ...]
    targets: Tuple[TargetObservation, ...]
    global_map: np.ndarray
    local_map: np.ndarray
    sensor_world_z_m: float


@dataclass
class ExpertState:
    mode: Mode = Mode.OBSERVE
    step_count: int = 0
    observe_direction: Optional[Action] = None
    observe_steps: int = 0
    avoid_direction: Optional[Action] = None
    avoid_steps_remaining: int = 0
    search_direction: Optional[Action] = None
    search_turn_steps: int = 0
    search_forward_steps: int = 0
    target_id: Optional[int] = None
    target_bbox: Optional[Tuple[float, float, float, float]] = None
    target_distance_m: Optional[float] = None
    target_last_bearing_rad: float = 0.0
    target_lost_steps: int = 0
    success_frames: int = 0
    rng_state: dict = field(default_factory=dict)
