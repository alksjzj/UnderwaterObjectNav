"""YAML configuration loading and validation."""

from pathlib import Path

import yaml


def load_config(path):
    source = Path(path).expanduser().resolve()
    with source.open(encoding="utf-8") as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError("expert configuration must be a YAML mapping")
    document["_source"] = str(source)
    _validate(document)
    return document


def resolve_path(config, value):
    path = Path(str(value)).expanduser()
    if not path.is_absolute():
        path = Path(config["_source"]).parent / path
    return path.resolve()


def _positive(group, names):
    for name in names:
        if float(group[name]) <= 0.0:
            raise ValueError("%s must be positive" % name)


def _validate(config):
    required = ("ros", "detector", "camera", "depth", "map", "policy",
                "timing", "episode", "spawn")
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError("missing configuration groups: %s" % missing)
    _positive(config["camera"], ("width", "height", "fx", "fy"))
    _positive(config["depth"], ("max_depth_m", "obstacle_depth_stride"))
    _positive(config["policy"], ("max_steps", "success_distance_m",
                                 "observe_turn_steps", "search_forward_steps"))
    if float(config["depth"]["min_depth_m"]) >= float(
            config["depth"]["max_depth_m"]):
        raise ValueError("depth min must be smaller than depth max")
    if int(config["episode"].get("max_episodes", 0)) < 0:
        raise ValueError("episode.max_episodes cannot be negative")
