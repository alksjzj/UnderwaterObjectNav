"""Small ROS-independent lifecycle state helper."""

from enum import Enum


class Lifecycle(str, Enum):
    PAUSED = "PAUSED"
    PREPARING = "PREPARING"
    RUNNING = "RUNNING"
    ENDING = "ENDING"


class EpisodeManager:
    def __init__(self, auto_start=True, max_episodes=0):
        self.enabled = bool(auto_start)
        self.max_episodes = int(max_episodes)
        self.completed = 0
        self.active = False
        self.state = Lifecycle.PAUSED

    def start(self):
        self.enabled = True

    def pause(self):
        self.enabled = False

    def may_start(self):
        return (self.enabled and not self.active and
                (self.max_episodes == 0 or self.completed < self.max_episodes))

    def begin_preparing(self):
        if not self.may_start():
            raise RuntimeError("episode cannot start")
        self.state = Lifecycle.PREPARING

    def began(self):
        self.active = True
        self.state = Lifecycle.RUNNING

    def ended(self):
        if self.active:
            self.completed += 1
        self.active = False
        if self.max_episodes and self.completed >= self.max_episodes:
            self.enabled = False
        self.state = Lifecycle.PAUSED
