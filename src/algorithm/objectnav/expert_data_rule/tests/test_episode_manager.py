from expert_data_rule.runtime.episode_manager import EpisodeManager


def test_continuous_lifecycle_and_limit():
    manager = EpisodeManager(True, max_episodes=2)
    for _ in range(2):
        assert manager.may_start()
        manager.begin_preparing()
        manager.began()
        manager.ended()
    assert not manager.enabled
    assert not manager.may_start()


def test_pause_prevents_next_episode():
    manager = EpisodeManager(True)
    manager.pause()
    assert not manager.may_start()
    manager.start()
    assert manager.may_start()
