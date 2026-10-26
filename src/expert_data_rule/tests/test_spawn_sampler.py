from pathlib import Path

from expert_data_rule.runtime.spawn_sampler import SceneSpawnSampler
from expert_data_rule.config import load_config


def test_sampler_stays_inside_pool_and_outside_inflated_rock(tmp_path):
    mesh = tmp_path/"rock.obj"
    mesh.write_text("v -0.5 -0.5 0\nv 0.5 0.5 1\n", encoding="utf-8")
    scene = tmp_path/"scene.scn"
    scene.write_text("""<?xml version='1.0'?>
<scenario>
  <static name='PoolFloor' type='box'>
    <dimensions xyz='10 6 0.2'/><world_transform xyz='0 0 4' rpy='0 0 0'/>
  </static>
  <static name='RockSurface01' type='model'><physical>
    <mesh filename='rock.obj' scale='1'/><origin xyz='0 0 0' rpy='0 0 0'/>
  </physical><world_transform xyz='0 0 4' rpy='0 0 0'/></static>
</scenario>""", encoding="utf-8")
    sampler = SceneSpawnSampler(scene, {
        "pool_static_name": "PoolFloor", "rock_name_regex": "^RockSurface",
        "boundary_margin_m": 0.5, "rock_clearance_m": 0.5,
        "fixed_z_m": 3.3, "max_sampling_attempts": 1000}, seed=2)
    for _ in range(100):
        pose = sampler.sample()
        assert sampler.pool.contains(pose.x, pose.y)
        assert not any(rock.contains(pose.x, pose.y) for rock in sampler.rocks)
        assert pose.z == 3.3


def test_sampler_parses_the_real_rock_seaurchin_scene():
    source_root = Path(__file__).resolve().parents[2]
    package_root = source_root/"expert_data_rule"
    config = load_config(package_root/"config"/"rule_expert.yaml")
    sampler = SceneSpawnSampler(
        source_root/"aquaflow_stonefish"/"scenarios"/"Rock_SeaUrchin.scn",
        config["spawn"], 42,
        lambda name: str(source_root/name))
    poses = [sampler.sample() for _ in range(1000)]
    assert len(sampler.rocks) == 6
    assert all(sampler.pool.contains(pose.x, pose.y) for pose in poses)
    assert all(not any(rock.contains(pose.x, pose.y) for rock in sampler.rocks)
               for pose in poses)
