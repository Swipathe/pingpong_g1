from __future__ import annotations

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
from omegaconf import OmegaConf

from envs.hitter import HitterEnv

DEPLOY_DIR = Path(__file__).resolve().parents[1]
MODEL20500_CHECKPOINT = "./data/model/hitter/hitter_model20500_20260818_104.onnx"


def test_hitter_deployment_uses_model20500_checkpoint():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )

    checkpoint = mimic_config["policy"]["checkpoint"]
    assert checkpoint == MODEL20500_CHECKPOINT
    assert (DEPLOY_DIR / checkpoint[2:]).is_file()


def test_racket_velocity_magnitude_increment_reads_deployment_key_and_defaults_to_zero():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )

    assert HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(mimic_config["policy"]) == 0.0
    assert HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg({}) == 0.0


def test_racket_velocity_magnitude_increment_accepts_dictconfig_policy_node():
    mimic_config = OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml")

    assert HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(mimic_config.policy) == 0.0
    assert HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(OmegaConf.create({})) == 0.0


def test_forehand_policy_vx_offset_reads_deployment_key_and_defaults_to_zero():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )

    assert HitterEnv._forehand_policy_vx_offset_from_policy_cfg(mimic_config["policy"]) == 0.0
    assert HitterEnv._forehand_policy_vx_offset_from_policy_cfg({}) == 0.0


def test_forehand_policy_vx_offset_accepts_dictconfig_policy_node():
    mimic_config = OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml")

    assert HitterEnv._forehand_policy_vx_offset_from_policy_cfg(mimic_config.policy) == 0.0
    assert HitterEnv._forehand_policy_vx_offset_from_policy_cfg(OmegaConf.create({})) == 0.0


def test_real_forehand_policy_velocity_has_no_production_compensation():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.racket_velocity_magnitude_increment_mps = HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(
        mimic_config["policy"]
    )
    env.forehand_policy_vx_offset_mps = HitterEnv._forehand_policy_vx_offset_from_policy_cfg(mimic_config["policy"])

    (
        policy_velocity,
        magnitude_increment,
        policy_vx_offset,
        policy_vy_offset,
    ) = env._policy_racket_target_velocity_w(
        [1.0, -0.2, 0.3],
        strike_type="forehand",
    )

    raw_velocity = np.asarray([1.0, -0.2, 0.3], dtype=np.float64)
    np.testing.assert_allclose(policy_velocity, raw_velocity, rtol=1.0e-6)
    assert magnitude_increment == 0.0
    assert policy_vx_offset == 0.0
    assert policy_vy_offset == 0.0


@pytest.mark.parametrize(
    "value",
    [
        True,
        "1.0",
        float("nan"),
        float("inf"),
        -0.01,
        float(np.finfo(np.float32).max) * 2.0,
        1.0e100,
    ],
)
def test_racket_velocity_magnitude_increment_rejects_invalid_values(value):
    with pytest.raises((TypeError, ValueError)):
        HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(
            {
                "real_world_racket_velocity_magnitude_increment_mps": value,
            }
        )


@pytest.mark.parametrize("strike_type", ["forehand", "backhand"])
def test_real_magnitude_increment_leaves_zero_planner_velocity_unchanged(
    strike_type,
):
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.racket_velocity_magnitude_increment_mps = 1.0

    policy_velocity, magnitude_increment, policy_vx_offset, policy_vy_offset = env._policy_racket_target_velocity_w(
        [0.0, 0.0, 0.0],
        strike_type=strike_type,
    )

    np.testing.assert_array_equal(policy_velocity, [0.0, 0.0, 0.0])
    assert magnitude_increment == 0.0
    assert policy_vx_offset == 0.0
    assert policy_vy_offset == 0.0


@pytest.mark.parametrize("strike_type", ["forehand", "backhand"])
def test_simulation_does_not_apply_real_world_magnitude_increment(strike_type):
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=False)
    env.racket_velocity_magnitude_increment_mps = 1.0

    policy_velocity, magnitude_increment, policy_vx_offset, policy_vy_offset = env._policy_racket_target_velocity_w(
        [3.0, 4.0, 0.0],
        strike_type=strike_type,
    )

    np.testing.assert_allclose(policy_velocity, [3.0, 4.0, 0.0])
    assert magnitude_increment == 0.0
    assert policy_vx_offset == 0.0
    assert policy_vy_offset == 0.0


def test_real_magnitude_increment_rejects_float32_output_overflow():
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.racket_velocity_magnitude_increment_mps = float(np.finfo(np.float32).max)

    with pytest.raises(ValueError, match="float32"):
        env._policy_racket_target_velocity_w(
            [np.finfo(np.float32).max, 0.0, 0.0],
            strike_type="forehand",
        )


@pytest.mark.parametrize(
    "value",
    [
        True,
        "0.25",
        float("nan"),
        float("inf"),
        -0.01,
        float(np.finfo(np.float32).max) * 2.0,
        1.0e100,
    ],
)
def test_forehand_policy_vx_offset_rejects_invalid_or_unrepresentable_values(
    value,
):
    with pytest.raises((TypeError, ValueError)):
        HitterEnv._forehand_policy_vx_offset_from_policy_cfg(
            {
                "real_world_forehand_racket_velocity_x_offset_mps": value,
            }
        )


def test_real_forehand_policy_velocity_rejects_float32_addition_overflow():
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.forehand_policy_vx_offset_mps = float(np.finfo(np.float32).max)

    with pytest.raises(ValueError, match="float32"):
        env._policy_racket_target_velocity_w(
            [np.finfo(np.float32).max, 0.1, 0.2],
            strike_type="forehand",
        )


def test_backhand_policy_vx_offset_reads_deployment_key_and_defaults_to_zero():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )

    assert HitterEnv._backhand_policy_vx_offset_from_policy_cfg(mimic_config["policy"]) == 0.0
    assert HitterEnv._backhand_policy_vx_offset_from_policy_cfg({}) == 0.0


def test_backhand_policy_vy_decrement_reads_deployment_key_and_defaults_to_zero():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )

    assert HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(mimic_config["policy"]) == 0.0
    assert HitterEnv._backhand_policy_vy_decrement_from_policy_cfg({}) == 0.0


def test_real_backhand_policy_velocity_has_no_production_compensation():
    mimic_config = OmegaConf.to_container(
        OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml"),
        resolve=True,
    )
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.racket_velocity_magnitude_increment_mps = HitterEnv._racket_velocity_magnitude_increment_from_policy_cfg(
        mimic_config["policy"]
    )
    env.backhand_policy_vx_offset_mps = HitterEnv._backhand_policy_vx_offset_from_policy_cfg(mimic_config["policy"])
    env.backhand_policy_vy_decrement_mps = HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(
        mimic_config["policy"]
    )

    (
        policy_velocity,
        magnitude_increment,
        policy_vx_offset,
        policy_vy_offset,
    ) = env._policy_racket_target_velocity_w(
        [1.0, -0.2, 0.3],
        strike_type="backhand",
    )

    raw_velocity = np.asarray([1.0, -0.2, 0.3], dtype=np.float64)
    np.testing.assert_allclose(policy_velocity, raw_velocity, rtol=1.0e-6)
    assert magnitude_increment == 0.0
    assert policy_vx_offset == 0.0
    assert policy_vy_offset == 0.0


def test_backhand_policy_vx_offset_accepts_dictconfig_policy_node():
    mimic_config = OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml")

    assert HitterEnv._backhand_policy_vx_offset_from_policy_cfg(mimic_config.policy) == 0.0
    assert HitterEnv._backhand_policy_vx_offset_from_policy_cfg(OmegaConf.create({})) == 0.0


def test_backhand_policy_vy_decrement_accepts_dictconfig_policy_node():
    mimic_config = OmegaConf.load(DEPLOY_DIR / "config/mimic/hitter.yaml")

    assert HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(mimic_config.policy) == 0.0
    assert HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(OmegaConf.create({})) == 0.0


@pytest.mark.parametrize(
    "value",
    [
        True,
        "0.3",
        float("nan"),
        float("inf"),
        -0.01,
        float(np.finfo(np.float32).max) * 2.0,
        1.0e100,
    ],
)
def test_backhand_policy_vy_decrement_rejects_invalid_or_unrepresentable_values(
    value,
):
    with pytest.raises((TypeError, ValueError)):
        HitterEnv._backhand_policy_vy_decrement_from_policy_cfg(
            {
                "real_world_backhand_racket_velocity_y_decrement_mps": value,
            }
        )


@pytest.mark.parametrize(
    "value",
    [
        True,
        "2.0",
        float("nan"),
        float("inf"),
        -0.01,
        float(np.finfo(np.float32).max) * 2.0,
        1.0e100,
    ],
)
def test_backhand_policy_vx_offset_rejects_invalid_or_unrepresentable_values(
    value,
):
    with pytest.raises((TypeError, ValueError)):
        HitterEnv._backhand_policy_vx_offset_from_policy_cfg(
            {
                "real_world_backhand_racket_velocity_x_offset_mps": value,
            }
        )


def test_real_backhand_policy_velocity_rejects_float32_x_addition_overflow():
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.backhand_policy_vx_offset_mps = float(np.finfo(np.float32).max)
    env.backhand_policy_vy_decrement_mps = 0.0

    with pytest.raises(ValueError, match="float32"):
        env._policy_racket_target_velocity_w(
            [np.finfo(np.float32).max, 0.1, 0.2],
            strike_type="backhand",
        )


def test_real_backhand_policy_velocity_rejects_float32_subtraction_overflow():
    env = HitterEnv.__new__(HitterEnv)
    env.simulator = SimpleNamespace(is_real=True)
    env.backhand_policy_vx_offset_mps = 0.0
    env.backhand_policy_vy_decrement_mps = float(np.finfo(np.float32).max)

    with pytest.raises(ValueError, match="float32"):
        env._policy_racket_target_velocity_w(
            [1.0, -np.finfo(np.float32).max, 0.2],
            strike_type="backhand",
        )
