import argparse
import hashlib
import json
import os
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace

parser = argparse.ArgumentParser()
parser.add_argument('root')
parser.add_argument('output')
parser.add_argument('--steps', type=int, default=1100)
parser.add_argument('--sim-clock', action='store_true')
args = parser.parse_args()
root = Path(args.root).resolve()
os.chdir(root / 'deploy')
sys.path[:0] = [str(root / 'deploy'), str(root)]

import numpy as np
from hydra import compose, initialize_config_dir
from hydra.utils import instantiate

result = {'root': str(root), 'requested_steps': args.steps, 'completed_steps': 0}
result['clock_mode'] = 'controlled_simulation_clock' if args.sim_clock else 'unchanged_wall_clock'
agent = None
started = time.monotonic()
trajectory = hashlib.sha256()
if args.sim_clock:
    import envs.hitter as hitter_module
    hitter_module.time = SimpleNamespace(
        monotonic=lambda: 1000.0 + (float(agent.env.simulator.mujoco_data.time) if agent else 0.0),
        time=time.time,
        sleep=time.sleep,
    )
try:
    with initialize_config_dir(config_dir=str(root / 'deploy/config'), version_base=None):
        config = compose(config_name='hitter', overrides=[
            'sim=mujoco', 'device=cpu', 'robot.control.viewer=false',
            'robot.control.real_time=false',
        ])
    agent = instantiate(config.agent)
    result['agent'] = type(agent).__module__ + '.' + type(agent).__name__
    result['env'] = type(agent.env).__module__ + '.' + type(agent.env).__name__
    simulator = agent.env.simulator
    assert not simulator.is_real
    result['simulator'] = type(simulator).__module__ + '.' + type(simulator).__name__
    result['model_inputs'] = [{'name': v.name, 'shape': v.shape} for v in agent.policy.get_inputs()]
    result['model_outputs'] = [{'name': v.name, 'shape': v.shape} for v in agent.policy.get_outputs()]
    obs = agent.env.reset()
    assert {key: list(value.shape) for key, value in obs.items()} == {'obs': [1, 104]}
    result['initial_observation_shapes'] = {key: list(value.shape) for key, value in obs.items()}
    result['initial_simulation_time_s'] = float(simulator.mujoco_data.time)
    for i in range(args.steps):
        obs = agent._run_iteration(obs)
        assert {key: list(value.shape) for key, value in obs.items()} == {'obs': [1, 104]}
        for value in obs.values():
            assert np.all(np.isfinite(value)), ('nonfinite observation', i)
            trajectory.update(np.ascontiguousarray(value).tobytes())
        assert agent.env.prev_policy_action.shape == (29,)
        for value in (agent.env.prev_policy_action, simulator.mujoco_data.qpos,
                      simulator.mujoco_data.qvel, simulator.mujoco_data.ctrl):
            assert np.all(np.isfinite(value)), ('nonfinite state/action', i)
            trajectory.update(np.ascontiguousarray(value).tobytes())
        result['completed_steps'] = i + 1
    result['simulation_time_s'] = float(simulator.mujoco_data.time)
    result['scheduled_serves_emitted'] = int(agent.env._mujoco_next_serve_index)
    result['trajectory_sha256'] = trajectory.hexdigest()
    result['loop_passed'] = True
    assert result['scheduled_serves_emitted'] == 2
except BaseException:
    result['loop_passed'] = False
    result['loop_exception'] = traceback.format_exc()
finally:
    if agent is not None:
        try:
            result['close_result'] = agent.env.close()
        except BaseException:
            result['close_exception'] = traceback.format_exc()
    result['wall_time_s'] = time.monotonic() - started
    Path(args.output).write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps(result, indent=2), flush=True)
sys.exit(0 if result.get('loop_passed') and not result.get('close_exception') else 1)
