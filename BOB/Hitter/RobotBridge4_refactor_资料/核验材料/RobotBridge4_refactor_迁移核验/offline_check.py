"""Local loading/configuration checks; no robot control or Vicon connection."""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path('/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor')
OUT = Path(__file__).resolve().parent
PYTHON = '/home/yhl/miniforge3/envs/isaaclab/bin/python'
OLD = '/home/yhl/Desktop/RobotBridge4_refactor'
ENV = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', HYDRA_FULL_ERROR='1')
ENV.pop('LD_LIBRARY_PATH', None)
results = []

def run(name, command, *, cwd=ROOT, env=ENV):
    done = subprocess.run(command, cwd=cwd, env=env, capture_output=True, text=True, timeout=60)
    (OUT / (name + '.log')).write_text(done.stdout + done.stderr)
    results.append({'name': name, 'command': command, 'cwd': str(cwd), 'exit_code': done.returncode})
    (OUT / 'offline_results.json').write_text(json.dumps(results, indent=2) + '\n')
    print(name, done.returncode, flush=True)
    done.check_returncode()
    return done.stdout + done.stderr

for name, binary in [
    ('trans', ROOT / 'unitree_sdk2/.build-robotbridge4-v2/bin/trans'),
    ('vicon_bridge', ROOT / 'deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2'),
    ('vicon_frame_stream', ROOT / 'deploy/mocap_bridge/.build-v2/vicon_frame_stream'),
]:
    linked = run(name + '_ldd', ['ldd', str(binary)])
    dynamic = run(name + '_dynamic', ['readelf', '-d', str(binary)])
    assert 'not found' not in linked, linked
    assert OLD not in dynamic, dynamic

cache = (ROOT / 'unitree_sdk2/.build-robotbridge4-v2/CMakeCache.txt').read_text()
assert OLD not in cache
for test in ['test_transformation_t_v2', 'test_vicon_ball_track_v2']:
    run(test, [str(ROOT / 'deploy/mocap_bridge/.build-v2' / test)])
run('bridge_help', [str(ROOT / 'deploy/mocap_bridge/.build-v2/vicon_table_lcm_bridge_v2'), '--help'])
run('calibration_preflight', ['bash', 'deploy/mocap_bridge/run_robotbridge4_vicon_real.sh', '--check-only'])
for mode in ['mujoco', 'real_world']:
    run('config_' + mode, [PYTHON, 'run.py', '--config-name=hitter', 'sim=' + mode,
                           'device=cpu', '--cfg', 'job', '--resolve'], cwd=ROOT / 'deploy')

sys.path.insert(0, str(ROOT))
from deploy.mocap_bridge.vicon_sdk_client import ViconSdkClient
client = ViconSdkClient(env={'LD_LIBRARY_PATH': ''})
assert str(client.helper_path).startswith(str(ROOT))
run('python_helper_ldd', ['ldd', str(client.helper_path)], env=client._build_env())
run('python_helper_help', [str(client.helper_path), '--help'], env=client._build_env())
print('Offline checks passed.', flush=True)
