import json
import os
from pathlib import Path
import signal
import subprocess
import time
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--root', default='/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor')
parser.add_argument('--suffix', default='')
parser.add_argument('--duration', type=float, default=28)
args = parser.parse_args()
root = Path(args.root)
out = Path(__file__).resolve().parent
command = ['/home/yhl/miniforge3/envs/isaaclab/bin/python', '-u', 'run.py',
           '--config-name=hitter', 'sim=mujoco', 'device=cpu',
           'hydra.run.dir=/tmp/rb4-startup-check/hydra-gui' + args.suffix]
env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONUNBUFFERED='1',
           HYDRA_FULL_ERROR='1')
env.pop('MUJOCO_GL', None)
start = time.monotonic()
with (out / ('mujoco_actual_cli' + args.suffix + '.log')).open('w') as log:
    process = subprocess.Popen(command, cwd=root/'deploy', env=env,
                               stdout=log, stderr=subprocess.STDOUT,
                               start_new_session=True)
    result = {'command': command, 'cwd': str(root/'deploy'), 'display': env.get('DISPLAY')}
    try:
        result['exit_code'] = process.wait(timeout=args.duration)
        result['stopped_by_test'] = False
    except subprocess.TimeoutExpired:
        result['alive_at_test_deadline'] = True
        result['test_duration_s'] = args.duration
        result['stopped_by_test'] = True
        log.write(f'\nSTARTUP_AUDIT: sending SIGINT after {args.duration} seconds\n')
        log.flush()
        os.killpg(process.pid, signal.SIGINT)
        try:
            result['exit_code'] = process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            result['exit_code'] = process.wait(timeout=5)
            result['required_sigkill'] = True
result['wall_time_s'] = time.monotonic() - start
result['log'] = str(out/('mujoco_actual_cli' + args.suffix + '.log'))
(out/('mujoco_actual_cli' + args.suffix + '.json')).write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps(result,indent=2))
