"""Rebuild relocated generated artifacts without changing deployment sources."""
import json
import os
from pathlib import Path
import shutil
import subprocess

ROOT = Path('/home/yhl/Desktop/BOB/Hitter/RobotBridge4_refactor')
OUT = Path(__file__).resolve().parent
LCM = Path('/home/yhl/miniforge3/envs/isaaclab/lib/python3.10/site-packages')
SDK = ROOT / 'vicon_datastream_sdk/linux64/Linux64'
results = []

def run(command, name):
    print(name, flush=True)
    env = dict(os.environ)
    env.pop('LD_LIBRARY_PATH', None)
    with (OUT / (name + '.log')).open('w') as log:
        completed = subprocess.run(command, cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)
    results.append({'name': name, 'command': command, 'exit_code': completed.returncode})
    (OUT / 'build_results.json').write_text(json.dumps(results, indent=2) + '\n')
    completed.check_returncode()

backup = OUT / 'previous_builds'
backup.mkdir(exist_ok=True)
for relative, name in [('unitree_sdk2/.build-robotbridge4-v2', 'unitree'),
                       ('deploy/mocap_bridge/.build-v2', 'vicon')]:
    source = ROOT / relative
    target = backup / name
    if target.exists():
        raise RuntimeError(f'Backup already exists: {target}; refusing to overwrite')
    if source.exists():
        shutil.move(str(source), str(target))

build = ROOT / 'unitree_sdk2/.build-robotbridge4-v2'
run(['cmake', '-S', str(ROOT / 'unitree_sdk2'), '-B', str(build),
     '-DCMAKE_BUILD_TYPE=Release', '-DCMAKE_CXX_FLAGS=-I' + str(LCM / 'include'),
     '-DCMAKE_EXE_LINKER_FLAGS=-L' + str(LCM / 'lib') + ' -Wl,-rpath,' + str(LCM / 'lib')],
    'unitree_configure')
run(['cmake', '--build', str(build), '--target', 'trans', '-j2'], 'unitree_build')

build = ROOT / 'deploy/mocap_bridge/.build-v2'
build.mkdir(parents=True, exist_ok=True)
for source in ['nexus_probe_cpp', 'vicon_datastream_dump', 'vicon_frame_stream',
               'vicon_table_lcm_bridge', 'tests/test_transformation_t_v2',
               'tests/test_vicon_ball_track_v2']:
    name = Path(source).name + ('_v2' if source == 'vicon_table_lcm_bridge' else '')
    run(['g++', '-std=c++17', '-O2', '-I' + str(ROOT), '-I' + str(LCM / 'include'),
         '-I' + str(SDK), '-I' + str(ROOT / 'unitree_sdk2/include'),
         str(ROOT / 'deploy/mocap_bridge' / (source + '.cpp')),
         '-L' + str(SDK), '-Wl,-rpath,' + str(SDK), '-lViconDataStreamSDK_CPP',
         str(ROOT / 'unitree_sdk2/lib/x86_64/libunitree_sdk2.a'),
         '-L' + str(LCM / 'lib'), '-Wl,-rpath,' + str(LCM / 'lib'), '-llcm', '-o', str(build / name)],
        'build_' + name)
print('All relocated binaries rebuilt successfully.', flush=True)
