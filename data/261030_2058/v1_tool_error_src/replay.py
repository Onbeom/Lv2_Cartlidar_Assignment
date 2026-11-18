"""새 모델 · 새 미로 대조 bag Cartographer CPU 재생(261030_1820/replay_control.py 에서 frame · 종류만 바꿈). 자식 PID/프로세스 그룹만 종료한다."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

p = argparse.ArgumentParser()
p.add_argument('kind', choices=['lidar_only', 'wheel_odom', 'truth_odom'])
p.add_argument('light', choices=['bright', 'dark'])
a = p.parse_args()
HERE = Path(__file__).resolve().parent
OUT = HERE / 'results_v2'
dest = OUT / f'carto_{a.kind}_{a.light}'
dest.mkdir(exist_ok=False)
binpath = Path('/home/pa4/git/ros_test/waffle-lyrical-slam/install/cartographer_ros/lib/cartographer_ros')
config = (HERE / 'carto_c6_2lidar_frozen.lua').read_text()
# 새 URDF 루트 = base_link, odom→base_link (팀 dev 규약). c6 는 그대로 두고 frame 이름만 바꿈
assert config.count('tracking_frame = "base_footprint"') == 1
config = config.replace('tracking_frame = "base_footprint"', 'tracking_frame = "base_link"')
if a.kind == 'lidar_only':
    # odom 없이 라이다만: Cartographer가 map→odom→base_link 를 모두 발행
    for old, new in [('published_frame = "odom"', 'published_frame = "base_link"'),
                     ('provide_odom_frame = false', 'provide_odom_frame = true'),
                     ('use_odometry = true', 'use_odometry = false')]:
        assert config.count(old) == 1, old
        config = config.replace(old, new)
(dest / 'config.lua').write_text(config)
children, streams = [], []
report = {'kind': a.kind, 'light': a.light, 'simulation_only': True, 'model': '261029_1030/urdf_261028', 'maze': 'maze.zip (261030_1943)',
          'truth_as_input': a.kind == 'truth_odom', 'diagnostic_only': True}


def start(name, cmd):
    stream = (dest / (name + '.log')).open('w')
    streams.append(stream)
    child = subprocess.Popen(cmd, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
    children.append((name, child))
    return child


def call(name, cmd, limit=120):     # 새 녹화는 경로가 길어 마무리 최적화가 더 걸릴 수 있음
    with (dest / (name + '.log')).open('w') as stream:
        result = subprocess.run(cmd, stdout=stream, stderr=subprocess.STDOUT, timeout=limit)
    report[name + '_exit'] = result.returncode
    if result.returncode:
        raise RuntimeError(name + ' 실패')


try:
    cmd = [str(binpath / 'cartographer_node'), '-configuration_directory', str(dest), '-configuration_basename', 'config.lua',
           '--ros-args', '-p', 'use_sim_time:=true', '-r', 'scan_1:=/scan_front', '-r', 'scan_2:=/scan_rear']
    if a.kind == 'truth_odom':
        cmd += ['-r', 'odom:=/odom_truth']
    elif a.kind == 'wheel_odom':
        cmd += ['-r', 'odom:=/odom']
    carto = start('cartographer', cmd)
    start('record', ['ros2', 'bag', 'record', '--use-sim-time', '-s', 'mcap', '-o', str(dest / 'out_bag'), '--topics', '/tf', '/clock'])
    time.sleep(6)
    if carto.poll() is not None:
        raise RuntimeError('Cartographer 기동 실패')
    play = start('play', ['ros2', 'bag', 'play', str(OUT / f'bag_{a.kind}_{a.light}'), '--rate', '1.0', '--disable-keyboard-controls'])
    play.wait(timeout=1200)
    report['play_exit'] = play.returncode
    if play.returncode:
        raise RuntimeError('재생 실패')
    time.sleep(3)
    call('finish', ['ros2', 'service', 'call', '/finish_trajectory', 'cartographer_ros_msgs/srv/FinishTrajectory', '{trajectory_id: 0}'])
    call('write', ['ros2', 'service', 'call', '/write_state', 'cartographer_ros_msgs/srv/WriteState',
                   json.dumps({'filename': str(dest / 'map.pbstream'), 'include_unfinished_submaps': True})])
except Exception as error:
    report['error'] = str(error)
finally:
    for name, child in reversed(children):
        if child.poll() is not None:
            continue
        for sig, wait in [(signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)]:
            try:
                os.killpg(child.pid, sig)
                child.wait(timeout=wait)
                break
            except ProcessLookupError:
                break
            except subprocess.TimeoutExpired:
                continue
    report['children'] = [{'name': n, 'pid': c.pid, 'exit': c.poll()} for n, c in children]
    for stream in streams:
        stream.close()
    if (dest / 'map.pbstream').exists():
        try:
            call('to_map', [str(binpath / 'cartographer_pbstream_to_ros_map'), '-pbstream_filename', str(dest / 'map.pbstream'),
                            '-map_filestem', str(dest / 'map'), '-resolution', '0.025'], 60)
        except Exception as error:
            report['map_error'] = str(error)
    (dest / 'run_summary.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
print(json.dumps(report, ensure_ascii=False), flush=True)
if 'error' in report:
    raise SystemExit(1)
