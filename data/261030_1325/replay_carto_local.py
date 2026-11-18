"""고정 VO 입력 Cartographer CPU 재생. 자식 PID/프로세스 그룹만 종료한다."""
import argparse
import json
import os
import signal
import subprocess
import time
from pathlib import Path

p=argparse.ArgumentParser();p.add_argument('light',choices=['bright','dark']);p.add_argument('weight',type=int,choices=[100000,1000,0]);p.add_argument('rotation',type=int,choices=[4,1]);a=p.parse_args()
ROOT=Path('/home/pa4/git/lv2_next_module');HERE=Path(__file__).resolve().parent
OUT=ROOT/'Cartlidar/sim_render/results/followup_261028_1457/improvement_261030_1325'
dest=OUT/f'carto_{a.light}_local1_{a.rotation}';dest.mkdir(exist_ok=False)
binpath=Path('/home/pa4/git/ros_test/waffle-lyrical-slam/install/cartographer_ros/lib/cartographer_ros')
config=(ROOT/'261029_1408/slam/carto_c6_2lidar.lua').read_text()
override=f'\n-- 승인된 진단: graph odom 구속만 비교, local 설정 유지\nPOSE_GRAPH.optimization_problem.odometry_translation_weight = {a.weight}.\nPOSE_GRAPH.optimization_problem.odometry_rotation_weight = {a.weight}.\n'
override+=f'\nTRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 1.\nTRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = {a.rotation}.\n'
config=config.replace('return options',override+'\nreturn options')
(dest/'config.lua').write_text(config)
children=[];streams=[];report={'light':a.light,'graph_odom_weight':a.weight,'local_translation_weight':1,'local_rotation_weight':a.rotation,'simulation_only':True,'old_model':'261027_1711'}
def start(name,cmd):
    stream=(dest/(name+'.log')).open('w');streams.append(stream)
    child=subprocess.Popen(cmd,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
    children.append((name,child));return child
def call(name,cmd,limit=30):
    with (dest/(name+'.log')).open('w') as stream:
        result=subprocess.run(cmd,stdout=stream,stderr=subprocess.STDOUT,timeout=limit)
    report[name+'_exit']=result.returncode
    if result.returncode:raise RuntimeError(name+' 실패')
try:
    carto=start('cartographer',[str(binpath/'cartographer_node'),'-configuration_directory',str(dest),'-configuration_basename','config.lua','--ros-args','-p','use_sim_time:=true','-r','scan_1:=/scan_front','-r','scan_2:=/scan_rear','-r','odom:=/visual_slam/tracking/odometry'])
    start('record',['ros2','bag','record','--use-sim-time','-s','mcap','-o',str(dest/'out_bag'),'--topics','/tf','/clock'])
    time.sleep(6)
    if carto.poll() is not None:raise RuntimeError('Cartographer 기동 실패')
    play=start('play',['ros2','bag','play',str(OUT/f'slam_input_{a.light}'),'--rate','1.0','--disable-keyboard-controls'])
    play.wait(timeout=720);report['play_exit']=play.returncode
    if play.returncode:raise RuntimeError('재생 실패')
    time.sleep(3)
    call('finish',['ros2','service','call','/finish_trajectory','cartographer_ros_msgs/srv/FinishTrajectory','{trajectory_id: 0}'])
    call('write',['ros2','service','call','/write_state','cartographer_ros_msgs/srv/WriteState',json.dumps({'filename':str(dest/'map.pbstream'),'include_unfinished_submaps':True})])
except Exception as error:
    report['error']=str(error)
finally:
    for name,child in reversed(children):
        if child.poll() is not None:continue
        try:
            os.killpg(child.pid,signal.SIGINT);child.wait(timeout=10)
        except (ProcessLookupError,subprocess.TimeoutExpired):
            try:os.killpg(child.pid,signal.SIGTERM);child.wait(timeout=5)
            except (ProcessLookupError,subprocess.TimeoutExpired):
                try:os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
                except (ProcessLookupError,subprocess.TimeoutExpired):pass
    report['children']=[{'name':n,'pid':c.pid,'exit':c.poll()} for n,c in children]
    for stream in streams:stream.close()
    if (dest/'map.pbstream').exists():
        try:call('to_map',[str(binpath/'cartographer_pbstream_to_ros_map'),'-pbstream_filename',str(dest/'map.pbstream'),'-map_filestem',str(dest/'map'),'-resolution','0.025'],60)
        except Exception as error:report['map_error']=str(error)
    (dest/'run_summary.json').write_text(json.dumps(report,ensure_ascii=False,indent=2))
print(json.dumps(report,ensure_ascii=False),flush=True)
if 'error' in report:raise SystemExit(1)
