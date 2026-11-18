#!/bin/bash
# Jetson 부하 2단계(주행 없음): 1단계(카메라 3대 + YOLO D, tools/load_test.py · tegrastats) 위에 위치 추정 묶음(launch/load2.launch.py)을
# 녹화 재생(bag/replay_k0_amcl, 모의 K0 판 351 s, 센서 토픽만)으로 PASSES 판 차례로 띄운다. 판마다 묶음을 새로 띄움(AMCL 출발 자세에서 시작).
#   사용(Jetson): bash run_load2.sh [PASSES=5]      환경 값(선택): ENGINE=<D 엔진>  NO_CAMS=1(카메라 · YOLO 없이 ROS 만)  ROS_DOMAIN_ID(기본 77)
#   결과: results/<시각>/ — cam/(load_test 결과: frames.csv · tegrastats.log · summary.json), proc.csv, passes.csv, pass_N/(launch.log · loc_events.jsonl · out_bag)
set -o pipefail
HERE=$(cd "$(dirname "$0")" && pwd)
PASSES=${1:-5}
OUT=$HERE/results/$(date +%Y%m%d_%H%M%S); mkdir -p $OUT
PASS_S=375                                   # 한 판 = 띄움 6 s + 재생 351 s + 정리 약 15 s
SECS=$((PASSES * PASS_S + 30))
set +u
source /opt/ros/lyrical/setup.bash
if [ ! -f $HERE/ws/install/setup.bash ]; then
  mkdir -p $HERE/ws/src && ln -sfn $HERE/src/localization $HERE/ws/src/localization
  (cd $HERE/ws && colcon build --symlink-install > $OUT/colcon.log 2>&1) || { echo "빌드 실패: $OUT/colcon.log"; exit 1; }
fi
source $HERE/ws/install/setup.bash
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-77} ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST ROS_LOG_DIR=$OUT/ros_logs
mark() { python3 -c "import time; print(f'{time.time():.3f},{time.monotonic():.3f}')"; }
echo "pass,event,wall_s,mono_s" > $OUT/passes.csv
echo "passes $PASSES secs $SECS no_cams ${NO_CAMS:-0} domain $ROS_DOMAIN_ID" > $OUT/info.txt
sha256sum $0 $HERE/launch/*.py $HERE/config/* $HERE/tools/*.py $HERE/src/localization/localization/*.py $HERE/src/localization/config/* $HERE/bag/replay_k0_amcl/*.mcap >> $OUT/info.txt
if [ "${NO_CAMS:-0}" = 1 ]; then
  command -v tegrastats >/dev/null && { tegrastats --interval 1000 > $OUT/tegrastats.log 2>&1 & TG=$!; }
else
  ENGINE=${ENGINE:-$HOME/yolo_D/deploy/yolo11n_task4_D_fp32_notf32.engine}   # 1단계 · 11-04 판과 같은 엔진(폴더 첫 파일로 고르면 새 엔진이 끼어듦)
  [ -f "$ENGINE" ] || { echo "D 엔진 없음 — ENGINE=<경로> 로 주세요"; exit 1; }
  echo "engine $ENGINE" >> $OUT/info.txt
  python3 $HERE/tools/load_test.py --engine $ENGINE --seconds $SECS --out $OUT/cam > $OUT/load_test.log 2>&1 & LT=$!
  sleep 8                                    # 카메라 켜짐 · 자동 노출(load_test 안에서 3 s)
  kill -0 $LT 2>/dev/null || { echo "load_test 가 시작하자마자 끝남 — $OUT/load_test.log"; tail -3 $OUT/load_test.log; exit 1; }
fi
python3 $HERE/tools/proc_sample.py --out $OUT/proc.csv --seconds $((SECS + 10)) & PS=$!
for n in $(seq $PASSES); do
  P=$OUT/pass_$n; mkdir -p $P
  echo "$n,start,$(mark)" >> $OUT/passes.csv
  setsid ros2 launch $HERE/launch/load2.launch.py loc_event_log:=$P/loc_events.jsonl > $P/launch.log 2>&1 & LP=$!
  sleep 6
  setsid ros2 bag record --use-sim-time -s mcap -o $P/out_bag --topics /amcl_pose /localization/status /scan_merged /odometry/filtered /tf > $P/record.log 2>&1 & RP=$!
  sleep 1
  echo "$n,play,$(mark)" >> $OUT/passes.csv
  ros2 bag play $HERE/bag/replay_k0_amcl --clock 100 > $P/play.log 2>&1
  echo "$n,play_end,$(mark)" >> $OUT/passes.csv
  sleep 2
  kill -TERM -$RP 2>/dev/null                # 배경 작업은 SIGINT 를 무시함 → TERM, bag 이 닫힐 때까지 기다림
  for i in $(seq 60); do kill -0 $RP 2>/dev/null || break; sleep 0.5; done
  kill -INT -$LP 2>/dev/null; sleep 5
  kill -KILL -$RP -$LP 2>/dev/null
  echo "$n,end,$(mark)" >> $OUT/passes.csv
  [ -n "$LT" ] && ! kill -0 $LT 2>/dev/null && { echo "load_test 가 먼저 끝남(판 $n 뒤)" >> $OUT/info.txt; break; }
done
[ -n "$LT" ] && wait $LT
kill $PS ${TG:-} 2>/dev/null
echo "끝 $OUT" | tee $OUT/done.txt
