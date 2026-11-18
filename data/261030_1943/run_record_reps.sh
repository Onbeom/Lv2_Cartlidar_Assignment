#!/usr/bin/env bash
# 녹화 반복 3회(조명 1.0, 경로 v5 전체) + 각 도구 확인. PC, 모의. run_record_lab.sh 를 그대로 부름(도메인 108 · GPU 잠금은 그 안에서).
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for i in 1 2 3; do
  "$HERE/run_record_lab.sh" 1.0 n261028_rep$i 0
  OUT=$(ls -d "$HERE"/results/n261028_rep${i}_* | tail -1)
  ( set +u; source /opt/ros/lyrical/setup.bash && export PYTHONPATH=/usr/lib/python3/dist-packages:${PYTHONPATH:-} && \
    /home/pa4/isaacsim-venv/bin/python "$HERE/check_record_lab.py" "$OUT" ) > "$OUT/check_stdout.txt" 2>&1
  echo "[reps] rep$i 확인 종료 코드 $?: $OUT"
done
echo done
