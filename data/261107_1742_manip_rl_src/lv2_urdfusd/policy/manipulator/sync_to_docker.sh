#!/usr/bin/env bash
# 팀 저장소 workspace/urdf_and_usd 의 isaac/ (로봇 설정 · USD) + policy/manipulator/ (이 폴더) 를 Docker 컨테이너 isaac-lab-base 가 보는 폴더로 복사 (rsync).
#   ./sync_to_docker.sh          ->  ~/dockerWorkspace/IsaacLab/scripts/lv2_urdfusd/{isaac, policy/manipulator}
#                                    (컨테이너: /workspace/isaaclab/scripts/lv2_urdfusd/policy/manipulator)
#   ./sync_to_docker.sh --pull   <-  학습 결과 (logs) 와 수집 데이터 (*.json) 를 되가져옴
# 저장소 구조 (isaac 과 policy 가 형제) 를 그대로 둬야 robot_cfg.py 가 팀 설정을 찾음.
set -eu
MAN="$(cd "$(dirname "$0")" && pwd)"
ROOT="$(cd "$MAN/../.." && pwd)"   # workspace/urdf_and_usd
DST="${ISAACLAB_HOST_DIR:-$HOME/dockerWorkspace/IsaacLab}/scripts/lv2_urdfusd"
if [ "${1:-}" = "--pull" ]; then
  [ -d "$DST/policy/manipulator/logs" ] && rsync -a "$DST/policy/manipulator/logs/" "$MAN/logs/"
  for f in store_handover.json insert_starts.json; do [ -f "$DST/policy/manipulator/$f" ] && cp -f "$DST/policy/manipulator/$f" "$MAN/$f"; done
  echo "가져옴: $DST/policy/manipulator/{logs,*.json} -> $MAN/"
  exit 0
fi
mkdir -p "$DST/policy"
rsync -a --delete --exclude 'rl/logs/' --exclude '*.log' --exclude '__pycache__/' "$ROOT/isaac/" "$DST/isaac/"
rsync -a --delete --exclude 'logs/' --exclude '*.log' --exclude '__pycache__/' --exclude 'store_handover.json' --exclude 'insert_starts.json' "$MAN/" "$DST/policy/manipulator/"
for f in store_handover.json insert_starts.json; do [ -f "$MAN/$f" ] && [ ! -f "$DST/policy/manipulator/$f" ] && cp "$MAN/$f" "$DST/policy/manipulator/$f"; done
echo "복사함: $ROOT/{isaac,policy/manipulator} -> $DST (컨테이너: /workspace/isaaclab/scripts/lv2_urdfusd/policy/manipulator)"
