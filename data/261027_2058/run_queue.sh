#!/usr/bin/env bash
# GPU 차례 큐 (PC, 모의, 2026-10-27 21:2x): 후방 렌더 도구 확인(seed_1, 9 장면) → 재생 확인 → SLAM cuVSLAM 재생 묶음 →
#   후방 판정 렌더(126 장면) → 재생 판정. 렌더는 /tmp/lv2_gpu.lock 을 잡고 돈다(다른 세션과 차례).
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
TOP=$(dirname "$HERE")
SETUP=${REAR_SETUP:?rear_obstacle 를 빌드한 perception install setup.bash}
cd "$TOP"
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python "$HERE/render_rear.py" --worlds Cartlidar/sim_world/worlds/b_dice/seed_1 \
  --n-in 1 --n-out 1 --n-empty 3 --out "$HERE/results/tool_render" --/crashreporter/url= > "$HERE/logs/tool_render.log" 2>&1
bash "$HERE/run_replay_rear.sh" "$HERE/results/tool_render" "$HERE/results/tool_replay" "$SETUP" > "$HERE/logs/tool_replay.log" 2>&1
bash "$TOP/261027_0953/slam/run_replays_1711.sh" > "$HERE/logs/slam_replays.log" 2>&1
flock /tmp/lv2_gpu.lock ~/isaacsim-venv/bin/python "$HERE/render_rear.py" --out "$HERE/results/judge_render" \
  --/crashreporter/url= > "$HERE/logs/judge_render.log" 2>&1
bash "$HERE/run_replay_rear.sh" "$HERE/results/judge_render" "$HERE/results/judge_replay" "$SETUP" > "$HERE/logs/judge_replay.log" 2>&1
echo "[queue] 끝"
