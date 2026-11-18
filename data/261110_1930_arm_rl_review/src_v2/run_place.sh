#!/usr/bin/env bash
# 2026-11-07 주사위 눈 놓기 (Waffle-Place60-Direct) 실행 모음. run_rl.sh 와 같은 Isaac Lab 파이썬.
#   ./run_place.sh check [환경 수]                      학습 없이 각본으로 환경 확인 (칸 위 -> 내림 -> 벌림 -> 위로)
#   ./run_place.sh train [환경 수] [반복] [체크포인트]   학습 (기본 2048 x 1500, 처음부터) -> logs/rsl_rl/waffle_place60/
#   ./run_place.sh diag [환경 수] [정책 .pt]             정책 진단 (인자 없으면 가장 최근 학습을 policies/place60_latest.pt 로 내보냄)
#   ./run_place.sh play [환경 수]                        창으로 (학습 중 금지)
#   ./run_place.sh unload [환경 수] [정책] / unload_play   스토리지에서 꺼내기 (각본) -> 놓기 (정책) 연결 (2026-11-09)
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${ISAACLAB_DIR:-}" ]; then
  if [ -d /workspace/isaaclab ]; then ISAACLAB_DIR=/workspace/isaaclab; else ISAACLAB_DIR="$HOME/dockerWorkspace/IsaacLab"; fi
fi
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$ISAACLAB_DIR/.venv/bin/python" ]; then PY=("$ISAACLAB_DIR/.venv/bin/python")
elif [ -x "$ISAACLAB_DIR/isaaclab.sh" ]; then PY=("$ISAACLAB_DIR/isaaclab.sh" -p)
else echo "Isaac Lab 을 찾지 못했습니다: $ISAACLAB_DIR"; exit 1; fi
if [ "${L2:-0}" = 1 ]; then T=Waffle-Place45L2-Direct; EXP=waffle_place45l2; PN=place45l2; DX=--l2; else T=Waffle-Place60-Direct; EXP=waffle_place60; PN=place60; DX=; fi  # L2=1 ./run_place.sh ... = L2 받침 (45°)
case "${1:-}" in
  check) "${PY[@]}" "$HERE/place_diag.py" --script $DX --num_envs "${2:-64}" ;;
  train) export HEADLESS=1; if [ -n "${4:-}" ]; then "${PY[@]}" "$HERE/train.py" --task $T --num_envs "${2:-2048}" --max_iterations "${3:-1500}" --viz none --checkpoint "$4"
         else "${PY[@]}" "$HERE/train.py" --task $T --num_envs "${2:-2048}" --max_iterations "${3:-1500}" --viz none; fi ;;
  diag) if [ -z "${3:-}" ]; then
          L=$(ls -d "$HERE"/logs/rsl_rl/$EXP/*/ | sort | tail -1)
          if [ ! -f "$L/exported/policy.pt" ]; then HEADLESS=1 timeout 300 "${PY[@]}" "$HERE/play.py" --task $T --num_envs 1 --checkpoint latest --viz none > "$HERE/export_place60.log" 2>&1 &
            P=$!; until [ -f "$L/exported/policy.pt" ] || ! kill -0 $P 2>/dev/null; do sleep 2; done; sleep 3; pkill -f "[p]lay.py --task $T --num_envs 1 "; wait $P 2>/dev/null; fi
          cp -f "$L/exported/policy.pt" "$HERE/policies/${PN}_latest.pt"; [ -f "$L/exported/policy.onnx" ] && cp -f "$L/exported/policy.onnx" "$HERE/policies/${PN}_latest.onnx"
        fi
        "${PY[@]}" "$HERE/place_diag.py" $DX --num_envs "${2:-64}" --policy "${3:-$HERE/policies/${PN}_latest.pt}" ;;
  unload) "${PY[@]}" "$HERE/unload_place_play.py" $DX --num_envs "${2:-16}" ${3:+--policy "$3"} "${@:4}" ;;  # 2026-11-09 스토리지에서 꺼내기 (각본) -> 놓기 (정책)
  unload_play) env -u HEADLESS "${PY[@]}" "$HERE/unload_place_play.py" $DX --gui --viz kit --num_envs "${2:-4}" ${3:+--policy "$3"} "${@:4}" ;;
  play) "${PY[@]}" "$HERE/play.py" --task $T --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  *) sed -n 2,7p "$0"; exit 2 ;;
esac
