#!/usr/bin/env bash
# waffle_4wd 매니퓰레이터 강화 학습 실행기 (팀 공유본 isaac/ 의 로봇 설정을 씀). Docker 컨테이너 isaac-lab-base 안에서, 또는 Isaac Lab 이 있는 PC 에서.
# 창이 필요한 것 (*_play) 만 창으로, 나머지는 창 없이. 학습 중에는 창을 띄우지 말 것 (GPU 8 GB 부족).
#   ./run_rl.sh sidepick [환경 수] [반복]      1 정렬·집기 학습 -> logs/rsl_rl/waffle_sidepick/
#   ./run_rl.sh sidepick45 [환경 수] [반복] [시작 체크포인트]  세운 퍽 45° 기울여 쥐기 학습 (2026-11-03, 기본 U4 에서 800) -> logs/rsl_rl/waffle_sidepick45/
#   ./run_rl.sh sidepick45_play [환경 수] [체크포인트] / sidepick45_diag [환경 수] [정책 .pt]
#   ./run_rl.sh sidepick45box [환경 수] [반복] [시작 체크포인트]  세운 직육면체만 45° 쥐기 (A 다시 집기용, 2026-11-04, 기본 = 가장 최근 45° 집기 체크포인트에서 500) -> logs/rsl_rl/waffle_sidepick45box/
#   ./run_rl.sh sidepick45box_play [환경 수] / sidepick45box_diag [환경 수] [정책 .pt]   (diag 가 policies/sidepick45box_latest.pt 를 만듦)
#   ./run_rl.sh lieroll45 [환경 수] [반복] [시작 체크포인트]  누운 직육면체 45° 쥐기 학습 (2026-11-04, 기본 = 가장 최근 45° 집기 체크포인트에서 500) -> logs/rsl_rl/waffle_lieroll45/
#   ./run_rl.sh lieroll45_play [환경 수] [체크포인트] / lieroll45_diag [환경 수] [정책 .pt]
#   ./run_rl.sh lieend [환경 수] [반복] [시작 체크포인트]  누운 직육면체 안쪽 끝 수평 쥐기 학습 (2026-11-04, 기본 U4 에서 500) -> logs/rsl_rl/waffle_lieend/
#   ./run_rl.sh lieend_play [환경 수] [체크포인트] / lieend_diag [환경 수] [정책 .pt]
#   ./run_rl.sh sidepick_play [환경 수]        1 정렬·집기 (policies/sidepick_latest.pt) 창으로
#   ./run_rl.sh sidepick_diag [환경 수]        1 실제 과제 평가 (창 없이)
#   ./run_rl.sh liepick [환경 수] [반복]       1' 누운 직사각형 퍽 위에서 가운데 잡아 세우기 학습 -> logs/rsl_rl/waffle_liepick/
#   ./run_rl.sh liepick_play [환경 수] [체크포인트]  위 것을 창으로
#   ./run_rl.sh liepick_hybrid [환경 수]       누운 퍽: 정책으로 쥐고 들기 + 손 IK 로 세우기 평가 (창: liepick_hybrid_play)
#   ./run_rl.sh lie_roll_test [환경 수] [dz]   누운 퍽 새 동작 (낮게 쥐기 -> 수평 -> 롤 −90°) 손 IK 확인 (창: lie_roll_test_play)
#   ./run_rl.sh liepick_eval [환경 수]         위 것 평가 (성공·가운데 벗어남·세움·옆 퍽)
#   ./run_rl.sh store [환경 수] [반복]         2 옮기기·맞추기 학습 -> logs/rsl_rl/waffle_store/
#   ./run_rl.sh store_play [환경 수]           2 옮기기·맞추기 최근 체크포인트를 창으로
#   ./run_rl.sh store_diag [환경 수]           2 진단 (통까지 거리 · 기울기 분해 · 접촉)
#   ./run_rl.sh store_hybrid [환경 수]         2 + 손 IK 넣기 (10 mm 넣고 놓기) 평가
#   ./run_rl.sh store_hybrid_play [환경 수]    위 것을 창으로
#   ./run_rl.sh insert [환경 수] [반복]        3 넣기 학습 -> logs/rsl_rl/waffle_insert/
#   ./run_rl.sh insert_play [환경 수]          3 넣기 최근 체크포인트를 창으로
#   ./run_rl.sh insert_eval [환경 수]          3 넣기 평가 (시작 위치별 성공률 · 종료 이유)
#   ./run_rl.sh chain_play [환경 수] [옵션...]  1 -> 2 -> 3 (넣기 = 손 IK) 이어서 창으로 (policies/*_latest.pt). 셋째부터 chain_play.py 옵션 그대로
#       예) 옆 3 칸 참 + 위에서 들어가기: ./run_rl.sh chain_play 4 --occ 1.0 --overhead 0.20 --ik_lift 0.08 --release_open -0.003 --seconds 35
#   ./run_rl.sh chain_eval [환경 수]           위 것을 창 없이 여러 대, 단계별 성공률
#   ./run_rl.sh chain45_play [환경 수] [옵션...]  (2026-11-03) 45° 집기 SP-45 -> 팀 궤적으로 B안 팔레트 12 칸에 놓기 -> 복귀, 창으로. 옵션은 chain45_play.py 그대로 (--occ 1.0 --slots 3:1 --place floor)
#   ./run_rl.sh chain45_eval [환경 수] [옵션...]  위 것을 창 없이 (48 = 12 칸 x 4), 칸별 성공·접촉
#   ./run_rl.sh chain_play_rl / chain_eval_rl  넣기를 강화 학습 모델로
#   ./run_rl.sh store_check                    통에 퍽 떨어뜨리기 + 팔-몸체 접촉 센서 확인
# Isaac Lab 위치: ISAACLAB_DIR (기본: 컨테이너 /workspace/isaaclab) 또는 ISAACLAB_PY
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
if [ -z "${ISAACLAB_DIR:-}" ]; then
  if [ -d /workspace/isaaclab ]; then ISAACLAB_DIR=/workspace/isaaclab; else ISAACLAB_DIR="$HOME/dockerWorkspace/IsaacLab"; fi
fi
export HEADLESS=1
if [ -n "${ISAACLAB_PY:-}" ]; then PY=("$ISAACLAB_PY")
elif [ -x "$ISAACLAB_DIR/.venv/bin/python" ]; then PY=("$ISAACLAB_DIR/.venv/bin/python")
elif [ -x "$ISAACLAB_DIR/isaaclab.sh" ]; then PY=("$ISAACLAB_DIR/isaaclab.sh" -p)
else echo "Isaac Lab 을 찾지 못했습니다: $ISAACLAB_DIR (ISAACLAB_DIR 또는 ISAACLAB_PY 로 지정)"; exit 1; fi
export_latest() {   # $1 = 과제, $2 = 실험 이름: 가장 최근 학습의 exported/policy.pt 가 없으면 play 를 창 없이 잠깐 돌려 만듦
  local last; last=$(ls -d "$HERE"/logs/rsl_rl/"$2"/*/ 2>/dev/null | sort | tail -1)
  if [ -n "$last" ] && [ ! -f "$last/exported/policy.pt" ]; then
    timeout 300 "${PY[@]}" "$HERE/play.py" --task "$1" --num_envs 1 --checkpoint latest --viz none > "$HERE/export_$2.log" 2>&1 &
    local pid=$!; until [ -f "$last/exported/policy.pt" ] || ! kill -0 $pid 2>/dev/null; do sleep 2; done; sleep 3
    pkill -f "[p]lay.py --task $1 --num_envs 1 "; wait $pid 2>/dev/null   # isaaclab.sh -p 는 자식 python 을 따로 띄우므로 이름으로 끔
  fi
}

pol() { [ -f "$HERE/policies/$1" ] && echo "--checkpoint $HERE/policies/$1" || echo "--checkpoint latest"; }
case "${1:-}" in
  sidepick) "${PY[@]}" "$HERE/train.py" --task Waffle-SidePick-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  sidepick_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-SidePick-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  sidepick45) "${PY[@]}" "$HERE/train.py" --task Waffle-SidePick45-Direct --num_envs "${2:-256}" --max_iterations "${3:-800}" --viz none --checkpoint "${4:-$HERE/checkpoints/sidepick_U4_model_2843.pt}" "${@:5}" ;;
  sidepick45_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-SidePick45-Direct --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  sidepick45_diag) if [ -z "${3:-}" ]; then export_latest Waffle-SidePick45-Direct waffle_sidepick45; L=$(ls -d "$HERE"/logs/rsl_rl/waffle_sidepick45/*/ | sort | tail -1); cp -f "$L/exported/policy.pt" "$HERE/policies/sidepick45_latest.pt"; [ -f "$L/exported/policy.onnx" ] && cp -f "$L/exported/policy.onnx" "$HERE/policies/sidepick45_latest.onnx"; fi; "${PY[@]}" "$HERE/sidepick_diag.py" --pitch45 --num_envs "${2:-64}" --policy "${3:-$HERE/policies/sidepick45_latest.pt}" ;;
  sidepick45box) C="${4:-}"; if [ -z "$C" ]; then L=$(ls -d "$HERE"/logs/rsl_rl/waffle_sidepick45/*/ | sort | tail -1); C=$(ls "$L"model_*.pt | sort -V | tail -1); fi; echo "[run_rl] 시작 체크포인트 $C"; "${PY[@]}" "$HERE/train.py" --task Waffle-SidePick45Box-Direct --num_envs "${2:-256}" --max_iterations "${3:-500}" --viz none --checkpoint "$C" ;;
  sidepick45box_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-SidePick45Box-Direct --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  sidepick45box_diag) if [ -z "${3:-}" ]; then export_latest Waffle-SidePick45Box-Direct waffle_sidepick45box; L=$(ls -d "$HERE"/logs/rsl_rl/waffle_sidepick45box/*/ | sort | tail -1); cp -f "$L/exported/policy.pt" "$HERE/policies/sidepick45box_latest.pt"; [ -f "$L/exported/policy.onnx" ] && cp -f "$L/exported/policy.onnx" "$HERE/policies/sidepick45box_latest.onnx"; fi; "${PY[@]}" "$HERE/sidepick_diag.py" --box45 --num_envs "${2:-64}" --policy "${3:-$HERE/policies/sidepick45box_latest.pt}" ;;
  lieroll45) C="${4:-}"; if [ -z "$C" ]; then L=$(ls -d "$HERE"/logs/rsl_rl/waffle_sidepick45/*/ | sort | tail -1); C=$(ls "$L"model_*.pt | sort -V | tail -1); fi; echo "[run_rl] 시작 체크포인트 $C"; "${PY[@]}" "$HERE/train.py" --task Waffle-LieRoll45-Direct --num_envs "${2:-256}" --max_iterations "${3:-500}" --viz none --checkpoint "$C" ;;
  lieroll45_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-LieRoll45-Direct --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  lieroll45_diag) if [ -z "${3:-}" ]; then export_latest Waffle-LieRoll45-Direct waffle_lieroll45; L=$(ls -d "$HERE"/logs/rsl_rl/waffle_lieroll45/*/ | sort | tail -1); cp -f "$L/exported/policy.pt" "$HERE/policies/lieroll45_latest.pt"; [ -f "$L/exported/policy.onnx" ] && cp -f "$L/exported/policy.onnx" "$HERE/policies/lieroll45_latest.onnx"; fi; "${PY[@]}" "$HERE/sidepick_diag.py" --lieroll --num_envs "${2:-64}" --policy "${3:-$HERE/policies/lieroll45_latest.pt}" ;;
  lieend) "${PY[@]}" "$HERE/train.py" --task Waffle-LieEnd-Direct --num_envs "${2:-256}" --max_iterations "${3:-500}" --viz none --checkpoint "${4:-$HERE/checkpoints/sidepick_U4_model_2843.pt}" ;;
  lieend_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-LieEnd-Direct --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  lieend_diag) if [ -z "${3:-}" ]; then export_latest Waffle-LieEnd-Direct waffle_lieend; L=$(ls -d "$HERE"/logs/rsl_rl/waffle_lieend/*/ | sort | tail -1); cp -f "$L/exported/policy.pt" "$HERE/policies/lieend_latest.pt"; [ -f "$L/exported/policy.onnx" ] && cp -f "$L/exported/policy.onnx" "$HERE/policies/lieend_latest.onnx"; fi; "${PY[@]}" "$HERE/sidepick_diag.py" --lieend --num_envs "${2:-64}" --policy "${3:-$HERE/policies/lieend_latest.pt}" ;;
  sidepick_diag) "${PY[@]}" "$HERE/sidepick_diag.py" --num_envs "${2:-64}" --policy "$HERE/policies/sidepick_latest.pt" ;;
  liepick) "${PY[@]}" "$HERE/train.py" --task Waffle-LiePick-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  liepick_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-LiePick-Direct --num_envs "${2:-16}" --checkpoint "${3:-latest}" --viz kit ;;
  liepick_hybrid) "${PY[@]}" "$HERE/liepick_hybrid.py" --num_envs "${2:-64}" ;;
  liepick_hybrid_play) env -u HEADLESS "${PY[@]}" "$HERE/liepick_hybrid.py" --num_envs "${2:-4}" --gui --viz kit ;;
  lie_roll_test) "${PY[@]}" "$HERE/lie_roll_test.py" --num_envs "${2:-32}" --dz "${3:-0.011}" ;;
  lie_roll_test_play) env -u HEADLESS "${PY[@]}" "$HERE/lie_roll_test.py" --num_envs "${2:-4}" --dz "${3:-0.011}" --gui --viz kit ;;
  liepick_eval) "${PY[@]}" "$HERE/liepick_eval.py" --num_envs "${2:-64}" ;;
  store)   "${PY[@]}" "$HERE/train.py" --task Waffle-Store-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  store_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Store-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  store_diag) export_latest Waffle-Store-Direct waffle_store; "${PY[@]}" "$HERE/store_diag.py" --num_envs "${2:-8}" ;;
  store_hybrid) "${PY[@]}" "$HERE/store_hybrid.py" --num_envs "${2:-16}" ;;
  store_hybrid_play) env -u HEADLESS "${PY[@]}" "$HERE/store_hybrid.py" --num_envs "${2:-8}" --gui --viz kit "${@:3}" ;;
  insert)  "${PY[@]}" "$HERE/train.py" --task Waffle-Insert-Direct --num_envs "${2:-256}" --max_iterations "${3:-300}" --viz none ;;
  insert_play) env -u HEADLESS "${PY[@]}" "$HERE/play.py" --task Waffle-Insert-Direct --num_envs "${2:-16}" --checkpoint latest --viz kit ;;
  insert_eval) export_latest Waffle-Insert-Direct waffle_insert; "${PY[@]}" "$HERE/insert_eval.py" --num_envs "${2:-192}" ;;
  chain_play) env -u HEADLESS "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-4}" --gui --viz kit --insert_ik "${@:3}" ;;
  chain_eval) "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-64}" --insert_ik "${@:3}" ;;
  chain45_play) env -u HEADLESS "${PY[@]}" "$HERE/chain45_play.py" --num_envs "${2:-4}" --gui --viz kit "${@:3}" ;;
  chain45_eval) "${PY[@]}" "$HERE/chain45_play.py" --num_envs "${2:-48}" "${@:3}" ;;
  chain_play_rl) env -u HEADLESS "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-4}" --gui --viz kit ;;
  chain_eval_rl) "${PY[@]}" "$HERE/chain_play.py" --num_envs "${2:-64}" ;;
  store_check) "${PY[@]}" "$HERE/store_check.py" ;;
  *) sed -n 2,20p "$0"; exit 2 ;;
esac
