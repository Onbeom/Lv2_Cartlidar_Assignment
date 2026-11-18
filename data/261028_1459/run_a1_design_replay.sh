#!/usr/bin/env bash
set -euo pipefail
cd /home/pa4/git/lv2_next_module
ROOT=/home/pa4/git/lv2_next_module
OUT=/home/pa4/git/lv2_next_module/Cartlidar/sim_render/results/a1_code_261028
ITERATION=${1:-v2}
[[ "$ITERATION" =~ ^v[0-9]+$ ]] || exit 2
DATASET=${2:-design}
case "$DATASET" in
  design) RENDER=$ROOT/Cartlidar/sim_render/results/render_a1_1711_design_261027 ;;
  inout) RENDER=$OUT/render_mission_inout ;;
  grasp) RENDER=$OUT/render_mission_grasp ;;
  *) exit 2 ;;
esac
FILTER=${3:-}
[[ "$FILTER" =~ ^[a-zA-Z0-9_]*$ ]] || exit 2
ONLY=()
CAL=()
[[ ${CALIBRATED:-0} = 1 ]] && CAL=(--calibrated-d435)
LABEL="replay_${DATASET}_${ITERATION}"
[[ ${CALIBRATED:-0} = 1 ]] && LABEL=${LABEL}_calibrated
[[ ${SEED_BOX:-0} = 1 ]] && LABEL=${LABEL}_box
RUN_TAG=${RUN_TAG:-}
[[ "$RUN_TAG" =~ ^[a-zA-Z0-9_]*$ ]] || exit 2
[ -z "$RUN_TAG" ] || LABEL=${LABEL}_${RUN_TAG}
if [ -n "$FILTER" ]; then ONLY=(--only "$FILTER"); LABEL=${LABEL}_${FILTER}; fi
REPLAY=$ROOT/261028_1459/replay_a1_code.py bash 261028_1459/run_replay_a1_private.sh "$ROOT/261027_1711/urdf/waffle_4wd_description_261027_1711/urdf/waffle_4wd.urdf" "$OUT/$LABEL" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" -- --render "$RENDER" "${ONLY[@]}" "${CAL[@]}" > "$OUT/$LABEL.log" 2>&1
test -f "$OUT/$LABEL/summary.json"
