#!/usr/bin/env bash
set -euo pipefail
ROOT=/home/pa4/git/lv2_next_module
VERSION=${1:?v버전}
DATASET=${2:-design}
[[ "$VERSION" =~ ^v[0-9]+$ ]] || exit 2
[[ "$DATASET" = design || "$DATASET" = independent ]] || exit 2
OUT=$ROOT/Cartlidar/sim_render/results/a1_new_model_261029/$DATASET
DELAY=${3:-0}
ONLY=${4:-}
[[ "$DELAY" =~ ^[0-9]+([.][0-9]+)?$ ]] || exit 2
[[ "$ONLY" =~ ^[a-zA-Z0-9_]*$ ]] || exit 2
LABEL=replay_$VERSION
ARGS=(--joint-delay-s "$DELAY")
if [[ "$DELAY" != 0 ]]; then LABEL=${LABEL}_delay${DELAY//./p}; fi
if [[ -n "$ONLY" ]]; then LABEL=${LABEL}_$ONLY; ARGS+=(--only "$ONLY"); fi
EMPTY=${5:-0}
[[ "$EMPTY" = 0 || "$EMPTY" = 1 ]] || exit 2
if [[ "$EMPTY" = 1 ]]; then LABEL=${LABEL}_empty_depth; ARGS+=(--front-depth-empty); fi
code=0
REPLAY=$ROOT/261029_a1_new_model/replay.py timeout --signal=TERM --kill-after=20s 1800s bash "$ROOT/261028_1459/run_replay_a1_private.sh" "$ROOT/261029_1030/urdf_261028/waffle_4wd_description_261028/urdf/waffle_4wd.urdf" "$OUT/$LABEL" "$ROOT/261028_1459/a1_work/a1_fix_build/install/setup.bash" --render "$OUT" "${ARGS[@]}" > "$OUT/$LABEL.log" 2>&1 || code=$?
test -s "$OUT/$LABEL/summary.json" || exit "$code"
python3 "$ROOT/261029_a1_new_model/score_perception.py" "$OUT" "$OUT/$LABEL" > "$OUT/${LABEL}_score.log"
