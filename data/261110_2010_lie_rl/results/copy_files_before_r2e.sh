#!/usr/bin/env bash
# 2026-11-11 (Onbeom, #192): (a) 묶음 파일을 팀 저장소 작업 폴더에 넣음. 인자 = 작업 폴더 (fix/lie-box-regrasp worktree)
set -eu
DST="$1/workspace/urdf_and_usd/policy/manipulator"
SRC=/home/pa4/git/lv2_next_module/261110_2010_lie_rl/ws/urdf_and_usd/policy/manipulator
H=/home/pa4/git/lv2_next_module/261110_2010_lie_rl
[ -d "$DST/waffle_isaac" ] || { echo "작업 폴더가 아님: $DST"; exit 1; }
cp "$SRC/waffle_isaac/lieupright_env.py" "$SRC/waffle_isaac/lieupright_env_cfg.py" "$DST/waffle_isaac/"
cp "$SRC/lieupright_eval.py" "$SRC/lieupright_diag.py" "$SRC/phys_report.py" "$SRC/lieupright_starts.json" "$SRC/lieupright_starts_v2.json" "$DST/"
cp "$H/handoff/lieupright_r2d.onnx" "$H/handoff/lieupright_r2d.pt" "$DST/policies/"
mkdir -p "$DST/기록"
cp "$H/handoff/README.md" "$DST/기록/1c_누운퍽_세우기_LieUpright.md"
(cd "$1" && patch -p1 --forward < "$H/team_commit/dev_register.patch")
git -C "$1" status --short
