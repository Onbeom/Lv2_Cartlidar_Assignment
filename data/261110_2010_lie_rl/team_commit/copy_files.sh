#!/usr/bin/env bash
# 2026-11-11 (#192 학습 코드 올리기) 작업 폴더 (fix/ 브랜치, origin/dev) 에 파일을 넣고 등록 패치를 붙임. 기존 파일은 __init__.py · agents/rsl_rl_ppo_cfg.py 에 등록 줄만 더함.
#   bash copy_files.sh <작업 폴더 (저장소 뿌리)>
set -eu
D="$1"; H="$(cd "$(dirname "$0")" && pwd)"
cp -rv "$H/files/workspace" "$D/"
(cd "$D" && patch -p1 < "$H/dev_register.patch")
echo "넣음. git -C $D status 로 확인하세요."
