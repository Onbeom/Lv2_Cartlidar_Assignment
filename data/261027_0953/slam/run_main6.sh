#!/usr/bin/env bash
# SLAM 비교 본 기록 6번(로봇 261027_1711, 이름 m1711_*. 사용자 승인 2026-10-27: b_dice/seed_1, 밝음 1.0 · 어두움 0.0 × 3회, 전체 경로). 밝음 · 어두움을 번갈아 돈다.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
for i in 1 2 3; do
  for c in bright:1.0 dark:0.0; do
    "$HERE/run_record.sh" "${c#*:}" "m1711_${c%%:*}_$i" 0
  done
done
