#!/usr/bin/env bash
# S2 · S3 전부 차례로 (PC). 결과: $1/<경우>/. S3 바탕 빌드는 BASE_WS=<HEAD archive ws> 로 줌 (s3_base 에만 씀)
R=${1:-/tmp/sb_s2}
H="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$R"
for c in "0 0,0 b0" "15 0,0 b15" "-15 0,0 bm15" "30 0,0 b30" "-30 0,0 bm30" "45 0,0 b45" "-45 0,0 bm45" "0 0.05,-0.05 s0" "-30 0.05,-0.05 sm30" "15 0.07,0.07 far"; do
  set -- $c
  echo "== $3 rot $1 shift $2"
  BASE_WS= "$H/run_mock.sh" "$R/$3" true --start_rot "$1" --start_shift "$2"
done
echo "== s3_new (enabled false)"; BASE_WS= "$H/run_mock.sh" "$R/s3_new" false
echo "== s3_base (바꾸기 전 mission)"; BASE_WS="$BASE_WS" "$H/run_mock.sh" "$R/s3_base" false
