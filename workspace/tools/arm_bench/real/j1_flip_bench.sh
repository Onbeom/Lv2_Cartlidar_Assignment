#!/bin/bash
# 관절 1 π 보정 펌웨어 (팔 fw v6, 2026-11-16) 업로드 · 실물 확인. 노트북에서, 팔 OpenCR 을 USB 로 (브링업 · meas_session 은 끔).
#   bash j1_flip_bench.sh            1 업로드 → 2 읽기 → 3 관절 1 조금 → 4 DRIVE 펴기 · 정면 지나기 (meas_session)
#   bash j1_flip_bench.sh --from 2   업로드는 건너뛰고 2 부터
# 단계마다 Enter 로 넘어가고, 기대와 다르면 Ctrl-C 로 멈춘 뒤 결과를 알려 주세요. 비상: Ctrl-C (그 자리 HOLD).
set -e
WS=$(cd "$(dirname "$0")/../../.." && pwd)            # workspace/
PORT=${PORT:-/dev/ttyACM0}
TOOL=$WS/ros2_ws/src/opencr_hardware/scripts/opencr_arm_tool.py
FROM=1; [ "$1" = "--from" ] && FROM=$2
step() { echo; echo "== $1"; }
pause() { read -r -p "   ${1:-Enter = 다음, Ctrl-C = 그만} "; }
if pgrep -f "[m]eas_session|[o]pencr_arm_tool|[r]os2_control_node" >/dev/null; then
  echo "!! 팔 보드를 쓰는 프로그램이 떠 있음 (한 포트에 둘이면 STATE 를 나눠 읽음) — 먼저 끌 것:"; pgrep -fa "[m]eas_session|[o]pencr_arm_tool|[r]os2_control_node"; exit 1
fi

if [ "$FROM" -le 1 ]; then
  step "1. 팔 펌웨어 v6 빌드 · 업로드 (12 V 켬, 팔은 손으로 STOW, 토크 꺼짐)"
  echo "   지금 보드 (role 2 = 팔 이어야 함, fw 5 = 보정 전):"
  python3 "$TOOL" monitor --port "$PORT" --seconds 1 2>&1 | tail -2 || true
  pause "Enter = 빌드 · 업로드 (마지막에 yes 를 한 번 더 물음)"
  bash "$WS/ros2_ws/src/opencr_firmware/tools/build_firmware.sh" arm --upload "$PORT"
  echo "   업로드 끝. 12 V 켠 채 OpenCR 리셋 버튼 (부팅 점검이 관절 1 을 확장 위치 모드로 바꿈)"
  pause
fi

if [ "$FROM" -le 2 ]; then
  step "2. 읽기만 (쓰기 없음): fw=6, 오류는 CMD_TIMEOUT 만, 관절 1 이 STOW 자리에서 약 +126° (범위 +120.2 ~ +130.2°)"
  python3 "$TOOL" monitor --port "$PORT" --seconds 2 | tail -2
  echo "   기대: arm[deg] ≈ 126  -120  89  -107  0  (집게 아무 값, −45.5 보다 커야 켜짐)"
  echo "   −55 근처면 업로드 · 리셋이 안 된 것. 0 근처나 ±180 근처면 멈추고 알려 주세요."
  echo "   (토크 꺼짐) 손으로 관절 1 을 ±5° 쯤 천천히 돌려 보며 값이 끊기지 않고 따라오는지 — 아래 3 초 동안"
  pause "Enter = 3 초 읽기"
  python3 "$TOOL" monitor --port "$PORT" --seconds 3 | tail -6
  echo "   끝나면 관절 1 을 다시 STOW 자리 (약 126°) 로"
  pause
  python3 "$WS/tools/arm_bench/real/arm_stow.py" --check || true
  pause "대기 자세 6 칸 모두 범위 안이면 Enter (밖이면 손으로 맞추고 다시 --from 2)"
fi

if [ "$FROM" -le 3 ]; then
  step "3. 관절 1 만 조금 (토크 ON → +0.05 rad 갔다 제자리, 0.1 rad/s). 팔 주변 비우고 지켜보기"
  python3 "$TOOL" move --port "$PORT" --joint 1 --delta 0.05 --check-standby --dry-run | tail -4
  pause "계획이 '관절 1 +0.050 rad' 면 Enter = 실제로 움직임"
  python3 "$TOOL" move --port "$PORT" --joint 1 --delta 0.05 --check-standby --yes
  echo "   기대: 위에서 봐서 반시계로 약 3° 갔다 돌아옴, 오류 없음. 끝나면 펌웨어가 그 자리 HOLD (토크 켜진 채)"
  pause
fi

if [ "$FROM" -le 4 ]; then
  step "4. DRIVE 펴기 · 정면 지나기 (meas_session 직접 모드)"
  echo "   메뉴 9 → 1 : STOW → DRIVE. ★ 마지막 단계 (관절 1 → 0) 에서 집게가 로봇 **정면** 으로 와야 함 (스토리지 쪽이면 Ctrl-C)"
  echo "   메뉴 8 : '10 -78.5 -0.6 106 0' → '-10 -78.5 -0.6 106 0' → '0 -78.5 -0.6 106 0'  (정면을 ±10° 로 지나기, 한 바퀴 돌면 Ctrl-C)"
  echo "   메뉴 7 : 지금 관절각 (관절 1 이 보낸 값과 같은지)"
  echo "   메뉴 9 → 2 : STOW 로 접고 0 끝"
  pause "Enter = meas_session 시작"
  (cd "$WS/tools/arm_bench/real" && python3 meas_session.py --direct "$PORT")
fi

step "끝 — 결과 (각 단계 기대대로였는지, 다르면 그 출력) 를 알려 주세요. 다음: 젯슨 pull · 빌드 → base_bringup 대기 검사 → 전체 브링업 켜질 때 정리"
