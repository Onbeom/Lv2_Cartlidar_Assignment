#!/usr/bin/env bash
# 집기 -> 퍽 스토리지 (팔레트) 넣기 여러 번 (2026-11-15, 빼는거 세션). 실물 젯슨, 전체 브링업 (팔 카메라 · 인식) + kbd_joy r 뒤.
# 한 번 = pick_test.py --load auto: 인식 (/perception/pucks) -> J1 에 가장 가까운 퍽 -> target_look -> toppick_tilt (강화학습 정책)
#   -> pallet_load_<열>_<칸>_tilt (노드 궤적 = 실측 칸 맞춤 #293). 칸 = 계획기 규칙 (slots.py can_load: 뒤 열부터, 퍽 모양에 맞는 칸).
#
#   ./load.sh <개수> [옵션]
#     --occupied "3_0 ..."   이미 퍽이 있는 팔레트 칸 (기본 없음). 실물은 점유 인식이 없으니 실제대로 적을 것
#     --slots "3_0 3_1 ..."  칸을 이 차례대로 (모양 · 규칙 검사는 하되 자동 고르기 안 함)
#     --removed "2_3 ..."    쓰지 않을 칸 (기본 PALLET_REMOVED)
#     --pause                한 번 끝날 때마다 Enter 기다림 (상자에 퍽 다시 놓기 · 팔레트 확인)
#     --recheck              넣은 뒤 처음 보던 자리로 가서 옆 퍽 밀림을 잼 (pick_test --recheck)
#     --keep_going           실패해도 다음 퍽 계속 (기본은 멈춤)
#     --dry                  고를 칸 차례만 보여 주고 끝 (모양 = 원기둥 · 직육면체 번갈아 가정)
# 기록: 한 번마다 ~/실측/pick_<시각>/ (arm.mp4 · joints.csv · log.txt) + 요약 ./load_<시각>.log
set -u
HERE="$(cd "$(dirname "$0")" && pwd)"
PICK="${PICK:-$HERE/pick_test.py}"
ROS_WS="${ROS_WS:-$HOME/Lv2_Cartlidar_Assignment/workspace/ros2_ws}"
N="${1:?개수 (예: ./load.sh 2)}"; shift
OCC=""; SLOTS=""; REMOVED="${PALLET_REMOVED-}"; PAUSE=0; RECHECK=""; KEEP=0; DRY=0
while [ $# -gt 0 ]; do case "$1" in
  --occupied) OCC="$2"; shift 2;; --slots) SLOTS="$2"; shift 2;; --removed) REMOVED="$2"; shift 2;;
  --pause) PAUSE=1; shift;; --recheck) RECHECK="--recheck"; shift;; --keep_going) KEEP=1; shift;; --dry) DRY=1; shift;;
  *) echo "모르는 옵션: $1"; exit 2;; esac; done
case "$N" in ''|*[!0-9]*) echo "개수는 숫자"; exit 2;; esac
[ -f "$PICK" ] || { echo "pick_test.py 필요: $PICK"; exit 3; }
if [ "$DRY" = 0 ]; then
  set +u; source "$ROS_WS/install/setup.bash" || { echo "작업 공간 source 실패: $ROS_WS"; exit 3; }; set -u
fi

# --slots: 차례 그대로 규칙 검사 (뒤 열을 앞 열보다 나중에 실으면 못 실음)
if [ -n "$SLOTS" ]; then
  python3 - "$OCC" "$REMOVED" $SLOTS <<'PY' || exit 2
import os, sys
sys.path.insert(0, os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace/ros2_ws/src/mission_planner"))
try:
    from mission_planner.slots import SLOT_SHAPE, _Rules
    can = _Rules("free").can_load
except Exception:
    SLOT_SHAPE = {(r, i): "?" for r in (1, 2) for i in range(4)}; SLOT_SHAPE.update({(3, 0): "?", (3, 1): "?"})
    can = lambda occ, s: not any(r < s[0] for r, _ in occ)
P = lambda t: (int(t.split("_")[0]), int(t.split("_")[1]))
occ, rem = {P(t) for t in sys.argv[1].split()}, {P(t) for t in sys.argv[2].split()}
for t in sys.argv[3:]:
    s = P(t)
    if s not in SLOT_SHAPE: sys.exit(f"칸 {t}: 팔레트 v5 에 없음 (1 · 2 열 0 ~ 3, 3 열 0 · 1)")
    if s in rem: sys.exit(f"칸 {t}: 쓰지 않는 칸 (--removed)")
    if s in occ: sys.exit(f"칸 {t}: 이미 차 있음 (--occupied 또는 --slots 에 두 번)")
    if not can(occ, s): sys.exit(f"칸 {t}: 앞 열에 퍽이 있어 못 실음 (뒤 열부터 실을 것, 계획기 규칙)")
    print(f"  {t} ({SLOT_SHAPE[s]})"); occ.add(s)
PY
  [ "$(echo $SLOTS | wc -w)" -lt "$N" ] && { echo "--slots 가 $N 개보다 적음"; exit 2; }
fi

if [ "$DRY" = 1 ]; then
  echo "[load] dry: 자동 고르기 차례 (모양 번갈아 가정, 실제는 인식한 모양대로)"
  python3 - "$N" "$OCC" "$REMOVED" "$PICK" <<'PY'
import importlib.util, os, sys, types
for m in ["cv2", "rclpy", "rclpy.executors", "rclpy.qos", "robot_interfaces", "robot_interfaces.msg", "sensor_msgs", "sensor_msgs.msg"]:
    sys.modules.setdefault(m, types.ModuleType(m))
sys.modules["rclpy.executors"].SingleThreadedExecutor = object; sys.modules["rclpy.qos"].qos_profile_sensor_data = None
sys.modules["robot_interfaces.msg"].PuckArray = object
sys.modules["sensor_msgs.msg"].Image = sys.modules["sensor_msgs.msg"].JointState = object
os.environ["HOME"] = "/tmp"                       # pick_test 가 기록 폴더를 만들지 않게
home = os.path.expanduser("~" + os.environ.get("USER", ""))
sp = importlib.util.spec_from_file_location("pt", sys.argv[4]); pt = importlib.util.module_from_spec(sp); sp.loader.exec_module(pt)
pt.WS = os.path.join(home, "Lv2_Cartlidar_Assignment/workspace/ros2_ws")
P = lambda t: (int(t.split("_")[0]), int(t.split("_")[1]))
pt.OCC, pt.REMOVED = {P(t) for t in sys.argv[2].split()}, {P(t) for t in sys.argv[3].split()}
for k in range(int(sys.argv[1])):
    s, why = pt.pick_slot(1 + k % 2)
    print(f"  {k + 1}: " + (f"{s[0]}_{s[1]}  ({why})" if s else why))
    if not s: break
    pt.OCC.add(s)
PY
  exit 0
fi

LOG="$HERE/load_$(date +%m%d_%H%M%S).log"; touch "$LOG"
say(){ echo "[load] $*" | tee -a "$LOG"; }
say "퍽 $N 개 집기 -> 넣기. 차 있는 칸: ${OCC:-없음} | 칸: ${SLOTS:-자동 (계획기 규칙 · 퍽 모양)} | 없는 칸: ${REMOVED:-없음}"
say "시작 박스 = 로봇 앞 (집기 정책 학습 거리), 퍽은 팔 카메라에 보이게. 매번 J1 에 가장 가까운 퍽부터 집음"
OK=0; FAIL=0; k=0
for ((n = 1; n <= N; n++)); do
  if [ -n "$SLOTS" ]; then
    s=$(echo $SLOTS | cut -d' ' -f$((k + 1))); LOADARG=(--load "${s%_*}" "${s#*_}")
  else
    LOADARG=(--load auto)
  fi
  say "== $n/$N: ${LOADARG[*]} (차 있음: ${OCC:-없음})"
  out=$(PALLET_REMOVED="$REMOVED" python3 "$PICK" "${LOADARG[@]}" --occupied "$OCC" $RECHECK "load $n/$N" 2>&1 | tee /dev/stderr)
  res=$(echo "$out" | grep -o "RESULT .*" | tail -1); read -r _ st slot <<< "${res:-RESULT error -}"
  dir=$(echo "$out" | grep -o "기록 끝: [^ ]*" | tail -1 | cut -d' ' -f3)
  case "$st" in
    ok) OK=$((OK + 1)); OCC="$OCC $slot"; k=$((k + 1)); say "  성공: 칸 $slot 에 넣음 (기록 $dir)";;
    load_fail)
      FAIL=$((FAIL + 1)); k=$((k + 1))
      say "  !! 넣기 실패 (칸 $slot, 기록 $dir) — 퍽이 칸에 들어갔으면 다음에 --occupied 에 $slot 추가, 팔은 kbd_joy r 로 DRIVE"
      [ "$KEEP" = 0 ] && break;;
    pick_fail) FAIL=$((FAIL + 1)); say "  !! 집기 실패 (기록 $dir) — 칸은 다음 퍽이 그대로 씀"; [ "$KEEP" = 0 ] && break;;
    no_puck) say "  인식된 퍽 없음 -> 끝 (상자에 퍽이 남았는지 · 팔 카메라 화면 확인)"; break;;
    no_slot) say "  실을 칸 없음 -> 끝"; break;;
    *) FAIL=$((FAIL + 1)); say "  !! pick_test 비정상 종료 ($res)"; [ "$KEEP" = 0 ] && break;;
  esac
  [ "$PAUSE" = 1 ] && [ $n -lt "$N" ] && read -r -p "[load] Enter = 다음 퍽, Ctrl+C = 그만 " _
done
say "끝: 성공 $OK / 시도 $((OK + FAIL)) (목표 $N). 지금 차 있는 칸:${OCC:- 없음}. 요약 $LOG"
say "  꺼내서 다시 쓰려면: ./dice.sh real - <눈> --bench --slots \"<꺼낼 칸>\" --occupied \"<남길 칸>\""
[ "$FAIL" = 0 ] && exit 0 || exit 1
