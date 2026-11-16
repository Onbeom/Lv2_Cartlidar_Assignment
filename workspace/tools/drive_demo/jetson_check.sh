#!/usr/bin/env bash
# Jetson 준비 점검 — 읽기만 (sudo 없음 · 설치 · 설정 변경 · 포트 쓰기 없음). 2026-11-11
#
#   cd ~/demo_v2 && ./jetson_check.sh | tee ~/jetson_check_$(date +%H%M).txt
#   ./jetson_check.sh opencr        # OpenCR 만 (4 번: 장치 · 권한 · 두 보드 STATE · 팔 대기 자세)
#   ./jetson_check.sh system | sensors   # 1~3 · 5~7 (시스템 · 권한 · ROS · 저장소) | 8 (카메라 · 라이다 · TFmini USB) — 11-14 시연용 사본
#
# ✅ 됨 · ⚠ 확인 필요 (그대로도 돌 수 있음) · ❌ 고쳐야 함 — ❌ / ⚠ 줄 아래 "→" 가 고치는 명령 (직접 실행, 대부분 sudo)
# OpenCR 은 열어서 STATE 만 1 초 읽는다 (opencr_arm_tool.Link = serial.read 만). 다른 프로그램이 포트를 잡고 있으면 건너뜀.
cd "$(dirname "$0")" || exit 1
OK=0; WARN=0; BAD=0
ok()   { echo "  ✅ $*"; OK=$((OK+1)); }
warn() { echo "  ⚠  $*"; WARN=$((WARN+1)); }
bad()  { echo "  ❌ $*"; BAD=$((BAD+1)); }
fix()  { echo "     → $*"; }
sec()  { echo; echo "== $*"; }
ONLY=${1:-}
SRC=$(python3 lv2_paths.py 2>/dev/null)

want() { [ -z "$ONLY" ] || [ "$ONLY" = "$1" ]; }   # 11-14: 모드 system (1~3 · 5~7) · opencr (4) · sensors (8) · 없음 = 전부
if want system; then

# ---------------------------------------------------------------- 1 시스템
sec "1 시스템"
echo "  $(hostname) · $(uname -m) · $(. /etc/os-release && echo "$PRETTY_NAME") · 커널 $(uname -r)"
[ "$(uname -m)" = aarch64 ] && ok "aarch64 (Jetson)" || warn "aarch64 아님 — 노트북에서 돌리는 중? (노트북이면 아래 ❌ 일부는 상관없음)"
if [ -r /etc/nv_tegra_release ]; then echo "  L4T: $(head -1 /etc/nv_tegra_release)"; fi
v=$(dpkg-query -W -f='${Version}' nvidia-jetpack 2>/dev/null) && echo "  JetPack: $v"
command -v nvpmodel >/dev/null && echo "  전원 모드: $(nvpmodel -q 2>/dev/null | head -1)"
free -h | awk '/^Mem/ {print "  메모리: " $2 " (사용 " $3 ")"}'
df -h "$HOME" | awk 'NR==2 {print "  디스크 (홈): 남음 " $4 " / " $2}'
avail=$(df -Pk "$HOME" | awk 'NR==2 {print $4}')
[ "${avail:-0}" -gt 10485760 ] && ok "디스크 남음 10 GB 이상" || warn "디스크 남음 10 GB 미만 — colcon build · 로그로 모자랄 수 있음"

# ---------------------------------------------------------------- 2 사용자 · 시리얼 권한
sec "2 사용자 · 시리얼 권한"
if id -nG | tr ' ' '\n' | grep -qx dialout; then ok "$(id -un) 이 dialout 그룹"
elif getent group dialout | grep -qw "$(id -un)"; then warn "dialout 에 넣었지만 아직 로그인에 반영 안 됨"; fix "로그아웃 후 다시 로그인 (또는 재부팅)"
else bad "$(id -un) 이 dialout 그룹 아님 — /dev/ttyACM* 를 못 엶"; fix "sudo usermod -aG dialout $(id -un)  # 그 뒤 다시 로그인"; fi

if systemctl list-unit-files ModemManager.service >/dev/null 2>&1 && systemctl cat ModemManager.service >/dev/null 2>&1; then
  st=$(systemctl is-active ModemManager 2>/dev/null); en=$(systemctl is-enabled ModemManager 2>/dev/null)
  if [ "$st" = active ]; then
    warn "ModemManager 동작 중 ($en) — udev 규칙 ID_MM_DEVICE_IGNORE 가 있으면 괜찮음 (아래 3 에서 확인)"
    fix "모뎀을 안 쓰면 끄는 것이 가장 확실: sudo systemctl disable --now ModemManager"
  else ok "ModemManager 꺼짐 ($st · $en)"; fi
else ok "ModemManager 없음"; fi

# ---------------------------------------------------------------- 3 udev 규칙
sec "3 udev 규칙"
for pair in "opencr_hardware/udev/99-opencr.rules:OpenCR (권한 · ModemManager 무시)" "perception/udev/99-sensors.rules:센서 (라이다 · TFmini · 팔 카메라 — 시리얼 자리표시 ★)"; do
  rel=${pair%%:*}; what=${pair#*:}; name=$(basename "$rel")
  if [ -f "/etc/udev/rules.d/$name" ]; then
    if [ -n "$SRC" ] && [ -f "$SRC/$rel" ] && ! cmp -s "/etc/udev/rules.d/$name" "$SRC/$rel"; then
      warn "$name 설치됨 · 저장소 것과 다름 — $what"; fix "diff /etc/udev/rules.d/$name $SRC/$rel"
    else ok "$name 설치됨 — $what"; fi
  else
    if [ "$name" = 99-opencr.rules ]; then bad "$name 없음 — $what"
    else warn "$name 없음 — $what (센서 조립 뒤에, 시리얼 채운 다음 설치)"; fi
    [ -n "$SRC" ] && fix "sudo cp $SRC/$rel /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger"
  fi
done

fi   # 1 ~ 3 끝

# ---------------------------------------------------------------- 4 OpenCR 장치
if want opencr; then
sec "4 OpenCR 장치 (/dev/ttyACM*)"
shopt -s nullglob; acms=(/dev/ttyACM*); shopt -u nullglob
if [ ${#acms[@]} -eq 0 ]; then bad "/dev/ttyACM* 없음 — OpenCR USB 를 Jetson 에 꽂았는지"; fi
for d in "${acms[@]}"; do
  eval "$(udevadm info -q property -n "$d" 2>/dev/null | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_PATH|ID_MM_DEVICE_IGNORE)=' | sed 's/^/local_/')"
  perm=$(stat -c '%G %a' "$d")
  echo "  $d  vid:pid ${local_ID_VENDOR_ID:-?}:${local_ID_MODEL_ID:-?} · 그룹/권한 $perm · USB 자리 ${local_ID_PATH:-?}"
  [ "${local_ID_VENDOR_ID}:${local_ID_MODEL_ID}" = 0483:5740 ] || warn "$d 는 OpenCR (0483:5740) 아님"
  [ "${local_ID_MM_DEVICE_IGNORE:-0}" = 1 ] && ok "$d ModemManager 무시 표시 있음" || warn "$d ModemManager 무시 표시 없음 (99-opencr.rules 미설치?) — 처음 몇 초 ModemManager 가 포트를 건드릴 수 있음"
  [ -r "$d" ] && [ -w "$d" ] && ok "$d 읽기 · 쓰기 권한 있음" || bad "$d 권한 없음 (dialout · udev 규칙 확인)"
  unset local_ID_VENDOR_ID local_ID_MODEL_ID local_ID_PATH local_ID_MM_DEVICE_IGNORE
done
if [ -d /dev/serial/by-path ]; then
  echo "  by-path (꽂은 USB 자리 — 두 보드 by-id 는 이름이 같아 이것으로 구분):"
  for l in /dev/serial/by-path/*; do case $l in *-usbv2-*) continue;; esac; echo "    $(basename "$l") → $(readlink -f "$l")"; done
fi

if [ ${#acms[@]} -gt 0 ]; then
  if python3 -c 'import serial' 2>/dev/null; then
    echo "  STATE 1 초 읽기 (쓰기 없음):"
    python3 - "${acms[@]}" <<'EOF'
import math, re, sys, time
from lv2_paths import SCRIPTS
sys.path.insert(0, str(SCRIPTS))
from opencr_arm_tool import Link, err_text, ERR_ABORT
ROLE = {1: '주행', 2: '팔'}
NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', '그리퍼']
roles = []
W = 0   # ⚠ · ❌ 수 → 종료 코드 (bash 요약에 더함)


def expected_fw(role):   # 11-16: 저장소 헤더가 기대하는 펌웨어 버전 (주행 = PROTOCOL_VERSION, 팔 = ARM_FW_VERSION 이 있으면 그것, #316)
    try:
        t = (SCRIPTS.parent / 'include' / 'opencr_hardware' / 'opencr_protocol.hpp').read_text()
        pv = int(re.search(r'PROTOCOL_VERSION = (\d+)', t).group(1))
        m = re.search(r'ARM_FW_VERSION = (\d+)', t)
        return int(m.group(1)) if (role == 2 and m) else pv
    except Exception:
        return None


def standby():   # 호스트 on_activate 가 쓰는 대기 자세 범위 (xacro arm_standby_min/max_tick)
    try:
        t = (SCRIPTS.parent / 'urdf' / 'opencr.ros2_control.xacro').read_text()
        return [[int(x) for x in re.search(k + r'">([^<]+)<', t).group(1).split(',')]
                for k in ('arm_standby_min_tick', 'arm_standby_max_tick')]
    except Exception:
        return None


def arm_report(st):
    global W
    rng, out, bad = standby(), [], 0
    for k in range(6):
        t = round(2048 + st['pos'][6 + k] / (2 * math.pi / 4096))
        if rng and not rng[0][k] <= t <= rng[1][k]:
            bad += 1
            out.append(f'{NAMES[k]} {t}❗({rng[0][k]}~{rng[1][k]})')
        else:
            out.append(f'{NAMES[k]} {t}')
    print('       팔 tick: ' + ' · '.join(out))
    if st['error'] & ERR_ABORT:
        W += 1
        print('       ⚠  팔 오류 비트 있음 → 위 tick 은 믿을 수 없음. 팔 모터를 보드가 켜진 채 연결했다면 팔 보드 RESET 버튼 (토크 안 켜짐 · 안 움직임)'
              ' 으로 다시 부팅해 6 개 모두로 부팅 점검을 다시 받은 뒤 이 스크립트를 다시')
    elif rng:
        W += bad > 0
        print('       ' + ('✅ 여섯 칸 모두 대기 자세 범위 안 (호스트 활성화 조건 통과)' if bad == 0 else
                         f'⚠  {bad} 칸이 대기 자세 범위 밖 (❗) — 호스트 on_activate 가 거부함 (토크 끈 채 손으로 맞추거나 범위 확인)'))
for p in sys.argv[1:]:
    try:
        ln = Link(p)
    except Exception as e:
        print(f'    ⚠  {p}: 못 엶 ({e}) — 다른 프로그램이 쓰는 중?'); W += 1; continue
    t0 = time.monotonic(); n0 = None
    while time.monotonic() - t0 < 1.3:            # 앞 0.3 s 는 버퍼에 남은 예전 STATE 버림
        ln.poll()
        if n0 is None and time.monotonic() - t0 > 0.3:
            n0 = ln.count
        time.sleep(0.005)
    st, n = ln.last, ln.count - (n0 or 0)
    if st is None or n == 0:
        print(f'    ❌ {p}: STATE 없음 (펌웨어 · 전원 확인)'); W += 1; continue
    roles.append(st['role'])
    print(f"    ✅ {p}: {ROLE.get(st['role'], '?')} 보드 · fw v{st['fw']} · {n} 프레임/s · 배터리 {st['battery']:.2f} V · err {err_text(st['error'])}")
    ef = expected_fw(st['role'])
    if ef is not None and st['fw'] != ef:
        print(f"    ❌ {ROLE.get(st['role'], '?')} 보드 펌웨어 v{st['fw']} ≠ 저장소 기대 v{ef} — 호스트가 활성화를 거부함"
              + (" (팔 v6 = 관절 1 π 보정 #316, 팔 담당이 opencr_arm_v6.bin 업로드)" if st['role'] == 2 else "")); W += 1
    if st['role'] == 2:
        arm_report(st)
if sorted(roles) == [1, 2]:
    print('    ✅ 주행 · 팔 두 보드 모두 보임 — 포트 번호는 상관없음 (호스트가 STATE.role 로 고름)')
elif roles:
    W += 1
    print(f"    ⚠  보이는 보드: {', '.join(ROLE.get(r, '?') for r in roles)} — 두 보드 ros2_control 은 둘 다 필요")
sys.exit(min(W, 50))
EOF
    WARN=$((WARN + $?))
  else bad "pyserial 없음 — 바탕화면 도구 · opencr_*_tool.py 를 못 씀"; fix "sudo apt install python3-serial"; fi
fi
fi   # 4 끝

if want system; then
# ---------------------------------------------------------------- 5 ROS 2
sec "5 ROS 2"
distros=$(ls /opt/ros 2>/dev/null | tr '\n' ' ')
if [ -z "$distros" ]; then bad "/opt/ros 없음 — ROS 2 미설치"; DISTRO=
else
  DISTRO=${ROS_DISTRO:-$(ls /opt/ros | head -1)}
  [ "$DISTRO" = lyrical ] && ok "ROS 2 $DISTRO (팀과 같음)" || warn "ROS 2 $DISTRO — 팀은 lyrical"
  [ -n "${ROS_DISTRO:-}" ] || warn "지금 셸에 ROS 환경이 안 잡힘"; [ -n "${ROS_DISTRO:-}" ] || fix "source /opt/ros/$DISTRO/setup.bash  (~/.bashrc 에 넣을지는 선택)"
  echo "  ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-(없음 = 0)} · RMW=${RMW_IMPLEMENTATION:-(기본)} · ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-(없음)}"
fi
command -v colcon >/dev/null && ok "colcon 있음" || { bad "colcon 없음"; fix "sudo apt install python3-colcon-common-extensions"; }
command -v rosdep >/dev/null && ok "rosdep 있음" || warn "rosdep 없음 (빠진 패키지 찾기에 편함: sudo apt install python3-rosdep)"

missing_all=()
check_pkgs() {   # $1 = 묶음 이름, 나머지 = ROS 패키지
  local title=$1; shift; local miss=()
  for p in "$@"; do
    [ -n "$DISTRO" ] && [ -f "/opt/ros/$DISTRO/share/$p/package.xml" ] && continue
    [ -n "$WS_INSTALL" ] && [ -f "$WS_INSTALL/$p/share/$p/package.xml" ] && continue
    miss+=("$p")
  done
  if [ ${#miss[@]} -eq 0 ]; then ok "$title — $# 개 모두 있음"
  else bad "$title — 없음 ${#miss[@]} / $#: ${miss[*]}"; missing_all+=("${miss[@]}"); fi
}
WS_INSTALL=; [ -n "$SRC" ] && [ -d "$SRC/../install" ] && WS_INSTALL=$(cd "$SRC/../install" && pwd)
# 바퀴 · 팔 구동만 (opencr_hardware base_bringup) — package.xml 의존 그대로
check_pkgs "주행 · 팔 구동 (opencr_hardware)" hardware_interface controller_manager diff_drive_controller joint_state_broadcaster \
  joint_trajectory_controller imu_sensor_broadcaster parallel_gripper_controller robot_state_publisher xacro twist_mux \
  joy teleop_twist_joy joy_teleop topic_based_ros2_control diagnostic_msgs
check_pkgs "위치 추정 · EKF (bringup · localization)" robot_localization nav2_amcl nav2_map_server nav2_lifecycle_manager cartographer_ros
check_pkgs "Nav2 (navigation)" nav2_bt_navigator nav2_controller nav2_planner nav2_smac_planner nav2_mppi_controller \
  nav2_behaviors nav2_collision_monitor nav2_velocity_smoother nav2_costmap_2d nav2_msgs
check_pkgs "팔 · 미션 (control · mission)" control_msgs tf2_geometry_msgs yaml_cpp_vendor
check_pkgs "센서 드라이버 (조립 뒤)" realsense2_camera coin_d4_driver cv_bridge
for lib in libzbar-dev liblapack-dev libopencv-dev; do
  dpkg -s "$lib" >/dev/null 2>&1 || { bad "$lib 없음 (perception 빌드)"; missing_all+=("$lib"); }
done
if [ ${#missing_all[@]} -gt 0 ] && [ -n "$DISTRO" ]; then
  apt_names=()
  for p in "${missing_all[@]}"; do case $p in lib*) apt_names+=("$p");; coin_d4_driver) ;; *) apt_names+=("ros-$DISTRO-${p//_/-}");; esac; done
  echo "     → 설치 후보 (승인 · 직접 실행, apt 에 없는 것은 소스 빌드):"
  echo "       sudo apt install ${apt_names[*]}"
  printf '%s\n' "${missing_all[@]}" | grep -qx coin_d4_driver && echo "       coin_d4_driver 는 apt 에 없음 — 라이다 조립 때 소스로"
fi

# ---------------------------------------------------------------- 6 저장소 · 빌드
sec "6 저장소 · 빌드"
if [ -z "$SRC" ]; then bad "저장소 못 찾음"; fix "git clone 후 LV2_SRC=<저장소>/workspace/ros2_ws/src 로 다시 실행"
else
  REPO=$(cd "$SRC/../../.." && pwd); echo "  저장소: $REPO"
  echo "  HEAD: $(git -C "$REPO" log -1 --format='%h %ad %s' --date=format:'%m-%d %H:%M' 2>/dev/null)"
  echo "  브랜치: $(git -C "$REPO" rev-parse --abbrev-ref HEAD 2>/dev/null) · 바뀐 파일 $(git -C "$REPO" status --porcelain 2>/dev/null | wc -l) 개"
  grep -q 'WHEEL_VEL_LIMIT_RAW = 313' "$SRC/opencr_firmware/config.h" && ok "config.h 바퀴 한계 313 (27cef56 이후)" \
    || warn "config.h 바퀴 한계가 313 아님 — 27cef56 (PR #207) 전 코드"
  [ -f "$SRC/opencr_firmware/battery_alarm.h" ] && ok "battery_alarm.h 있음 (배터리 기능 포함)" || warn "battery_alarm.h 없음 — dev 29d4c22 로 받을 것"
  WS=$(cd "$SRC/.." && pwd)
  cache=$WS/build/opencr_hardware/CMakeCache.txt
  if [ -f "$cache" ]; then
    home=$(grep '^CMAKE_HOME_DIRECTORY' "$cache" | cut -d= -f2)
    [ "$home" = "$SRC/opencr_hardware" ] && ok "build/ 캐시 경로 맞음" \
      || { bad "build/ 캐시가 다른 경로 것 ($home) — 그대로 빌드하면 실패"; fix "cd $WS && rm -rf build install log  # 그 뒤 colcon build"; }
  else warn "아직 빌드 안 함 ($WS/build 없음)"; fi
  [ -f "$WS/install/opencr_hardware/share/opencr_hardware/package.xml" ] && ok "opencr_hardware 설치됨 ($WS/install)" || warn "opencr_hardware 아직 설치 안 됨"
fi

# ---------------------------------------------------------------- 7 그 밖
sec "7 그 밖"
command -v arduino-cli >/dev/null && echo "  arduino-cli 있음" || echo "  arduino-cli 없음 — 펌웨어 업로드는 노트북에서 (OpenCR 업로드 도구가 aarch64 용으로 없음)"
timedatectl show -p NTPSynchronized --value 2>/dev/null | grep -q yes && ok "시계 동기화 (NTP) 됨" \
  || warn "시계 동기화 안 됨 — 노트북과 ROS 를 같이 쓰면 TF 시간 어긋남 (rviz 등)"
ip -4 -o addr show scope global 2>/dev/null | awk '{print "  IP: " $2 " " $4}'

# ---------------------------------------------------------------- 8 센서 · USB
fi   # 5 ~ 7 끝
if want sensors; then
sec "8 센서 · USB (읽기만 — 카메라 영상은 열지 않음)"
echo "  USB 장치 (루트 허브 빼고) — 480 = USB2 · 5000 이상 = USB3:"
for d in /sys/bus/usb/devices/*; do
  [ -f "$d/idVendor" ] || continue
  n=$(basename "$d"); case $n in usb*) continue;; esac
  vp="$(cat "$d/idVendor"):$(cat "$d/idProduct")"; prod=$(cat "$d/product" 2>/dev/null); spd=$(cat "$d/speed" 2>/dev/null)
  ser=$(cat "$d/serial" 2>/dev/null)
  if [ "$(cat "$d/bDeviceClass" 2>/dev/null)" = 09 ]; then
    a=$(cat "$d/bmAttributes" 2>/dev/null); [ $(( 0x${a:-80} & 0x40 )) -ne 0 ] && pw="자체 전원 (설명자 기준)" || pw="버스 전원 — 카메라 2 대엔 전원 모자랄 수 있음"
    echo "    [허브] $n $vp ${prod} · $spd Mb/s · $pw"
  else echo "    $n $vp ${prod:-?} · $spd Mb/s${ser:+ · USB 시리얼 $ser}"; fi
done

# D435 두 대 — USB3 · 시리얼 (launch 기본값과 비교)
rs_n=0
for d in /sys/bus/usb/devices/*; do
  [ "$(cat "$d/idVendor" 2>/dev/null)" = 8086 ] && grep -qi realsense "$d/product" 2>/dev/null || continue
  rs_n=$((rs_n+1)); spd=$(cat "$d/speed")
  [ "${spd%%.*}" -ge 5000 ] && ok "$(cat "$d/product") ($(basename "$d")) USB3 $spd Mb/s" \
    || { warn "$(cat "$d/product") ($(basename "$d")) 가 USB2 ($spd Mb/s) — 깊이 해상도 · fps 제한"; fix "USB3 (파란) 포트 · USB3 케이블로, 허브를 거치면 USB3 허브인지"; }
done
[ $rs_n -eq 2 ] && ok "RealSense 2 대 보임" || warn "RealSense $rs_n 대 보임 (2 대 기대)"
want_f=; want_r=
[ -n "$SRC" ] && want_f=$(grep -oP "'front_serial', default_value='\K[0-9]*" "$SRC/bringup/launch/sensors.launch.py") \
  && want_r=$(grep -oP "'rear_serial', default_value='\K[0-9]*" "$SRC/bringup/launch/sensors.launch.py")
if command -v rs-enumerate-devices >/dev/null; then
  got=$(timeout 15 rs-enumerate-devices -s 2>/dev/null | grep -oE '\b[0-9]{12}\b' | sort -u | tr '\n' ' ')
  echo "  rs-enumerate-devices 시리얼: ${got:-(없음)} · launch 기본값 앞 $want_f · 뒤 $want_r"
  for pair in "앞:front:$want_f" "뒤:rear:$want_r"; do
    s=${pair##*:}; arg=${pair#*:}; arg=${arg%%:*}; [ -n "$s" ] || continue
    echo " $got " | grep -q " $s " && ok "${pair%%:*} D435 ($s) 보임" \
      || warn "${pair%%:*} D435 시리얼 $s 가 안 보임 — 다른 카메라면 launch 인자 ${arg}_serial:=<시리얼> 로 줌"
  done
else
  warn "rs-enumerate-devices 없음 — 카메라 시리얼을 launch 기본값 (앞 $want_f · 뒤 $want_r) 과 비교 못 함"
  fix "realsense2_camera 를 띄우면 로그 첫 줄에 시리얼이 나옴 (또는 librealsense2-utils 설치 — 승인 뒤)"
fi

# 영상 장치 · 팔 카메라
for v in /sys/class/video4linux/video*; do
  [ -e "$v" ] && echo "    /dev/$(basename "$v") — $(cat "$v/name") (index $(cat "$v/index" 2>/dev/null))"
done
if grep -qs '^0c45$' /sys/bus/usb/devices/*/idVendor; then ok "Arducam (0c45) USB 보임"; else warn "Arducam (0c45:6366) 안 보임 — vid:pid 가 다르면 99-sensors.rules 와 위 목록 비교"; fi
[ -e /dev/arm_cam ] && ok "/dev/arm_cam → $(readlink -f /dev/arm_cam)" || warn "/dev/arm_cam 없음 (udev 규칙의 vid:pid · index 확인)"

# TFmini (USB-UART) · 라이다
shopt -s nullglob; usbs=(/dev/ttyUSB*); shopt -u nullglob
for t in "${usbs[@]}"; do
  eval "$(udevadm info -q property -n "$t" | grep -E '^(ID_VENDOR_ID|ID_MODEL_ID|ID_SERIAL_SHORT|ID_PATH)=' | sed 's/^/u_/')"
  echo "    $t  ${u_ID_VENDOR_ID}:${u_ID_MODEL_ID} · 시리얼 ${u_ID_SERIAL_SHORT:-?} · USB 자리 ${u_ID_PATH}"
  case ${u_ID_SERIAL_SHORT} in 0001|"") warn "$t 시리얼이 흔한 기본값 (${u_ID_SERIAL_SHORT:-없음}) — 같은 칩 (라이다 등) 이 또 붙으면 시리얼로 구분 안 됨 → by-path 로";; esac
  unset u_ID_VENDOR_ID u_ID_MODEL_ID u_ID_SERIAL_SHORT u_ID_PATH
done
[ ${#usbs[@]} -eq 0 ] && warn "/dev/ttyUSB* 없음 — TFmini USB-UART 를 꽂았는지"
if [ -e /dev/tfmini ]; then ok "/dev/tfmini → $(readlink -f /dev/tfmini)"
else warn "/dev/tfmini 없음 — 99-sensors.rules 의 TFMINI_SERIAL 이 자리표시 (perception 팀 파일 · 위 시리얼로 채워야 생김)"; fi
for l in lidar_front lidar_rear; do [ -e /dev/$l ] && ok "/dev/$l → $(readlink -f /dev/$l)" || echo "  ·  /dev/$l 없음 (LDS-03 미장착이면 정상)"; done

# 커널 로그의 USB 전원 · 대역 문제 (읽을 권한이 없으면 건너뜀)
klog=$(journalctl -k -b --no-pager -q 2>/dev/null | grep -iE "over-current|overcurrent|not enough bandwidth|device descriptor read|unable to enumerate|cannot enable" | tail -5)
if [ -n "$klog" ]; then warn "커널 로그에 USB 전원 · 대역 경고 (최근 5 줄):"; echo "$klog" | sed 's/^/       /'
else echo "  ·  커널 로그: USB 과전류 · 대역 경고 없음 (또는 읽을 권한 없음)"; fi

fi   # 8 끝

echo; echo "== 요약: ✅ $OK · ⚠ $WARN · ❌ $BAD  (읽기만 했음 — 바뀐 것 없음)"
