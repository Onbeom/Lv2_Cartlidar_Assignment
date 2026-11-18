#!/usr/bin/env bash
# 받는 쪽 영상 손실 진단 한 번 (PC, 검증계획 12장). L2 장면 600 s + 수신기 R1·R2·R3 동시 + RcvbufErrors·CPU 기록.
#   사용: 261025_0307/diag/run_diag.sh D1|D2 [시뮬레이션 시간 s, 기본 600]
#   D1: 지금 그대로(rmem_max 4 MB, CycloneDDS 설정 없음)
#   D2: 사용자가 먼저 rmem_max를 64 MB로 올린 뒤 실행. 수신기 3개에 CycloneDDS 수신 버퍼 최소 16 MB 설정을 준다
set -u
COND=${1:?D1 또는 D2}
DURATION=${2:-600}
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORK="$(dirname "$HERE")"
OUT="$WORK/results/diag_${COND}_$(date +%y%m%d_%H%M%S)"
mkdir -p "$OUT"
RMEM=$(sysctl -n net.core.rmem_max)
echo "$RMEM" > "$OUT/rmem_max.txt"

COMMON_ENV=(HOME="$HOME" USER="$USER" LOGNAME="$USER" LANG=en_US.UTF-8
            PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
            ROS_DOMAIN_ID=26 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
RX_ENV=()
if [ "$COND" = "D2" ]; then
    if [ "$RMEM" -lt 67108864 ]; then
        echo "[diag] D2인데 rmem_max가 $RMEM 이다. 먼저 64 MB(67108864)로 올려야 한다"; exit 1
    fi
    cat > "$OUT/cyclonedds_rx.xml" <<'XML'
<?xml version="1.0" encoding="UTF-8"?>
<CycloneDDS xmlns="https://cdds.io/config">
  <Domain id="any">
    <Internal>
      <SocketReceiveBufferSize min="16MB"/>
    </Internal>
  </Domain>
</CycloneDDS>
XML
    RX_ENV=(CYCLONEDDS_URI="file://$OUT/cyclonedds_rx.xml")
fi
echo "[diag] 조건 $COND, rmem_max $RMEM, 결과 $OUT"

stop_pid() { kill -INT "$1" 2>/dev/null; for _ in $(seq 20); do kill -0 "$1" 2>/dev/null || return 0; sleep 0.5; done; kill -9 "$1" 2>/dev/null; }

python3 "$HERE/sysmon.py" --out "$OUT" &
MON_PID=$!
env -i "${COMMON_ENV[@]}" "${RX_ENV[@]}" bash -c "source /opt/ros/lyrical/setup.bash && exec python3 '$HERE/rx_min.py' --out '$OUT'" \
    > "$OUT/rx_min_stdout.txt" 2>&1 &
R2_PID=$!
env -i "${COMMON_ENV[@]}" "${RX_ENV[@]}" L2_RX_OUT="$OUT/rx_cpp.csv" bash -c "source /opt/ros/lyrical/setup.bash && source '$HERE/ws/install/setup.bash' && exec '$HERE/ws/install/l2_rx_counter/lib/l2_rx_counter/rx_counter'" \
    > "$OUT/rx_cpp_stdout.txt" 2>&1 &
R3_PID=$!
sleep 2

# R1(관찰 노드)에도 같은 수신 설정을 준다: run_l2.sh가 OBS_EXTRA_ENV를 관찰 노드 환경에만 넣는다
OBS_EXTRA_ENV="${RX_ENV[*]:-}" "$WORK/l2/run_l2.sh" "$DURATION" "$OUT"

stop_pid "$R2_PID"
stop_pid "$R3_PID"
stop_pid "$MON_PID"
wait 2>/dev/null
python3 "$HERE/analyze_diag.py" "$OUT"
