"""1 s마다 UDP 수신 버퍼 넘침(RcvbufErrors)과 프로세스별 CPU 사용률을 sysmon.csv에 쓴다.

사용: python3 sysmon.py --out <폴더>   (SIGINT로 끝냄)
CPU %는 직전 1 s 동안의 (utime+stime) 증가 ÷ 실제 시간 × 100 (코어 1개 = 100 %).
"""

import argparse
import csv
import os
import signal
import time

PATTERNS = {"R1_observer": "observe_l2.py", "R2_rx_min": "rx_min.py", "R3_rx_cpp": "rx_counter", "isaac": "scene_l2.py"}
HZ = os.sysconf("SC_CLK_TCK")


def udp_counters():
    with open("/proc/net/snmp") as f:
        lines = [ln.split() for ln in f if ln.startswith("Udp:")]
    d = dict(zip(lines[0][1:], (int(x) for x in lines[1][1:])))
    return d["RcvbufErrors"], d["InErrors"], d["InDatagrams"]


def find_pids():
    found = {}
    for pid in os.listdir("/proc"):
        if not pid.isdigit():
            continue
        try:
            with open(f"/proc/{pid}/cmdline", "rb") as f:
                cmd = f.read().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        for name, pat in PATTERNS.items():
            if pat in cmd and "bash -c" not in cmd and name not in found:
                found[name] = int(pid)
    return found


def cpu_ticks(pid):
    try:
        with open(f"/proc/{pid}/stat") as f:
            parts = f.read().rsplit(")", 1)[1].split()
        return int(parts[11]) + int(parts[12])
    except (OSError, IndexError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    stop = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stop.update(flag=True))
    signal.signal(signal.SIGTERM, lambda *_: stop.update(flag=True))
    names = list(PATTERNS)
    with open(os.path.join(a.out, "sysmon.csv"), "w", newline="", buffering=1) as f:
        w = csv.writer(f)
        w.writerow(["wall_s", "rcvbuf_errors", "in_errors", "in_datagrams"] + [f"cpu_{n}" for n in names])
        prev, prev_t = {}, time.time()
        while not stop["flag"]:
            time.sleep(1.0)
            now = time.time()
            pids = find_pids()
            cpu = []
            for n in names:
                t = cpu_ticks(pids[n]) if n in pids else None
                old = prev.get(n)
                cpu.append(f"{(t - old[1]) / HZ / (now - prev_t) * 100:.1f}" if (t is not None and old and old[0] == pids[n])
                           else "")
                prev[n] = (pids.get(n), t) if t is not None else None
            prev_t = now
            w.writerow([f"{now:.3f}", *udp_counters()] + cpu)


if __name__ == "__main__":
    main()
