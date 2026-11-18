"""P1 학습 감시 (설계 6-1 반복 규칙). 학습 로그를 따라 읽으며 250 반복 구간마다 '처음부터' 에피소드 지표 평균을 내고,
500 반복 (두 구간) 동안 최종 성공 · 단계 진행률 4 개가 모두 2 %p 넘게 안 오르면 학습 프로세스에 SIGINT. NaN · OOM · Traceback 이면 바로 멈춤.

  python3 p1_monitor.py <학습 로그> <학습 pid> <요약 파일> [구간 250] [문턱 0.02]
요약 파일에 구간마다 한 줄 (사람 · Claude 가 아침에 봄).
"""
import os
import re
import signal
import sys
import time

log, pid, summ = sys.argv[1], int(sys.argv[2]), sys.argv[3]
W = int(sys.argv[4]) if len(sys.argv) > 4 else 250
TH = float(sys.argv[5]) if len(sys.argv) > 5 else 0.02
KEYS = ["s0_success", "s0_grasp_center", "s0_lift10", "s0_stood", "s0_placed"]
OTHER = ["badspot", "success_rate", "s1_success", "s2_success", "s3_success", "s4_success", "s3_grasp", "s4_grasp", "s0_grasp_any"]
it_re = re.compile(r"Learning iteration (\d+)/(\d+)")
m_re = re.compile(r"(?:Metrics|Episode_End)/(\w+):\s*(-?[\d.]+|nan)")
# 2026-11-10 RL1 조건 2: 환경 변수 CHECK="300:s4_grasp:0.2" = 반복 300 에서 그 지표 최근 50 개 평균이 문턱 아래면 멈춤
CHK = os.environ.get("CHECK", "")
chk_it, chk_key, chk_th = (int(CHK.split(":")[0]), CHK.split(":")[1], float(CHK.split(":")[2])) if CHK else (None, None, None)
checked = False
recent = {}
r_re = re.compile(r"Mean reward:\s*(-?[\d.]+|nan)")


def say(s):
    line = f"{time.strftime('%F %T')} {s}"
    print(line, flush=True)
    with open(summ, "a") as f:
        f.write(line + "\n")


def alive():
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def stop(why):
    say(f"멈춤: {why}")
    try:
        os.kill(pid, signal.SIGINT)
    except OSError:
        pass


cur, start_it = None, None
win, hist = {}, []
rew = []
pos = 0
buf = ""
while alive() or pos < (os.path.getsize(log) if os.path.exists(log) else 0):
    if not os.path.exists(log):
        time.sleep(5)
        continue
    with open(log, "rb") as f:
        f.seek(pos)
        chunk = f.read().decode("utf-8", "replace")
        pos = f.tell()
    if not chunk:
        if not alive():
            break
        time.sleep(10)
        continue
    buf += chunk
    lines = buf.split("\n")
    buf = lines.pop()
    for ln in lines:
        if re.search(r"Traceback|out of memory|CUDA error", ln):
            stop(f"오류 줄: {ln.strip()[:200]}")
        m = it_re.search(ln)
        if m:
            cur = int(m.group(1))
            if start_it is None:
                start_it = cur
            k = (cur - start_it) // W
            if chk_it is not None and not checked and cur - start_it >= chk_it:
                checked = True
                v = recent.get(chk_key, [])
                mv = sum(v) / len(v) if v else float("nan")
                say(f"중간 확인 (반복 {cur}): {chk_key} 최근 {len(v)} 개 평균 {mv:.3f} (문턱 {chk_th}) | " + " ".join(f"{kk} {sum(vv) / len(vv):.3f}" for kk, vv in sorted(recent.items()) if vv))
                if not (mv >= chk_th):
                    stop(f"중간 확인: {chk_key} {mv:.3f} < {chk_th}")
            if k > len(hist) and win:  # 구간 끝
                avg = {key: sum(v) / len(v) for key, v in win.items() if v}
                hist.append(avg)
                rr = sum(rew) / max(1, len(rew))
                say(f"구간 {len(hist)} (반복 {start_it + (len(hist) - 1) * W} ~ {start_it + len(hist) * W - 1}): 보상 {rr:.2f} | "
                    + " ".join(f"{key} {avg.get(key, float('nan')):.3f}" for key in KEYS + OTHER))
                win, rew = {}, []
                if len(hist) >= 3:
                    a, b, c = hist[-3], hist[-2], hist[-1]
                    up1 = any(b.get(key, 0) - a.get(key, 0) > TH for key in KEYS)
                    up2 = any(c.get(key, 0) - b.get(key, 0) > TH for key in KEYS)
                    if not up1 and not up2:
                        stop(f"500 반복 동안 성공 · 단계 진행률 모두 {TH * 100:.0f} %p 넘게 안 오름")
        m = r_re.search(ln)
        if m:
            if m.group(1) == "nan":
                stop("보상 NaN")
            else:
                rew.append(float(m.group(1)))
        m = m_re.search(ln)
        if m and m.group(1) in KEYS + OTHER:
            if m.group(2) == "nan":
                continue
            win.setdefault(m.group(1), []).append(float(m.group(2)))
            recent.setdefault(m.group(1), []).append(float(m.group(2)))
            recent[m.group(1)] = recent[m.group(1)][-50:]
if win:
    avg = {key: sum(v) / len(v) for key, v in win.items() if v}
    say(f"마지막 구간 (반복 ~{cur}): " + " ".join(f"{key} {avg.get(key, float('nan')):.3f}" for key in KEYS + OTHER))
say("감시 끝 (학습 프로세스 끝남)")
