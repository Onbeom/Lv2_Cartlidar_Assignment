#!/usr/bin/env python3
"""팔 보드에 붙어 있는 컴퓨터에서 돈다. 표준 입력의 한 줄 명령으로 팔을 움직이고 상태를 JSON 줄로 낸다.

명령: start · stop · halt · hb · quit · step <칸 0~5> <+1|-1> <rad>
입력이 끊기거나 hb 가 1.5 s 없으면 ESTOP 뒤 현재 자리에서 HOLD(토크 유지).
안전 규칙(arm_control.ArmController, 팀 슬라이더 제어부)을 그대로 쓴다: 속도 상한, 추종 오차 감시, 오류 비트 정지, 그리퍼 막힘.
"""
import argparse
import json
import queue
import sys
import threading
import time

from teleop_common import ROLE_ARM, find_port, tool
import arm_control

HB_TIMEOUT = 1.5
LIMIT_MARGIN = 0.002  # rad


def step(ctl, i, sign, size):
    """목표는 명령보다 한 칸 넘게 앞서지 않는다. 키를 떼면 많아야 한 칸 더 가고 멈춘다."""
    snap = ctl.snapshot()
    if snap['state'] != 'RUN' or snap['target'] is None or snap['cmd'] is None:
        return
    # 한계 바로 앞에서 멈춘다. float32 로 바뀌면서 한계를 넘어 CMD 가 거부되는 일을 막는다.
    new = min(ctl.hi[i] - LIMIT_MARGIN, max(ctl.lo[i] + LIMIT_MARGIN, snap['target'][i] + sign * size))
    if abs(new - snap['cmd'][i]) > size + 1e-6:
        return
    ctl.set_target(i, new)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--port', default='auto')
    ap.add_argument('--speed', type=float, default=0.2, help='관절 명령 변화 상한 rad/s (슬라이더 기본값)')
    a = ap.parse_args()
    port = find_port(ROLE_ARM, a.port)
    ctl = arm_control.ArmController(port, speed=a.speed)
    lines = queue.Queue()

    def reader():
        for line in sys.stdin:
            lines.put(line.strip())
        lines.put(None)

    threading.Thread(target=reader, daemon=True).start()
    print(json.dumps({'ready': True, 'port': port, 'lo': ctl.lo, 'hi': ctl.hi}), flush=True)
    last_hb, t_out, prev = time.monotonic(), 0.0, None
    hb_lost = False
    done = False
    while not done:
        now = time.monotonic()
        try:
            while True:
                m = lines.get_nowait()
                if m is None or m == 'quit':
                    done = True
                    break
                last_hb, hb_lost = now, False
                p = m.split()
                if p[0] in ('start', 'stop'):
                    ctl.request(p[0])
                elif p[0] == 'halt':
                    snap = ctl.snapshot()
                    if snap['state'] == 'RUN':
                        for i in range(arm_control.N_MOVE):
                            ctl.set_target(i, snap['cmd'][i])
                elif p[0] == 'step':
                    step(ctl, int(p[1]), int(p[2]), float(p[3]))
        except queue.Empty:
            pass
        if not hb_lost and now - last_hb > HB_TIMEOUT:
            hb_lost = True
            ctl.request('stop')
        snap = ctl.snapshot()
        s = snap['last']
        cur = (snap['state'], snap['msg'])
        if cur != prev or now - t_out > 0.25:
            print(json.dumps({'state': snap['state'], 'msg': snap['msg'], 't': round(now, 3),
                              'q': [round(x, 4) for x in tool.arm_pos(s)] if s else None,
                              'cmd': [round(x, 4) for x in snap['cmd']] if snap['cmd'] else None,
                              'cur': [round(x, 3) for x in s['effort'][tool.ARM0:tool.ARM0 + 6]] if s else None,
                              'err': tool.err_text(s['error']) if s else None}, ensure_ascii=False), flush=True)
            prev, t_out = cur, now
        time.sleep(0.02)
    ctl.close()
    print(json.dumps({'state': 'END', 'msg': '끝: 송신 중단, 펌웨어가 현재 자리에서 HOLD(토크 유지)'}, ensure_ascii=False), flush=True)


if __name__ == '__main__':
    main()
