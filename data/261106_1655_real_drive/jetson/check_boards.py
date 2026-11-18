#!/usr/bin/env python3
"""OpenCR 두 보드(팔 role 2 · 바퀴 role 1) 상태 3 초 읽기 (읽기만, 포트에 쓰지 않음). check_boards.py
정상: 전압 약 11~12.6 V, 오류는 CMD_TIMEOUT 만(명령 보내는 프로그램이 없을 때), 관절 위치가 ±3.142 가 아님.
전압 약 0 V · SYNCREAD · MOTOR_CONFIG · FAULT = 12 V 가 안 들어감 또는 12 V 전에 보드가 켜짐 → 12 V 켠 채 보드 리셋."""
import glob, time
from teleop_common import tool
ports = sorted(glob.glob('/dev/ttyACM*'))
if not ports:
    print('/dev/ttyACM* 없음 — OpenCR USB 가 안 꽂혔거나 보드 전원이 없음 (12 V 먼저 켜고 USB, 아니면 보드 RESET)')
for p in ports:
    try:
        l = tool.Link(p); s = l.wait_state(1.5)
        if not s:
            print(p, 'STATE 없음'); l.ser.close(); continue
        t0 = time.time(); errs = set(); last = s
        while time.time() - t0 < 3:
            s2 = l.poll()
            if s2:
                last = s2; errs.add(tool.err_text(s2['error']))
        name = {1: '바퀴', 2: '팔'}.get(last['role'], '?')
        pos = last['pos'][6:12] if last['role'] == 2 else last['pos'][:4]
        bad = all(abs(abs(v) - 3.142) < 0.01 for v in pos)
        ok = last.get('battery', 0) > 10 and not (last['error'] & tool.ERR_ABORT) and not bad
        print(f"{p} {name}(role {last['role']}) 전압 {last.get('battery', 0):.2f} V, 오류 {' / '.join(sorted(errs))}, "
              f"위치 {[round(v, 3) for v in pos]} → {'정상' if ok else '이상'}")
        l.ser.close()
    except Exception as e:
        print(p, type(e).__name__, e)
