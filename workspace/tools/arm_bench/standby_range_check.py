#!/usr/bin/env python3
"""대기 자세 tick 범위 (opencr.ros2_control.xacro arm_standby_*_tick) 모서리에서 켜질 때 정리 경로 최소 틈 (#305, 계산만).

- 범위 모서리 (J1 ~ J5 각 끝값 조합 32 곳) 에서 켜질 때 정리 (J3 → J2 → J4 → J1 → J5 · 그리퍼) 로 DRIVE E2 까지 가는 경로를
  fold_clearance.py 와 같은 모델 (설계 URDF · 팔 상자 ↔ 차체 · 팔레트 v5 · 퍽 겉면 점) 로 잼.
- 11-15 범위 (#305) 와 11-16 범위 (관절 1 #277 로 되돌림) 를 나란히 출력. 상판 (static_427) 은 접은 팔이 얹히는 곳이라 --no-plate 로 빼고 볼 수 있음.

실행: python3 standby_range_check.py [--j2 653,721] [--no-plate]   (numpy · scipy, 약 1 분)
"""
import argparse
import contextlib
import io
import itertools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fold_clearance as F  # noqa: E402

OLD = dict(j1=(3174, 3325), j5=(1849, 2246))   # 11-15 #305 (관절 1 모터 돌아 설치)
NEW = dict(j1=(3425, 3539), j5=(1849, 2246))   # 11-16 관절 1 모터 정상 설치, J1 #277 로 되돌림
J3, J4 = (3025, 3091), (797, 864)
E2 = [0.0, -1.37, -0.01, 1.85, 0.0, 0.923]
ORDER = [[2], [1], [3], [0], [4, 5]]


def t2r(t):
    return (t - 2048) * 2 * math.pi / 4096


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--j2', default='653,721', help='관절 2 tick 끝값 (STOW 값 −2.088 rad = 687 만 보려면 687,721)')
    ap.add_argument('--no-plate', action='store_true', help='상판 (static_427) 을 빼고 칸 벽 · 퍽 · 차체만')
    a = ap.parse_args()
    j2 = tuple(int(v) for v in a.j2.split(','))
    m = F.Model(os.path.join(F.DESC, 'urdf/waffle_4wd_loaded.urdf'))
    obs, tags = m.obstacle_points('base_link', with_pucks=True)
    if a.no_plate:
        keep = tags != 'static_427'
        obs, tags = obs[keep], tags[keep]
    boxes = m.arm_boxes()
    for name, r in (('11-15 범위 (#305)', OLD), ('11-16 범위', NEW)):
        res = []
        for c in itertools.product(r['j1'], j2, J3, J4, r['j5']):
            q = [t2r(v) for v in c] + [0.923]
            with contextlib.redirect_stdout(io.StringIO()):
                _, per = F.evaluate(m, obs, tags, boxes, F.path(q, E2, ORDER), '')
            st, (d, ln, tg, _) = min(per.items(), key=lambda kv: kv[1][0])
            res.append((d, per['start'][0], c, st, ln, tg))
        res.sort()
        print(f'== {name}: 모서리 {len(res)} 곳, 경로 최소 {res[0][0] * 1000:+.1f} mm · 대기 자세 자체 최소 {min(x[1] for x in res) * 1000:+.1f} mm')
        for d, s0, c, st, ln, tg in res[:3]:
            print(f'   경로 {d * 1000:+6.1f} · 시작 {s0 * 1000:+6.1f} mm  tick J1..J5={c}  단계 {st} {ln} ↔ {tg}')


if __name__ == '__main__':
    main()
