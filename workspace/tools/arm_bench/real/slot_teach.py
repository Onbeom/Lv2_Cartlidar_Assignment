#!/usr/bin/env python3
"""퍽스토리지 (팔레트 v5) 칸 위치 맞추기 (2026-11-13, 젯슨, 브링업 상태). 칸마다 실물 퍽에 집게를 손으로 맞춰 기록하고,
그 차이로 노드 팔레트 궤적 (pallet_place_traj.json) 을 고친 판을 만든다.

    python3 ~/slot_teach.py                 10 칸 모두 차례로 (각 칸: 칸 위 -> 맞춤 -> ok)
    python3 ~/slot_teach.py 1_3 2_2         고른 칸만
    python3 ~/slot_teach.py --make          기록 (~/실측/slot_teach.json) -> ~/pallet_place_traj_taught.json (노드용)
    python3 ~/slot_teach.py --show          기록된 칸별 차이 (mm · 도) 만 출력
    python3 ~/slot_teach.py --set 3_1 왼 5 [node|offset]   맞춰 보지 않고 말로: 쥐는 자리에서 5 mm 왼쪽 (offset = 영점 보정 더한 자리 기준)

준비: 브링업 (skill_enabled:=true) + kbd_joy.py 에서 r -> 팔이 DRIVE 까지 펴진 뒤. **맞출 칸에만 퍽 하나**를 넣어 둠 (다른 칸은 비움).
맞춤 기준 (노드 쥐는 자리): 퍽이 두 손가락 정가운데, 손가락 가운데가 퍽 윗면보다 15 mm 아래, 집게 45° 기울임 그대로.
  조정 명령 (meas_session 과 같음): '아래 5' · '위 3' · '멀리 2' · '가까이 2' · '왼 1' · '오른 1' (mm, 로봇 기준) · '집게 42' (손가락 사이 mm)
  ok = 맞춤 끝 · 기록 / b = 이 칸 건너뜀 / q = 끝
시작 자세 = 노드가 실제로 보내는 값 (모델 궤적, pallet.apply_joint_offsets 가 true 면 + ik.joint_offsets) 의 칸 위 (place 구간 처음) -> 맞춘 관절 - 노드 쥐는 자리 = 칸 보정.
--make: 맞춘 칸은 그 보정을 칸 궤적 (carry · place · release · retract · return 전부, rotate · rotate_back 은 앞 0 -> 칸 1 로 서서히) 에 더함.
        안 맞춘 칸은 같은 열에서 맞춘 칸들의 평균 (없으면 그대로). 노드에 넣는 법은 --make 끝에 출력.
"""
import datetime
import json
import math
import os
import sys

REPO = os.environ.get("TEAM_WS", os.path.expanduser("~/Lv2_Cartlidar_Assignment/workspace"))
TRAJ = os.path.join(REPO, "ros2_ws/src/control/config/pallet_place_traj.json")
YAML = os.path.join(REPO, "ros2_ws/src/control/config/manipulator.yaml")
OUT_DIR = os.environ.get("TEACH_OUT", os.path.expanduser("~/실측"))
REC = os.path.join(OUT_DIR, "slot_teach.json")
ORIG = os.path.expanduser("~/pallet_place_traj_orig.json")   # 노드 궤적을 바꾸기 전 원본 (--make 는 늘 이걸 기준으로)
TAUGHT = os.environ.get("TEACH_TAUGHT", os.path.expanduser("~/pallet_place_traj_taught.json"))
ORDER = ["1_3", "1_0", "1_2", "1_1", "2_2", "2_0", "2_3", "2_1", "3_1", "3_0"]   # 계획기 꺼내기 차례 (앞 열부터)


def offsets(force=False):
    """노드가 팔레트 궤적에 더하는 값: pallet.apply_joint_offsets 가 true 면 ik.joint_offsets, false 면 0 (#273, 11-14).
    force=True 면 설정과 관계없이 ik.joint_offsets."""
    import yaml
    d = yaml.safe_load(open(YAML, encoding="utf-8"))
    node = next(iter(d.values()))["ros__parameters"]
    if not force and not node.get("pallet", {}).get("apply_joint_offsets", True):
        return [0.0] * 5
    return [float(v) for v in node["ik"]["joint_offsets"]][:5]


# 집게 값 <-> 손가락 사이 (mm): 실측 3 점 (-0.764 = 66.7, 0.0798 = 36.4 (#190), 0.922 = 23.8) 사이 직선.
#   (meas_session '집게 42' 의 직선식은 0.21 rad = 실물 약 34 mm 라 거의 안 벌어짐 -> 퍽을 누름, 11-14 실물)
GRIP_PTS = [(-0.764, 66.7), (0.0798, 36.4), (0.922, 23.8)]


def grip_for_mm(mm):
    (g0, w0), (g1, w1), (g2, w2) = GRIP_PTS
    if mm >= w1:
        return g0 + (w0 - mm) / (w0 - w1) * (g1 - g0)
    return g1 + (w1 - mm) / (w1 - w2) * (g2 - g1)


TEACH_OPEN_MM = 50.0   # 맞출 때 벌림 (퍽 30 mm + 양쪽 10 mm)


def load_traj(base=False):
    """base=True: 보정 계산 기준 = 원본 (ORIG 가 있으면, 없으면 지금 노드 파일). False: 지금 노드가 쓰는 파일."""
    t = json.load(open(ORIG if base and os.path.exists(ORIG) else TRAJ, encoding="utf-8"))
    return t, {f"{s['row']}_{s['index']}": s for s in t["slots"]}


def seg(s, name):
    return next(g for g in s["segments"] if g["name"] == name)["q"]


def load_rec():
    return json.load(open(REC, encoding="utf-8")) if os.path.exists(REC) else {}


def fk_mm(m, q):
    p, _ = m.fk(list(q) + [0.0] * (5 - len(q)))
    return [v * 1000 for v in p]


def show(m=None):
    rec = load_rec()
    if not rec:
        print("기록 없음:", REC); return
    if m is None:
        sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real")); import meas_session as m   # noqa: E401
    print("칸   | 관절 보정 (도) j1 j2 j3 j4 | 손끝 차이 (mm, 노드 쥐는 자리 -> 맞춘 자리) x y z | 시각")
    for k in ORDER:
        if k in rec:
            r = rec[k]
            a, b = fk_mm(m, r["q_node"]), fk_mm(m, r["q_meas"])
            print(f"{k}  | " + " ".join(f"{math.degrees(v):+5.1f}" for v in r["delta"][:4])
                  + " | " + " ".join(f"{y - x:+6.1f}" for x, y in zip(a, b)) + f" | {r['t']}")


def make():
    t, slots = load_traj(base=True)
    rec = load_rec()
    if not rec:
        sys.exit(f"기록 없음: {REC} (먼저 칸을 맞추세요)")
    off = offsets()
    for k, r in rec.items():   # 보정 = 맞춘 관절 - 원본 쥐는 자리 (노드 파일이 이미 바뀌었어도 두 번 더하지 않음)
        g = [v + o for v, o in zip(seg(slots[k], "place")[-1][:5], off)]
        r["delta"] = [x - y for x, y in zip(r["q_meas"], g)]
    rows = {}
    for k, r in rec.items():
        rows.setdefault(k[0], []).append(r["delta"])
    used = {}
    for k, s in slots.items():
        if k in rec:
            d, how = rec[k]["delta"], "맞춤"
        elif k[0] in rows:
            ds = rows[k[0]]
            d, how = [sum(x[j] for x in ds) / len(ds) for j in range(5)], f"{k[0]} 열 평균"
        else:
            used[k] = "그대로"; continue
        used[k] = how
        for g in s["segments"]:
            n = len(g["q"])
            for i, q in enumerate(g["q"]):
                # 칸 근처만 고침 (#273: 이동 자세에 보정을 더하면 팔꿈치가 내려가 퍽 스토리지를 누름):
                #   rotate · rotate_back (관절 1 회전) 그대로, carry 는 칸 쪽으로 0 -> 1, return 은 1 -> 0, place · release · retract 전부
                if g["name"] in ("rotate", "rotate_back"):
                    f = 0.0
                elif g["name"] == "carry":
                    f = (i + 1) / n
                elif g["name"] == "return":
                    f = 1 - (i + 1) / n
                else:
                    f = 1.0
                for j in range(5):
                    q[j] = round(q[j] + f * d[j], 5)
        if "q_front" in s:
            pass                               # 꺼내기 끝 앞 자세는 그대로 (칸에서 먼 곳)
    t["note"] = (f"{datetime.datetime.now():%Y-%m-%d %H:%M} slot_teach.py 실물 칸 맞춤 반영 ("
                 + ", ".join(f"{k} {v}" for k, v in sorted(used.items())) + "). 원본: " + (ORIG if os.path.exists(ORIG) else TRAJ) + "  ") + t["note"]
    json.dump(t, open(TAUGHT, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print("만듦:", TAUGHT)
    for k in ORDER:
        print(f"  {k}: {used.get(k)}")
    print("\n노드에 넣기 (원본은 백업 · 되돌리기는 git checkout):")
    print(f"  cp {TRAJ} ~/pallet_place_traj_orig.json && cp {TAUGHT} {TRAJ}")
    print("  -> 브링업 다시 띄우기 (노드가 시작할 때 읽음). 되돌리기: "
          f"cd {REPO} && git checkout ros2_ws/src/control/config/pallet_place_traj.json")


def teach(names):
    sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real"))
    import meas_session as m                   # Arm (arm_controller 액션) · jog · fk 그대로
    _, slots = load_traj()
    off = offsets()
    bad = [k for k in names if k not in slots]
    if bad:
        sys.exit(f"없는 칸 {bad} (팔레트 v5: {' '.join(ORDER)})")
    m.P(f"\n===== 칸 맞추기 {datetime.datetime.now():%Y-%m-%d %H:%M} 칸 {' '.join(names)} | ik.joint_offsets {off} =====")
    a = m.Arm()
    q, _ = a.show()
    if q[1] < -1.9:
        sys.exit("STOW 자세로 보임 -> kbd_joy.py 에서 r 로 DRIVE 까지 편 뒤 다시")
    rec = load_rec()
    os.makedirs(OUT_DIR, exist_ok=True)
    try:
        for k in names:
            s = slots[k]
            pl = seg(s, "place")
            above = [v + o for v, o in zip(pl[0][:5], off)]       # 노드가 보내는 칸 위 (모델 + 보정)
            grasp = [v + o for v, o in zip(pl[-1][:5], off)]      # 노드가 보내는 쥐는 자리
            m.P(f"\n[{k}] {s['shape']} 칸 — 이 칸에만 퍽을 넣었는지 확인. 칸 위 (노드 값) 로 감")
            if m.ask("시작? (y = 시작 / s = 건너뜀)") != "y":
                continue
            a.move(a.cmd, grip_for_mm(TEACH_OPEN_MM), label=f"집게 {TEACH_OPEN_MM:.0f} mm 벌림")
            if not a.move(above, label=f"{k} 칸 위 (노드 값)"):
                continue
            m.P("  노드 쥐는 자리로 내릴까? (퍽 바로 위면 '아래' 로 직접 내려도 됨)")
            if m.ask("내림? (y / n)") == "y":
                a.move(grasp, label=f"{k} 쥐는 자리 (노드 값)")
            qm = m.jog(a, above, f"{k} 칸 위")
            if qm is not None:
                delta = [x - y for x, y in zip(qm, grasp)]
                rec[k] = {"q_meas": qm, "q_node": grasp, "delta": delta, "t": f"{datetime.datetime.now():%m-%d %H:%M}"}
                json.dump(rec, open(REC, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
                a_mm, b_mm = fk_mm(m, grasp), fk_mm(m, qm)
                m.P(f"[기록] {k} 관절 보정 (도) " + " ".join(f"{math.degrees(v):+.1f}" for v in delta)
                    + " | 손끝 차이 (mm) " + " ".join(f"{y - x:+.1f}" for x, y in zip(a_mm, b_mm)))
                if m.ask("쥐어 볼까? (y = 닫고 2 초 뒤 열기 / n)") == "y":
                    a.move(a.cmd, 0.923, confirm=False, T=1.5, label="닫기")
                    _, g = a.show("쥔 집게")
                    m.P(f"  집게 {g:+.3f} rad ({'쥠 범위 안' if 0.30 <= g <= 0.773 else '쥠 범위 밖 — 빈손이거나 너무 큼'})")
                    a.move(a.cmd, grip_for_mm(TEACH_OPEN_MM), confirm=False, T=1.5, label=f"집게 열기 {TEACH_OPEN_MM:.0f} mm")
            a.move(above, label="칸 위로 올림")
            a.move([above[0]] + m.HIGH, label="높은 자세")
    except m.Back:
        m.P("  (끝냄)")
    a.move([0] + m.HIGH, label="정면 높은 자세")
    m.P(f"기록: {REC} (칸 {len(rec)} 개). 다음: python3 ~/slot_teach.py --make")


def set_nudge(slot, d, mm, base):
    """맞춰 보지 않고 말로 보정: 칸 slot 의 쥐는 자리에서 방향 d (meas_session jog 와 같은 이름) 로 mm 옮긴 자리를 기록.
    base = node (노드가 지금 보내는 자리 기준) | offset (ik.joint_offsets 를 더한 자리 기준 — 11-14 이전 slot_teach 가 간 자리)."""
    sys.path.insert(0, os.path.join(REPO, "tools/arm_bench/real")); import meas_session as m   # noqa: E401
    _, slots = load_traj(base=True)
    if slot not in slots:
        sys.exit(f"없는 칸 {slot}")
    g = seg(slots[slot], "place")[-1][:5]
    seen = [v + o for v, o in zip(g, offsets(force=(base == "offset")))]
    node = [v + o for v, o in zip(g, offsets())]
    if d in ("멀리", "가까이"):
        p, _ = m.fk(seen); ux, uy = p[0] - 0.096, p[1]; nn = math.hypot(ux, uy) or 1.0
        v = ((1 if d == "멀리" else -1) * ux / nn, (1 if d == "멀리" else -1) * uy / nn, 0)
    elif d in m.DIRS:
        v = m.DIRS[d]
    else:
        sys.exit(f"방향은 {' · '.join(list(m.DIRS) + ['멀리', '가까이'])}")
    qm, err = m.cart_step(seen, [c * mm / 1000 for c in v])
    if err > 1.0:
        sys.exit("그 방향으로 IK 안 됨")
    rec = load_rec()
    rec[slot] = {"q_meas": qm, "q_node": node, "delta": [x - y for x, y in zip(qm, node)],
                 "t": f"{datetime.datetime.now():%m-%d %H:%M}", "how": f"{base} 자리에서 {d} {mm:g} mm (말로)"}
    os.makedirs(OUT_DIR, exist_ok=True)
    json.dump(rec, open(REC, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"[기록] {slot}: {rec[slot]['how']}")
    show(m)


def main():
    args = sys.argv[1:]
    if args and args[0] == "--set":   # --set 3_1 왼 5 [offset]
        set_nudge(args[1], args[2], float(args[3]), args[4] if len(args) > 4 else "node")
        return
    if "--make" in args:
        make()
    elif "--show" in args:
        show()
    else:
        teach([x for x in args if not x.startswith("-")] or ORDER)


if __name__ == "__main__":
    main()
