"""비스듬히 서기 mock 미션 결과 확인 (run_mock.sh 결과 폴더).

사용: python3 check_oblique.py <run 폴더> --oblique 0|1 --rot <deg> --shift dx,dy [--base <기준 run 폴더>]
  - 켬: mock 의 진짜 상자 · 재기 때 차체 자세로 기대값 (beta · 회전각 · R_look · TFmini 목표 · 방법) 을 따로 계산해 mission_log "oblique" 줄과 비교,
        mock 빔 계산으로 접근 뒤 상자 중심이 축 위 R_look 에 왔는지, box_look 문자열 (놓기 0.44/50 · verify 0.40/55) 과 조준점, DONE
  - 끔: measure_front · oblique 줄 없음, box_look 5 칸만. --base 면 기준 (origin/dev 빌드) 과 단계 · 주행 · 정렬 · 팔 · 모드 · 재기 기록이 같은지
결과를 <run>/check.json 에 쓰고 요약을 출력. 하나라도 틀리면 종료 코드 1.
"""
import argparse
import glob
import json
import math
import re
import sys

FRONT, HALF, TF_X, ICR = 0.1328, 0.1025, 0.13, -0.0658
VIEW_GAP, TF_MAX_DEG, MIN_TURN_DEG = 0.35, 30.0, 0.5
PLACE_LOOK, TAG_LOOK = (0.44, 50.0), (0.40, 55.0)


def cround(x):
    """C++ std::round (반올림, 0.5 는 0 에서 먼 쪽)."""
    return math.copysign(math.floor(abs(x) + 0.5), x)


def fold45(a):
    return a - (math.pi / 2) * cround(a / (math.pi / 2))


def look_range(beta):
    return FRONT + VIEW_GAP + HALF * (abs(math.cos(beta)) + abs(math.sin(beta)))


def tf_target(r, beta):
    return r - HALF / math.cos(beta) - TF_X


def expect_place(robot, box_xy, rot_deg):
    x, y, yaw = robot
    bx, by = box_xy
    cx = math.cos(-yaw) * (bx - x) - math.sin(-yaw) * (by - y)
    cy = math.sin(-yaw) * (bx - x) + math.cos(-yaw) * (by - y)
    beta = fold45(math.radians(rot_deg) - yaw)
    phi = math.atan2(cy, cx - ICR)
    turned = abs(math.degrees(phi)) > MIN_TURN_DEG
    bv = fold45(beta - phi) if turned else beta
    r_now = ICR + math.hypot(cx - ICR, cy) if turned else cx
    rl = look_range(bv)
    t = tf_target(rl, bv)
    return dict(center_base=[cx, cy], beta_deg=math.degrees(beta), beta_view_deg=math.degrees(bv), phi_deg=math.degrees(phi),
                turn_deg=math.degrees(phi) if turned else 0.0, turned=turned, range_now=r_now, R_look=rl, align_target=t,
                method="tfmini" if abs(math.degrees(bv)) <= TF_MAX_DEG else "tfmini_wide")


def look_fields(name):
    f = name.split(":")
    return f if f[0] == "box_look" else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("run")
    ap.add_argument("--oblique", type=int, required=True)
    ap.add_argument("--rot", type=float, default=0.0)
    ap.add_argument("--shift", default="0,0")
    ap.add_argument("--base", default="")
    a = ap.parse_args()
    out = json.load(open(f"{a.run}/mock_out.json", encoding="utf-8"))
    logs = glob.glob(f"{a.run}/log/*/mission_log.jsonl")
    ev = [json.loads(line) for line in open(logs[0], encoding="utf-8")] if logs else []
    obl = [e for e in ev if e["kind"] == "oblique"]
    mlog = open(f"{a.run}/mission.log", encoding="utf-8", errors="replace").read()
    steps = [re.sub(r"^\[\d+\] ", "", s) for s in out["steps_seen"]]
    checks = []

    def ck(name, ok, detail=""):
        checks.append(dict(check=name, ok=bool(ok), detail=detail))

    ck("미션 DONE", out["final"].get("state") == "DONE", json.dumps(out["final"], ensure_ascii=False))
    looks = [look_fields(s) for s in out["skill"] if look_fields(s)]
    measured = [s.split()[1] for s in steps if s.startswith("measure_box")]
    verifies = [s.split()[1] for s in steps if s.startswith("verify")]
    places = []
    if a.oblique:
        mf = [m for m in out["measure_front"] if m.get("ok")]
        ck("measure_front 장소마다 1 번", sorted(m["place"] for m in mf) == sorted(set(measured)) and len(mf) == len(set(measured)),
           f"measure_front {[m['place'] for m in mf]}, measure_box {measured}")
        ck("사전값 (지도 칸, 5 s 안) 을 measure_front 전에 받음",
           all(m["prior"] and m["prior"]["frame"] == "map" and m["prior"]["age_s"] < 5.0 for m in mf),
           str([m["prior"] for m in mf]))
        pev = [e for e in obl if e.get("event") == "place"]
        ck("oblique place 줄 = measure_front 수", len(pev) == len(mf), f"{len(pev)} vs {len(mf)}")
        used = set()
        spins = [n for n in out["nav_sim"] if n["kind"] == "spin"]
        n_turned = 0
        for e, m in zip(pev, mf):
            x = expect_place(m["robot"], m["map_center"], m["rot_deg"])
            n_turned += x["turned"]
            errs = dict(center_base=max(abs(e["center_base"][i] - x["center_base"][i]) for i in range(2)),
                        beta_deg=abs(e["beta_deg"] - x["beta_deg"]), beta_view_deg=abs(e["beta_view_deg"] - x["beta_view_deg"]),
                        turn_deg=abs(e["turn_deg"] - x["turn_deg"]), R_look=abs(e["R_look"] - x["R_look"]),
                        align_target=abs(e["align_target"] - x["align_target"]))
            tol = dict(center_base=1e-3, beta_deg=0.05, beta_view_deg=0.05, turn_deg=0.05, R_look=1e-3, align_target=1e-3)
            ok = all(errs[k] <= tol[k] for k in tol) and e["method"] == x["method"] and e["turned"] == x["turned"]
            # mock 빔 계산: 이 정렬 뒤 상자 중심 (base)
            al = None
            for i, s in enumerate(out["align_sim"]):
                if i not in used and abs(s["target"] - e["align_target"]) < 2e-4 and s["place"] == m["place"]:
                    al = s
                    used.add(i)
                    break
            geo = "정렬 기록 없음"
            geo_ok = False
            if al and "center_base_after" in al:
                ca = al["center_base_after"]
                lat_tol = 1e-3 if x["turned"] else 7e-3
                geo_ok = abs(ca[0] - x["R_look"]) <= 1e-3 and abs(ca[1]) <= lat_tol
                geo = f"접근 뒤 중심 base ({ca[0]:.4f}, {ca[1]:.4f}), R_look {x['R_look']:.4f}, 이동 {al['moved']:+.4f} m"
            # 놓기 보기 box_look
            pl = [f for f in looks if len(f) == 7 and abs(float(f[5]) - PLACE_LOOK[0]) < 1e-6 and abs(float(f[6]) - PLACE_LOOK[1]) < 1e-6
                  and math.hypot(float(f[2]) - m["map_center"][0], float(f[3]) - m["map_center"][1]) < 1e-3]
            places.append(dict(place=m["place"], expect={k: round(v, 4) if isinstance(v, float) else v for k, v in x.items()},
                               logged={k: e[k] for k in ("beta_deg", "beta_view_deg", "turn_deg", "R_look", "align_target", "method")},
                               max_err=errs, geometry=geo, place_look=[":".join(f) for f in pl]))
            ck(f"{m['place']} 공식 (beta · 회전 · R_look · 목표 · 방법)", ok,
               f"beta {x['beta_deg']:.2f}° → 보기 {x['beta_view_deg']:.2f}°, 회전 {x['turn_deg']:.2f}°, R_look {x['R_look']:.4f}, "
               f"목표 {x['align_target']:.4f}, {x['method']} (기록 {e['method']}, 최대 차 {max(errs.values()):.2e})")
            ck(f"{m['place']} mock 빔: 중심이 축 위 R_look", geo_ok, geo)
            ck(f"{m['place']} 놓기 box_look 0.44/50 · 잰 중심 조준", len(pl) == 1, str([":".join(f) for f in pl]))
        ck("제자리 회전 (spin) 수 = 회전한 장소 수", len(spins) == n_turned, f"spin {len(spins)}, 회전 {n_turned}")
        nfb = sum(1 for e in obl if e.get("method") == "tfmini_wide")
        ck("tfmini_wide 경고 줄 수 = fallback 결정 수", mlog.count("odom 직진 없음(#225)") == nfb, f"{mlog.count('odom 직진 없음(#225)')} vs {nfb}")
        # verify
        vev = [e for e in obl if e.get("event") == "verify_approach"]
        n_view = sum(1 for s in steps if s.startswith("approach_view"))
        ck("verify 앞 approach_view 수 = verify_approach 줄 수 (approach_box 0.13 없음)",
           len(vev) == n_view and not any(s == "approach_box 0.13" and steps[i + 1].startswith("verify")
                                          for i, s in enumerate(steps[:-1])), f"{len(vev)} vs {n_view}")
        for e in vev:
            al = None
            for i, s in enumerate(out["align_sim"]):
                if i not in used and abs(s["target"] - e["align_target"]) < 2e-4 and s["place"] == e["place"]:
                    al = s
                    used.add(i)
                    break
            if al is None:
                ck(f"verify {e['place']} 접근 목표", False, "정렬 기록 없음")
                continue
            b = fold45(math.radians(a.rot) - al["before"][2])
            t = tf_target(look_range(b), b)
            meth = "tfmini" if abs(math.degrees(b)) <= TF_MAX_DEG else "tfmini_wide"
            ck(f"verify {e['place']} 접근 목표 (잰 요, 보기 자리)", abs(e["align_target"] - t) < 1e-3 and e["method"] == meth and e["beta_src"] == "arm",
               f"beta {math.degrees(b):.2f}° 목표 {t:.4f} {meth} (기록 {e['beta_deg']:.2f}° {e['align_target']:.4f} {e['method']} {e['beta_src']})")
        vl = [f for f in looks if len(f) == 7 and abs(float(f[5]) - TAG_LOOK[0]) < 1e-6 and abs(float(f[6]) - TAG_LOOK[1]) < 1e-6]
        dx, dy = (float(v) for v in a.shift.split(","))
        aims_ok = all(any(math.hypot(float(f[2]) - (m["map_center"][0]), float(f[3]) - m["map_center"][1]) < 1e-3 for m in mf) for f in vl)
        ck("verify box_look 0.40/55 = verify 수, 잰 중심 조준", len(vl) == len(verifies) and aims_ok, str([":".join(f) for f in vl]))
        ck("5 칸 box_look 없음", not any(len(f) == 5 for f in looks), str([":".join(f) for f in looks if len(f) == 5]))
    else:
        ck("measure_front 없음 · oblique 줄 없음", not out.get("measure_front") and not obl, f"{len(out.get('measure_front', []))} / {len(obl)}")
        ck("box_look 은 5 칸만", all(len(f) == 5 for f in looks), str([":".join(f) for f in looks]))
        if a.base:
            b = json.load(open(f"{a.base}/mock_out.json", encoding="utf-8"))
            for k in ("steps_seen", "navigate", "align", "drive_skill", "pose", "skill", "set_mode", "check_dice", "place_box", "nav_sim", "align_sim"):
                ck(f"기준과 같음: {k}", out.get(k) == b.get(k), f"{len(out.get(k) or [])} vs {len(b.get(k) or [])}")
    json.dump(dict(run=a.run, oblique=a.oblique, rot=a.rot, shift=a.shift, checks=checks, places=places), open(f"{a.run}/check.json", "w", encoding="utf-8"),
              ensure_ascii=False, indent=1)
    for c in checks:
        print(("OK  " if c["ok"] else "BAD ") + c["check"] + (f" — {c['detail']}" if c["detail"] else ""))
    nbad = sum(not c["ok"] for c in checks)
    print(f"== {len(checks) - nbad}/{len(checks)} 통과")
    sys.exit(1 if nbad else 0)


if __name__ == "__main__":
    main()
