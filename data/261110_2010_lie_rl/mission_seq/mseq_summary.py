"""2026-11-11 (RL 261111 1) 미션 순서 모의 A · B 결과 합치기 → M1 · M1′ · M2 · M3 · M4 · M5 (기준 `../미션순서모의_기준초안.md`).
  python3 mseq_summary.py <A json> <B json> [<A json> <B json> ...]
"""
import json
import sys

BAD = ("arm_box", "arm_nb", "nb_move", "nb_topple", "puck_topple", "floor_drop")  # M1′ 실격 · 큰 감점 사건 (상자 접촉 · 옆 퍽 · 퍽 넘어짐 · 낙하)
tot = dict(n=0, m1=0, m1p=0, m2=0, m3=0, s4ok=0, picked=0, b_n=0, m5=0, align=0, seated=0, only_s8body=0)
forces = {}
for ia in range(1, len(sys.argv), 2):
    A, B = json.load(open(sys.argv[ia])), json.load(open(sys.argv[ia + 1]))
    rows = A["rows"]
    bmap = {r["a_env"]: r for r in B["results"]}
    n = len(rows)
    c = dict(n=n, m1=0, m1p=0, m2=0, m3=0, s4ok=0, picked=0, b_n=len(bmap), m5=0, align=0, seated=0, only_s8body=0)
    cause = {}
    for r in rows:
        ev = r["events"]
        b = bmap.get(r["env"])
        c["align"] += bool(r.get("align_contact"))
        if any(set(v) & {"S1", "S2", "S3", "S5", "S6"} for v in ev.values()):
            c["m2"] += 1
        if r["s4_end"] == "ok":
            c["s4ok"] += 1
            if "S4" not in ev["arm_box"] and "S4" not in ev["puck_topple"]:
                c["m3"] += 1
        floor = bool(ev["floor_drop"]) or bool(b and b["events"]["floor_drop"])
        c["m5"] += floor
        if b is not None:
            c["picked"] += bool(b["picked"])
            c["seated"] += bool(b.get("seated"))
            if b.get("seated") and not b.get("m1"):  # 칸에 앉았는데 몸체 1 N 넘어 빠짐: 그 원인이 S8 손가락-몸체 (칸 벽) 뿐인지
                mf = b["max_force_N"]
                arm_over = any(v > 1.0 for v in mf["arm"].values()) or b["body_max_N"][0] > 1.0
                if not arm_over:
                    c["only_s8body"] += 1
            if b.get("picked") and "max_force_N" in b:
                for kk in ("arm", "finger", "puck"):
                    for ph, v in b["max_force_N"][kk].items():
                        forces.setdefault((kk, ph), []).append(v)
        if b is not None and b.get("m1"):
            c["m1"] += 1
            bad = [k for k in BAD if ev.get(k)] + [k + "(B)" for k in BAD if b["events"].get(k)]
            if not bad:
                c["m1p"] += 1
            else:
                for k in bad:
                    cause[k] = cause.get(k, 0) + 1
        elif b is None:
            k = "A:" + (r["s4_end"] or "?") + ("" if r["ok"] else " / " + r["fail"].split(" (")[0])
            cause[k] = cause.get(k, 0) + 1
        else:
            k = "B:" + b.get("stage", "?") + (" (body > 1 N)" if b.get("seated") else "")
            cause[k] = cause.get(k, 0) + 1
    p = lambda x, d: f"{x}/{d} ({100 * x / max(1, d):.1f} %)"  # noqa: E731
    print(f"== {sys.argv[ia]} (seed {A['seed']}, 옆 퍽 확률 {A['nb_prob']}, v {A['vmax']})")
    print(f"  M1 칸 넣기까지 {p(c['m1'], n)} | M1′ 실격 사건 없는 성공 {p(c['m1p'], n)} | M2 이동 단계 사건 {p(c['m2'], n)} | M3 S4 접촉 없는 끝 {p(c['m3'], n)} (노드 끝 {c['s4ok']})")
    print(f"  M4 고정 집기 {p(c['picked'], c['b_n'])} (S6 까지 끝 대 중) | M5 바닥 낙하 {c['m5']} 대 | ALIGN_A 팔-몸체 접촉 (따로) {p(c['align'], n)}")
    print(f"  칸 판정만 (seated) {p(c['seated'], n)} | 칸에 앉았지만 S8 손가락-몸체 (칸 벽) 1 N 만으로 M1 에서 빠진 대 {c['only_s8body']}")
    print(f"  M1 실패 · M1′ 감점 원인: {dict(sorted(cause.items(), key=lambda kv: -kv[1]))}")
    for k in tot:
        tot[k] += c[k]
if len(sys.argv) > 3:
    n = tot["n"]
    print(f"== 합계 {n} 대: M1 {tot['m1']} ({100 * tot['m1'] / n:.1f} %), M1′ {tot['m1p']} ({100 * tot['m1p'] / n:.1f} %), M2 {tot['m2']} ({100 * tot['m2'] / n:.2f} %), "
          f"M3 {tot['m3']} ({100 * tot['m3'] / n:.1f} %), M4 {tot['picked']}/{tot['b_n']}, M5 {tot['m5']}, ALIGN_A {tot['align']}, 칸 판정만 {tot['seated']} ({100 * tot['seated'] / n:.1f} %), S8 칸 벽 접촉만으로 빠짐 {tot['only_s8body']}")

if forces:
    print("== 몸체 접촉 힘 (집은 대, 단계별 최대 N): 1 N 넘은 대 / 중앙 / 90 % / 최대")
    for (kk, ph), v in sorted(forces.items()):
        v = sorted(v)
        print(f"  {kk:6s} {ph:8s} {sum(1 for x in v if x > 1.0)}/{len(v)} / {v[len(v)//2]:.1f} / {v[int(len(v)*0.9)]:.1f} / {v[-1]:.1f}")
