"""보고서 8~15절(P7 기준 이후 주행 방식 시험) 표를 판정 파일(verdict_stow.json)에서 채운다. build_report.py가 부른다.

대상: 검증계획 16~24장의 V1~V3(팔 접은 자세), V1b, V4·V5, V6·V6b·V7, V6c·V6cb·V7c, V8, V9.
MuJoCo 결과는 drive/results/<실행 폴더>/<이름>/, Isaac 결과는 phys_tests/results/<시험>_v12_<이름>_<시각>/에서 읽는다.
"""
import base64
import glob
import json
from pathlib import Path

RUNS = {"stow": "stow_261023_1943", "v1b": "v1b_261024_1955", "v4v5": "v4v5_261024_2023", "v6": "v6_261024_2047",
        "v6b": "v6_261024_2052", "v7": "v7_261024_2114", "v6c": "v6c_261025_0055", "v8": "v8_261025_0132", "v9": "v9_261025_0138"}


def _load(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


class Src:
    def __init__(self, drive_results, isaac_results):
        self.D, self.I = Path(drive_results), Path(isaac_results)

    def mj(self, run, name):
        return _load(self.D / RUNS[run] / name / "verdict_stow.json")

    def isc(self, prefix):
        """Isaac 결과 폴더 `<prefix>_<YYMMDD_HHMMSS>`(가장 최근)의 판정."""
        hits = sorted(glob.glob(str(self.I / f"{prefix}_2610*")))
        if not hits:
            raise FileNotFoundError(prefix)
        return _load(Path(hits[-1]) / "verdict_stow.json")


def f(v, d=1, sign=False):
    if v is None:
        return "—"
    return f"{v:+.{d}f}" if sign else f"{v:.{d}f}"


def vcell(d):
    ok = d["verdict"] == "성공"
    return f'<td class="{"ok" if ok else "ng"}">{d["verdict"]}</td>'


def tr(label, *cells):
    """첫 칸은 이름(왼쪽 정렬), 나머지는 숫자 칸. '<td'로 시작하는 칸은 그대로 넣는다."""
    return "<tr><td>" + label + "</td>" + "".join(c if c.startswith("<td") else f'<td class="n">{c}</td>' for c in cells) + "</tr>"


def build(drive_results, isaac_results, stow_png):
    s = Src(drive_results, isaac_results)
    out = {}

    # ---- 11. V1~V3 (팔 접은 자세 S0) ----
    m1, i1 = s.mj("stow", "V1_dtB"), s.isc("V1_v12")
    m2, i2 = s.mj("stow", "V2_dtB"), s.isc("V2_v12")
    m3, i3 = s.mj("stow", "V3_dtB"), s.isc("V3_v12")
    mm, im = m1["measured"], i1["measured"]
    rows = [
        "<tr><td>V1 접은 채 제자리 회전</td><td>R_full ≤ 307 mm</td>"
        f'<td class="n">{f(mm["R_full_mm"])} mm (회전 중심 x {f(mm["pivot_mm"][0])})</td>'
        f'<td class="n">{f(im["R_full_mm"])} mm (회전 중심 x {f(im["pivot_mm"][0])})</td>{vcell(m1)}{vcell(i1)}</tr>',
        "<tr><td>V2 접은 채 직진·후진·정지</td><td>P6·P8 기준 + 팔 이탈 ≤ 0.02 rad</td>"
        f'<td class="n">{f(m2["measured"]["forward_m"], 4)} / {f(m2["measured"]["backward_m"], 4)} m, 정지 {f(m2["measured"]["stop_time_s"], 3)} s</td>'
        f'<td class="n">{f(i2["measured"]["forward_m"], 4)} / {f(i2["measured"]["backward_m"], 4)} m, 정지 {f(i2["measured"]["stop_time_s"], 3)} s</td>{vcell(m2)}{vcell(i2)}</tr>',
        "<tr><td>V3 0 자세 → S0 → 0 자세</td><td>접촉 없음, 팔 실제 속도 ≤ 4.95 rad/s, 도달 ≤ 0.05 rad</td>"
        f'<td class="n">최대 {f(max(m3["measured"]["arm_speed_max_rad_s"].values()), 2)} rad/s</td>'
        f'<td class="n">최대 {f(max(i3["measured"]["arm_speed_max_rad_s"].values()), 2)} rad/s</td>{vcell(m3)}{vcell(i3)}</tr>']
    out["V13_ROWS"] = "".join(rows)
    out["S0_IMG"] = "data:image/png;base64," + base64.b64encode(Path(stow_png).read_bytes()).decode()

    # ---- 12. V1b ----
    rows = []
    for d in (52, 66, 80):
        m, i = s.mj("v1b", f"V1b_d{d}"), s.isc(f"V1b_v12_d{d}")
        rows.append(tr(f"{m['measured']['pivot_offset_mm']:.1f} mm", f(m["measured"]["e_mm"]), f(m["measured"]["R_cell_mm"]), vcell(m),
                       f(i["measured"]["e_mm"]), f(i["measured"]["R_cell_mm"]), vcell(i)))
    out["V1B_ROWS"] = "".join(rows)

    # ---- 14. V4·V5 ----
    rows = []
    for k in "AB":
        for r in ("10", "20", "35"):
            mL, mR = s.mj("v4v5", f"V4_{k}L_r{r}"), s.mj("v4v5", f"V4_{k}R_r{r}")
            iL, iR = s.isc(f"V4_v12_{k}L_r{r}"), s.isc(f"V4_v12_{k}R_r{r}")
            ml, il = mL["measured"], iL["measured"]
            ok = all(x["verdict"] == "성공" for x in (mL, mR, iL, iR))
            rows.append(tr(f"{k} · 0.{r}", f"{f(ml['min_clearance_mm'])} / {f(mR['measured']['min_clearance_mm'])}",
                           f"{f(il['min_clearance_mm'])} / {f(iR['measured']['min_clearance_mm'])}",
                           f"{ml['arc_time_s'] / ml['arc_time_nominal_s']:.2f} · {il['arc_time_s'] / il['arc_time_nominal_s']:.2f}",
                           f"{ml['arc_radius_fit_m'] / ml['radius_cmd_m']:.2f} · {il['arc_radius_fit_m'] / il['radius_cmd_m']:.2f}",
                           f"{f(ml['arc_end_outward_mm'], 0, True)} · {f(il['arc_end_outward_mm'], 0, True)}",
                           f'<td class="{"ok" if ok else "ng"}">{"4/4 성공" if ok else "실패 있음"}</td>'))
    out["V4_ROWS"] = "".join(rows)
    rows = []
    for k in "AB":
        for eng, d in (("MuJoCo", s.mj("v4v5", f"V5_dead{k}")), ("Isaac", s.isc(f"V5_v12_dead{k}"))):
            m = d["measured"]
            rows.append(tr(f"{k} · {eng}", f"{f(m['backward_m'], 4)} ({m['cell_len_m']:.3f})", f(m["side_max_mm"], 2),
                           f(m["stop_time_s"], 3), f(m["min_clearance_mm"]), vcell(d)))
    out["V5_ROWS"] = "".join(rows)

    # ---- 15. V6 ----
    rows = []
    for eng in ("MuJoCo", "Isaac"):
        for h in ("90", "180", "m90"):
            d = s.mj("v6", f"V6_h{h}") if eng == "MuJoCo" else s.isc(f"V6_v12_h{h}")
            m = d["measured"]
            o = m["origin_after_spin_mm"]
            rows.append(tr(f"{eng} · {m['start_heading_deg']:+d}°", f(m["min_clearance_mm"]), f"({o[0]:+.0f}, {o[1]:+.0f})",
                           f(m["end_lat_mm"], 2, True), f(m["end_heading_deg"], 2, True), f(m["end_along_err_mm"], 2, True), vcell(d)))
    out["V6_ROWS"] = "".join(rows)

    # ---- 16. V6b → V6cb ----
    rows = []
    for k in "AB":
        for r in ("10", "20", "35"):
            cells = [f"{k} · 0.{r}"]
            for eng in ("MuJoCo", "Isaac"):
                for run, pre in (("v6b", "V6b"), ("v6c", "V6cb")):
                    hs = []
                    for side in "LR":
                        d = s.mj(run, f"{pre}_{k}{side}_r{r}") if eng == "MuJoCo" else s.isc(f"{pre}_v12_{k}{side}_r{r}")
                        h = d["measured"]["end_heading_deg"]
                        hs.append(f'<span class="{"ok" if d["verdict"] == "성공" else "ng"}">{abs(h):.2f}</span>')
                    cells.append(" / ".join(hs))
            rows.append(tr(*cells))
    out["V6B_ROWS"] = "".join(rows)

    # ---- 17. V7 → V7c ----
    rows = []
    for eng in ("MuJoCo", "Isaac"):
        for h in ("90", "180"):
            for e, lab in (("E1", "E1 편향"), ("E2s1", "E2 잡음 s1"), ("E2s2", "E2 잡음 s2"), ("E3", "E3 편향+잡음")):
                a = s.mj("v7", f"V7_h{h}_{e}") if eng == "MuJoCo" else s.isc(f"V7_v12_h{h}_{e}")
                b = s.mj("v6c", f"V7c_h{h}_{e}") if eng == "MuJoCo" else s.isc(f"V7c_v12_h{h}_{e}")
                ma, mb = a["measured"], b["measured"]
                rows.append(tr(f"{eng} · {h}° · {lab}", f(ma["end_heading_deg"], 2, True), vcell(a), f(mb["end_heading_deg"], 2, True),
                               f(mb["end_lat_mm"], 1, True), vcell(b)))
    out["V7_ROWS"] = "".join(rows)
    v6c_all = ([s.mj("v6c", f"V6c_h{h}") for h in ("90", "180", "m90")] + [s.isc(f"V6c_v12_h{h}") for h in ("90", "180", "m90")])
    out["N_V6C"] = str(len(v6c_all) + 24 + 16)

    # ---- 18. V8 ----
    rows = []
    for eng in ("MuJoCo", "Isaac"):
        for r, v in (("10", "10"), ("20", "10"), ("35", "10"), ("35", "15")):
            d = s.mj("v8", f"V8_AL_r{r}_v{v}") if eng == "MuJoCo" else s.isc(f"V8_v12_AL_r{r}_v{v}")
            b = s.mj("v4v5", f"V4_AL_r{r}") if eng == "MuJoCo" else s.isc(f"V4_v12_AL_r{r}")
            m, mb = d["measured"], b["measured"]
            rows.append(tr(f"{eng} · 0.{r} · 0.{v}", f"{f(mb['min_clearance_mm'])} → {f(m['min_clearance_mm'])}",
                           f"{mb['arc_time_s'] / mb['arc_time_nominal_s']:.2f} → {m['arc_time_s'] / m['arc_time_nominal_s']:.2f}",
                           f"{f(mb['arc_end_outward_mm'], 0, True)} → {f(m['arc_end_outward_mm'], 0, True)}",
                           f"{f(mb['yaw_overshoot_deg'], 2)} → {f(m['yaw_overshoot_deg'], 2)}", vcell(d)))
    out["V8_ROWS"] = "".join(rows)

    # ---- 19. V9 ----
    rows, chart = [], {}
    for eng in ("MuJoCo", "Isaac"):
        for scene, base_m, base_i, pre, key in (
                ("V4 R 0.20 최소 간격 (mm)", ("v4v5", "V4_AL_r20"), "V4_v12_AL_r20", "V4_AL_r20", "min_clearance_mm"),
                ("V4 R 0.35 최소 간격 (mm)", ("v4v5", "V4_AL_r35"), "V4_v12_AL_r35", "V4_AL_r35", "min_clearance_mm"),
                ("V4 R 0.35 호 끝 바깥 쓸림 (mm)", ("v4v5", "V4_AL_r35"), "V4_v12_AL_r35", "V4_AL_r35", "arc_end_outward_mm"),
                ("V6 180° 회전 시간 (s)", ("v6", "V6_h180"), "V6_v12_h180", "V6_h180", "spin_time_s")):
            vals = [(s.mj(*base_m) if eng == "MuJoCo" else s.isc(base_i))["measured"][key]]
            for mu in ("04", "06", "08"):
                d = s.mj("v9", f"{pre}_mu{mu}") if eng == "MuJoCo" else s.isc(f"{pre.replace('_', '_v12_', 1)}_mu{mu}")
                vals.append(d["measured"][key])
            dd = 2 if "시간" in scene else 1
            rows.append(tr(f"{eng} · {scene}", *[f(v, dd, "쓸림" in scene) for v in vals]))
            if "최소 간격" in scene:
                chart[f"{eng} {scene[3:9]}"] = vals
    out["V9_ROWS"] = "".join(rows)
    out["V9_SVG"] = svg_chart(chart)
    return out


def svg_chart(series):
    """V9 최소 간격 vs 마찰(μ 0.3·0.4·0.6·0.8) 선 그래프. 색 = 엔진(파랑 --a1 MuJoCo, 주황 --a0 Isaac), 선 모양 = 반경(실선 0.35, 점선 0.20)."""
    W, H, L, R, T, B = 560, 260, 48, 130, 12, 34
    xs = [0.3, 0.4, 0.6, 0.8]
    ymin, ymax = 0, 200
    X = lambda x: L + (W - L - R) * (x - 0.3) / 0.5
    Y = lambda y: T + (H - T - B) * (1 - (y - ymin) / (ymax - ymin))
    e = [f'<svg viewBox="0 0 {W} {H}" role="img" aria-label="마찰에 따른 모퉁이 호 회전 최소 간격">']
    for v in range(0, 201, 50):
        e.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(v):.1f}" y2="{Y(v):.1f}" stroke="var(--grid)"/>'
                 f'<text x="{L - 6}" y="{Y(v) + 4:.1f}" text-anchor="end">{v}</text>')
    for x in xs:
        e.append(f'<text x="{X(x):.1f}" y="{H - 12}" text-anchor="middle">μ {x}</text>')
    e.append(f'<line x1="{L}" x2="{W - R}" y1="{Y(30):.1f}" y2="{Y(30):.1f}" stroke="var(--bad)" stroke-dasharray="2 3"/>'
             f'<text x="{W - R + 6}" y="{Y(30) + 4:.1f}" fill="var(--bad)">기준 30 mm</text>')
    labels = []
    for name, vals in series.items():
        col = "var(--a1)" if name.startswith("MuJoCo") else "var(--a0)"
        dash = "" if "0.35" in name else ' stroke-dasharray="6 4"'
        pts = " ".join(f"{X(x):.1f},{Y(v):.1f}" for x, v in zip(xs, vals))
        e.append(f'<polyline points="{pts}" fill="none" stroke="{col}" stroke-width="2"{dash}/>')
        for x, v in zip(xs, vals):
            e.append(f'<circle cx="{X(x):.1f}" cy="{Y(v):.1f}" r="3" fill="{col}"><title>{name} μ {x}: {v:.1f} mm</title></circle>')
        labels.append([Y(vals[-1]) + 4, col, f"{name} {vals[-1]:.0f}"])
    labels.sort()
    for j in range(1, len(labels)):  # 끝점 이름표가 겹치지 않게 최소 13 px 띄운다
        labels[j][0] = max(labels[j][0], labels[j - 1][0] + 13)
    for y, col, txt in labels:
        e.append(f'<text x="{W - R + 6}" y="{y:.1f}" fill="{col}">{txt}</text>')
    e.append("</svg>")
    return "".join(e)
