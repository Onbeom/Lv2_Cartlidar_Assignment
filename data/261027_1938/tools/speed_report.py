"""최고 속도 시험(M5 · P5: 정지에서 바퀴 목표 = 한계의 2배, 제어기가 한계로 자름)과 직진(M6 · P6) 기록에서
차체 최고 속도, 95 %에 닿는 시간, 평균 가속, 앞뒤 기울기 변화(pitch), 바퀴 토크 최대 · 상한(1.8 N·m) 근처 비율을 뽑는다(모의).
비교용 계산: 증속 벨트가 없을 때 바퀴 한계 7.3304 rad/s → 차체 0.2419 m/s(미끄럼 없음).
사용(PC): python3 tools/speed_report.py <결과 폴더> ...
"""
import csv
import json
import sys
from pathlib import Path

import numpy as np

R_WHEEL, LIM, TAU = 0.0330006, 10.995574, 1.8


def load(folder):
    with open(Path(folder) / "log.csv") as f:
        rows = list(csv.DictReader(f))
    def num(x):
        try:
            return float(x)
        except ValueError:
            return np.nan
    cols = {k: np.array([num(r[k]) for r in rows]) for k in rows[0] if k != "segment"}
    s = json.loads((Path(folder) / "summary.json").read_text())
    return cols, s


out = {}
for folder in sys.argv[1:]:
    c, s = load(folder)
    t = c["t_s"]
    sch = s.get("schedule") or []
    t0, t1 = (sch[0][0], sch[0][1]) if sch and isinstance(sch[0], list) else (1.0, t[-1])
    dt = np.median(np.diff(t))
    k = max(1, int(round(0.1 / dt)))
    v = np.hypot(np.diff(c["base_x_m"][::k]), np.diff(c["base_y_m"][::k])) / (dt * k)
    tv = t[::k][1:]
    on = (tv > t0) & (tv <= t1)
    top = float(np.mean(v[on & (tv > t1 - 1.0)])) if on.any() else float("nan")
    reach = tv[on & (v >= 0.95 * top)]
    t95 = float(reach[0] - t0) if len(reach) else float("nan")
    pitch = np.degrees(c["pitch_rad"] - c["pitch_rad"][np.searchsorted(t, t0) - 1])
    r = {"top_speed_mps": top, "time_to_95pct_s": t95, "mean_accel_mps2": 0.95 * top / t95 if t95 == t95 and t95 > 0 else None,
         "pitch_change_max_deg": float(np.nanmax(np.abs(pitch[(t > t0)]))), "distance_m": float(np.hypot(c["base_x_m"][-1] - c["base_x_m"][0], c["base_y_m"][-1] - c["base_y_m"][0])),
         "speed_vs_limit": top / (LIM * R_WHEEL), "speed_vs_no_belt": top / (7.3304 * R_WHEEL)}
    fk = [k_ for k_ in c if k_.startswith("force_Wheel_")]
    if fk:
        F = np.abs(np.stack([c[k_] for k_ in fk]))
        r["wheel_torque_max_Nm"] = float(np.nanmax(F))
        r["wheel_torque_ge_99pct_of_1.8_frac"] = float(np.mean(F[:, t > t0] >= 0.99 * TAU))
    out[Path(folder).name] = r
    print(Path(folder).name, json.dumps({k_: (round(v_, 4) if isinstance(v_, float) else v_) for k_, v_ in r.items()}, ensure_ascii=False))
Path(sys.argv[1]).parent.joinpath("speed_report.json").write_text(json.dumps(out, indent=1))
