"""R3 렌더(팔 장면)를 zbar(ctypes)로 읽어 보기 — 확대 배율별. 사용: python3 -I zbar_check.py <render_r3 폴더>"""
import collections
import ctypes as C
import json
import sys
from pathlib import Path

import cv2
import numpy as np

Z = C.CDLL("libzbar.so.0")
Z.zbar_image_scanner_create.restype = C.c_void_p
Z.zbar_image_create.restype = C.c_void_p
Z.zbar_image_first_symbol.restype = C.c_void_p
Z.zbar_image_first_symbol.argtypes = [C.c_void_p]
Z.zbar_symbol_next.restype = C.c_void_p
Z.zbar_symbol_next.argtypes = [C.c_void_p]
Z.zbar_symbol_get_data.restype = C.c_char_p
Z.zbar_symbol_get_data.argtypes = [C.c_void_p]
Z.zbar_symbol_get_type.argtypes = [C.c_void_p]
Z.zbar_symbol_get_quality.argtypes = [C.c_void_p]
Z.zbar_image_scanner_set_config.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int]
Z.zbar_image_set_format.argtypes = [C.c_void_p, C.c_ulong]
Z.zbar_image_set_size.argtypes = [C.c_void_p, C.c_uint, C.c_uint]
Z.zbar_image_set_data.argtypes = [C.c_void_p, C.c_void_p, C.c_ulong, C.c_void_p]
Z.zbar_scan_image.argtypes = [C.c_void_p, C.c_void_p]
Z.zbar_image_destroy.argtypes = [C.c_void_p]
CODE128 = 128
sc = Z.zbar_image_scanner_create()
Z.zbar_image_scanner_set_config(sc, 0, 0, 0)          # 모두 끔
Z.zbar_image_scanner_set_config(sc, CODE128, 0, 1)    # CODE128 켬
Z.zbar_image_scanner_set_config(sc, 0, 0x100, 1)      # X_DENSITY 1 (모든 열)
Z.zbar_image_scanner_set_config(sc, 0, 0x101, 1)      # Y_DENSITY 1 (모든 행)
Y800 = int.from_bytes(b"Y800", "little")
# 44차(2026-10-26, 640×480 가까이서 읽기): 읽기 전에 감마 2 — 막대가 1.5~1.8 화소면 좁은 흰 칸이 흐려 회색이 되는데,
# 밝은 쪽 대비를 늘려 흰 칸을 살림(조명 1 에서 못 읽던 2장이 읽힘). 설계 자료 5벌에서 감마 1.5~2.5 모두 같은 판정 결과.
GAMMA = 2.0
# zbar quality = 그 문자열을 읽은 스캔 수. 체크섬이 상쇄되는 잘못 읽기(P2-0x → PF-&x)는 몇 스캔에서만 나옴
# (감마 2 에서 최대 2, 판정 범위 맞는 문자열 최소 34) → 이보다 적게 읽힌 문자열은 버림
QMIN = 4
LUT = (255 * (np.arange(256) / 255.0) ** GAMMA).round().astype(np.uint8)


def read(gray):
    g = np.ascontiguousarray(gray)
    img = Z.zbar_image_create()
    Z.zbar_image_set_format(img, Y800)
    Z.zbar_image_set_size(img, g.shape[1], g.shape[0])
    Z.zbar_image_set_data(img, g.ctypes.data, g.size, None)
    Z.zbar_scan_image(sc, img)
    out, s = [], Z.zbar_image_first_symbol(img)
    while s:
        if Z.zbar_symbol_get_quality(s) >= QMIN:
            out.append(Z.zbar_symbol_get_data(s).decode())
        s = Z.zbar_symbol_next(s)
    Z.zbar_image_destroy(img)
    return out


def decide(syms):
    """한 장에서 낸 문자열 결정: 서로 다른 문자열이 둘 이상이면 어느 것이 맞는지 모르므로 '못 읽음'(None)."""
    u = set(syms)
    return u.pop() if len(u) == 1 else None


root = Path(sys.argv[1])
rep = json.loads((root / "render_r3.json").read_text())
scales = [1, 2, 3]
tab = collections.defaultdict(lambda: collections.Counter())
rows = []
raw = {}   # 원본 배율에서 맞지 않는 문자열을 낸 장면: zbar 가 낸 것 전부
JD, JT = 0.05, 20   # 판정 범위(기준 R3 변경 4): 0.05 m 이하 · 20° 이하
# 실제 팔 자세 렌더(2026-10-27 밤, 사용자 결정 1): 거리는 d_min + extra(0 · 0.01 · 0.02 m) — extra 가 있는 렌더는 거리 조건 대신
# extra ≤ 0.02 m(세 단계 모두)를 판정 범위로 본다. 기울기 ≤ 20° · 보임 ≥ 0.95 · 틀린 문자열 0 은 같음
REAL = any("extra_m" in p for p in rep["poses"] if p["kind"] == "arm")
for p in rep["poses"]:
    if p["kind"] != "arm":
        continue
    g = LUT[cv2.cvtColor(cv2.imread(str(root / p["images"]["color"])), cv2.COLOR_BGR2GRAY)]
    res = {}
    for s in scales:
        gg = g if s == 1 else cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
        syms = read(gg)
        r = decide(syms)
        res[s] = "none" if r is None else ("ok" if r == p["text"] else "wrong")
        if s == 1 and syms and r != p["text"]:
            raw[p["name"]] = syms
    key = (("e", p["extra_m"]) if REAL else p["dist_nominal"], p["tilt_deg"])
    for s in scales:
        tab[key][(s, res[s])] += 1
    rows.append((p["name"], p["module_px"], p["label_visible"], res, p["extra_m"] if REAL else p["dist_nominal"], p["tilt_deg"]))
    if REAL:
        JD = 0.04 if rep.get("mission") else 0.02
        JT = 15 if rep.get("mission") else 20
print("실제 팔 자세 렌더: 거리 = d_min + extra" if REAL else "자유 카메라 렌더")
print("거리(또는 extra) 기울기 | " + " | ".join(f"x{s} ok/none/wrong" for s in scales))
for key in sorted(tab):
    c = tab[key]
    print(key, " | ".join(f"{c[(s, 'ok')]}/{c[(s, 'none')]}/{c[(s, 'wrong')] + c[(s, 'multi')]}" for s in scales))
# 판정(기준 R3 변경 4): 0.05 m 이하 · 20° 이하 · 라벨 보임 ≥ 0.95 장면 원본 배율 100 %, 모든 팔 장면 틀린 문자열 0
J = [r for r in rows if r[4] <= JD + 1e-9 and r[5] <= JT and r[2] >= 0.95]
print(f"판정: 대상 {len(J)}장 읽음 {sum(r[3][1] == 'ok' for r in J)}/{len(J)}, 틀린 문자열(전 장면 원본 배율) {sum(r[3][1] == 'wrong' for r in rows)}, "
      f"판정 범위 중 보임 < 0.95 로 뺀 장면 {sum(r[4] <= JD + 1e-9 and r[5] <= JT and r[2] < 0.95 for r in rows)}")
for n, mp, v, r, d, t in rows:
    if r[1] != "ok" and d <= JD + 1e-9 and t <= JT:
        print("판정 범위 못 읽음", n, mp, v, r)
for n, syms in raw.items():
    print("맞지 않는 문자열", n, syms)

summary = {"mission": rep.get("mission", False), "frames": len(rows), "eligible": len(J),
           "correct": sum(r[3][1] == "ok" for r in J),
           "wrong_strings": sum(r[3][1] == "wrong" for r in rows),
           "excluded_visibility": sum(r[2] < 0.95 for r in rows),
           "unreachable": rep.get("unreachable", []),
           "pass_visible": bool(J) and all(r[3][1] == "ok" for r in J) and not any(r[3][1] == "wrong" for r in rows)}
(root / "zbar_summary.json").write_text(json.dumps(summary, indent=1, ensure_ascii=False))
(root / "zbar_rows.json").write_text(json.dumps(rows, indent=1, ensure_ascii=False))
