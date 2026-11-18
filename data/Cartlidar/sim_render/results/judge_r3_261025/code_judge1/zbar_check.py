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


def read(gray):
    g = np.ascontiguousarray(gray)
    img = Z.zbar_image_create()
    Z.zbar_image_set_format(img, Y800)
    Z.zbar_image_set_size(img, g.shape[1], g.shape[0])
    Z.zbar_image_set_data(img, g.ctypes.data, g.size, None)
    Z.zbar_scan_image(sc, img)
    out, s = [], Z.zbar_image_first_symbol(img)
    while s:
        out.append(Z.zbar_symbol_get_data(s).decode())
        s = Z.zbar_symbol_next(s)
    Z.zbar_image_destroy(img)
    return out


root = Path(sys.argv[1])
rep = json.loads((root / "render_r3.json").read_text())
scales = [1, 2, 3]
tab = collections.defaultdict(lambda: collections.Counter())
rows = []
for p in rep["poses"]:
    if p["kind"] != "arm":
        continue
    g = cv2.cvtColor(cv2.imread(str(root / p["images"]["color"])), cv2.COLOR_BGR2GRAY)
    res = {}
    for s in scales:
        gg = g if s == 1 else cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_CUBIC)
        r = read(gg)
        res[s] = "ok" if r == [p["text"]] else ("none" if not r else ("wrong" if p["text"] not in r else "multi"))
    key = (p["dist_nominal"], p["tilt_deg"])
    for s in scales:
        tab[key][(s, res[s])] += 1
    rows.append((p["name"], p["module_px"], p["label_visible"], res))
print("거리 기울기 | " + " | ".join(f"x{s} ok/none/wrong" for s in scales))
for key in sorted(tab):
    c = tab[key]
    print(key, " | ".join(f"{c[(s, 'ok')]}/{c[(s, 'none')]}/{c[(s, 'wrong')] + c[(s, 'multi')]}" for s in scales))
for n, mp, v, r in rows:
    if r[1] != "ok" and n.split("_a_")[1][:7] in ("d06_t00", "d06_t20", "d08_t00", "d08_t20"):
        print("판정 범위 못 읽음", n, mp, v, r)
