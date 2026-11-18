"""팔 장면마다 zbar 가 낸 문자열과 quality(그 문자열을 읽은 스캔 수). 사용: python3 -I zq.py <렌더>..."""
import ctypes as C, json, sys
from pathlib import Path
import cv2, numpy as np
Z = C.CDLL("libzbar.so.0")
for f, r, a in [("zbar_image_scanner_create", C.c_void_p, []), ("zbar_image_create", C.c_void_p, []),
                ("zbar_image_first_symbol", C.c_void_p, [C.c_void_p]), ("zbar_symbol_next", C.c_void_p, [C.c_void_p]),
                ("zbar_symbol_get_data", C.c_char_p, [C.c_void_p]), ("zbar_symbol_get_quality", C.c_int, [C.c_void_p]),
                ("zbar_image_scanner_set_config", C.c_int, [C.c_void_p, C.c_int, C.c_int, C.c_int]),
                ("zbar_image_set_format", None, [C.c_void_p, C.c_ulong]), ("zbar_image_set_size", None, [C.c_void_p, C.c_uint, C.c_uint]),
                ("zbar_image_set_data", None, [C.c_void_p, C.c_void_p, C.c_ulong, C.c_void_p]),
                ("zbar_scan_image", C.c_int, [C.c_void_p, C.c_void_p]), ("zbar_image_destroy", None, [C.c_void_p])]:
    getattr(Z, f).restype = r; getattr(Z, f).argtypes = a
sc = Z.zbar_image_scanner_create()
for s, c, v in [(0, 0, 0), (128, 0, 1), (0, 0x100, 1), (0, 0x101, 1)]:
    Z.zbar_image_scanner_set_config(sc, s, c, v)
def read(g):
    g = np.ascontiguousarray(g); img = Z.zbar_image_create()
    Z.zbar_image_set_format(img, int.from_bytes(b"Y800", "little")); Z.zbar_image_set_size(img, g.shape[1], g.shape[0])
    Z.zbar_image_set_data(img, g.ctypes.data, g.size, None); Z.zbar_scan_image(sc, img)
    out, s = [], Z.zbar_image_first_symbol(img)
    while s:
        out.append((Z.zbar_symbol_get_data(s).decode(), Z.zbar_symbol_get_quality(s))); s = Z.zbar_symbol_next(s)
    Z.zbar_image_destroy(img); return out
rows = []
for root in map(Path, sys.argv[1:]):
    rep = json.loads((root / "render_r3.json").read_text())
    for p in rep["poses"]:
        if p["kind"] != "arm": continue
        g = cv2.cvtColor(cv2.imread(str(root / p["images"]["color"])), cv2.COLOR_BGR2GRAY)
        g2 = cv2.resize(g, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        rows.append(dict(name=p["name"], text=p["text"], d=p["dist_nominal"], t=p["tilt_deg"], vis=p["label_visible"], mpx=p["module_px"], syms=read(g), syms2=read(g2)))
json.dump(rows, open("zq_rows.json", "w"))
ok_q = [q for r in rows for s, q in r["syms"] if s == r["text"]]
bad_q = [q for r in rows for s, q in r["syms"] if s != r["text"]]
print("맞는 문자열 quality: 개수", len(ok_q), "분위 0/1/5/50 %", np.percentile(ok_q, [0, 1, 5, 50]))
print("틀린 문자열 quality:", sorted(bad_q))
for r in rows:
    if any(s != r["text"] for s, _ in r["syms"]): print(r["name"], r["text"], r["syms"], "mpx", r["mpx"])
