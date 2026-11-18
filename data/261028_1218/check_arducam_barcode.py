"""실물 Arducam 바코드 사진(MJPG)을 zbar 로 읽고, 줄여 가며 몇 px/모듈까지 읽히는지 본다 (도구 확인).
실행: ~/yolo-venv/bin/python -I check_arducam_barcode.py <P2-01 막대가 지나는 1080p 행> bc_1080.jpg bc_480.jpg ...
"""
import ctypes as C
import sys

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
Z.zbar_image_scanner_set_config.argtypes = [C.c_void_p, C.c_int, C.c_int, C.c_int]
Z.zbar_image_set_format.argtypes = [C.c_void_p, C.c_ulong]
Z.zbar_image_set_size.argtypes = [C.c_void_p, C.c_uint, C.c_uint]
Z.zbar_image_set_data.argtypes = [C.c_void_p, C.c_void_p, C.c_ulong, C.c_void_p]
Z.zbar_scan_image.argtypes = [C.c_void_p, C.c_void_p]
Z.zbar_image_destroy.argtypes = [C.c_void_p]
CODE128 = 128
sc = Z.zbar_image_scanner_create()
Z.zbar_image_scanner_set_config(sc, 0, 0, 0)
Z.zbar_image_scanner_set_config(sc, CODE128, 0, 1)
Z.zbar_image_scanner_set_config(sc, 0, 0x100, 1)
Z.zbar_image_scanner_set_config(sc, 0, 0x101, 1)


def scan(gray):
    g = np.ascontiguousarray(gray, np.uint8)
    im = Z.zbar_image_create()
    Z.zbar_image_set_format(im, 0x30303859)   # 'Y800'
    Z.zbar_image_set_size(im, g.shape[1], g.shape[0])
    Z.zbar_image_set_data(im, g.ctypes.data, g.size, None)
    Z.zbar_scan_image(sc, im)
    out, s = [], Z.zbar_image_first_symbol(im)
    while s:
        out.append(Z.zbar_symbol_get_data(s).decode())
        s = Z.zbar_symbol_next(s)
    Z.zbar_image_destroy(im)
    return sorted(set(out))


def bar_span(gray, y, x0, x1):
    """가로줄 y 에서 x0~x1 사이 첫 · 마지막 어두운 화소 (막대 폭 px)."""
    row = gray[y, x0:x1].astype(int)
    dark = np.nonzero(row < (row.max() + row.min()) / 2)[0]
    return int(dark[-1] - dark[0] + 1) if len(dark) else 0


ROW = int(sys.argv[1]); files = sys.argv[2:]
for f in files:
    g = cv2.imread(f, cv2.IMREAD_GRAYSCALE)
    print(f, g.shape, '원본 읽힘:', scan(g))
    if g.shape[1] == 1920:
        span = bar_span(g, ROW, 540, 1500)   # P2-01 막대 줄
        ppm = span / 90.0                    # 막대 구간 = 90 모듈
        print(f'  P2-01 막대 폭 {span} px → {ppm:.2f} px/모듈')
        for s in (0.8, 0.6, 0.5, 0.4, 0.35, 0.3, 0.25, 0.2, 0.15):
            small = cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_AREA)
            print(f'  축소 {s:.2f} → {ppm * s:.2f} px/모듈: {scan(small)}')
