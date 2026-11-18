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



# 회전각은 정답/렌더 자세를 읽지 않는 고정 후보. 서로 다른 문자열은 선택하지 않고 거부.
ANGLES = [0, -5, 5, -10, 10, -15, 15, -20, 20, -25, 25, -30, 30]

def read_rotations(bgr):
    gray = LUT[cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)] if bgr.ndim == 3 else LUT[bgr]
    h,w = gray.shape
    symbols=[]; hits=[]
    for angle in ANGLES:
        if angle == 0:
            rotated=gray
        else:
            M=cv2.getRotationMatrix2D((w/2,h/2),angle,1.0)
            c,s=abs(M[0,0]),abs(M[0,1]);nw=int(np.ceil(w*c+h*s));nh=int(np.ceil(h*c+w*s))
            M[0,2]+=(nw-w)/2;M[1,2]+=(nh-h)/2
            rotated=cv2.warpAffine(gray,M,(nw,nh),flags=cv2.INTER_LINEAR,borderValue=255)
        found=read(rotated)
        symbols.extend(found)
        if found:hits.append({'angle_deg':angle,'symbols':found})
    return {'text':decide(symbols),'symbols':sorted(set(symbols)),'hits':hits}
