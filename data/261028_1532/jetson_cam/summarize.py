"""run_cam_test.sh 결과 폴더를 판정 기준(K1~K7, 판정_카메라3대.md)으로 요약. K1~K5 는 토픽마다 reliable(topics.json) · best effort(topics_be.json) 중 Hz 가 높은 쪽.
실행: python3 summarize.py <결과 폴더>
"""
import json
import re
import sys
from pathlib import Path

d = Path(sys.argv[1])
T = json.loads((d / 'topics.json').read_text())['topics']
B = json.loads((d / 'topics_be.json').read_text())['topics'] if (d / 'topics_be.json').exists() else {}
# 발행자 QoS 를 모르니(reliable 이 아니면 reliable 구독은 못 붙음) 토픽마다 Hz 가 높은 쪽 측정을 판정에 씀
T = {t: (v if v.get('hz', 0) >= B.get(t, {}).get('hz', 0) else {**B[t], 'from': 'be'}) for t, v in T.items()}
RS = ['/top_camera_front/color/image_raw', '/top_camera_front/aligned_depth_to_color/image_raw',
      '/top_camera_rear/color/image_raw', '/top_camera_rear/aligned_depth_to_color/image_raw']
ARM = '/arm_camera/image_raw'
INFOS = ['/top_camera_front/color/camera_info', '/top_camera_rear/color/camera_info', '/arm_camera/camera_info']
FRAME = {RS[0]: 'top_camera_front_color_optical_frame', RS[1]: 'top_camera_front_color_optical_frame',
         RS[2]: 'top_camera_rear_color_optical_frame', RS[3]: 'top_camera_rear_color_optical_frame',
         ARM: 'camera_optical_frame'}

cpu, ram, gpu = [], [], []
for line in (d / 'tegrastats.log').read_text().splitlines():
    m = re.search(r'RAM (\d+)/(\d+)MB', line)
    c = re.search(r'CPU \[([^\]]+)\]', line)
    g = re.search(r'GR3D_FREQ (\d+)%', line)
    if m:
        ram.append(int(m.group(1)))
    if c:
        v = [int(x.split('%')[0]) for x in c.group(1).split(',') if '%' in x]
        if v:
            cpu.append(sum(v) / len(v))
    if g:
        gpu.append(int(g.group(1)))
# 측정 구간(시작 15 초 뒤)만: 앞 15 줄은 켜지는 중
cpu_m, ram_m, gpu_m = cpu[15:] or cpu, ram[15:] or ram, gpu[15:] or gpu
bad = [ln for ln in (d / 'dmesg_new.txt').read_text().splitlines()
       if re.search(r'error|fail|reset|disconnect|overflow|timeout', ln, re.I)]


def hz(t):
    return T.get(t, {}).get('hz', 0.0)


rows = []
rows.append(('K1 D435 컬러 · 정렬 깊이 ≥ 14.0 Hz', ' · '.join(f'{hz(t):.1f}' for t in RS), all(hz(t) >= 14.0 for t in RS)))
rows.append(('K2 팔 영상 ≥ 15.0 Hz', f'{hz(ARM):.1f}', hz(ARM) >= 15.0))
gaps = {t: T.get(t, {}).get('max_gap_s', 99) for t in RS + [ARM]}
rows.append(('K3 영상 가장 긴 간격 ≤ 0.25 s', ' · '.join(f'{v:.2f}' for v in gaps.values()), all(v <= 0.25 for v in gaps.values())))
rows.append(('K4 camera_info 3개 받음', ' · '.join(str(T.get(t, {}).get('count', 0)) for t in INFOS), all(T.get(t, {}).get('count', 0) > 0 for t in INFOS)))
fr = {t: T.get(t, {}).get('frame_id', '') for t in FRAME}
rows.append(('K5 frame_id = URDF 이름', ' · '.join(fr.values()), all(fr[t] == FRAME[t] for t in FRAME)))
rows.append(('K6 새 dmesg USB 오류 0', str(len(bad)), not bad))
cpu_avg = sum(cpu_m) / len(cpu_m) if cpu_m else -1
ram_max = max(ram_m) if ram_m else -1
rows.append(('K7 CPU 평균 ≤ 60 % · RAM 최대 ≤ 5000 MB', f'{cpu_avg:.1f} % · {ram_max} MB', 0 <= cpu_avg <= 60 and 0 <= ram_max <= 5000))

print('| 기준 | 측정 | 판정 |\n|---|---|---|')
for name, val, ok in rows:
    print(f'| {name} | {val} | {"통과" if ok else "실패"} |')
print(f'\n전체: {"통과" if all(r[2] for r in rows) else "실패"}')
print(f'참고: GPU(GR3D) 평균 {sum(gpu_m) / len(gpu_m) if gpu_m else -1:.1f} %, 팔 영상 크기 {T.get(ARM, {}).get("bytes", 0)} B '
      f'(1280×720 rgb8 ≈ 2764800), 지연 중앙값 ' + ' · '.join(f'{T.get(t, {}).get("lag_median_s", "-")}' for t in RS + [ARM]))
print('참고(판정 밖): best effort 구독(인지 image_qos: sensor_data)으로 20 초 동안 받은 Hz — '
      + ' · '.join(f"{t.split('/')[1]}{'/depth' if 'depth' in t else ''} {B.get(t, {}).get('hz', 0)}" for t in RS + [ARM]))
print('참고(판정 밖): 정렬 전 깊이 Hz — 앞 ' + str(T.get('/top_camera_front/depth/image_rect_raw', {}).get('hz', 0))
      + ' · 뒤 ' + str(T.get('/top_camera_rear/depth/image_rect_raw', {}).get('hz', 0)))
for ln in bad[:10]:
    print('dmesg:', ln)
