"""재생용 params: 라이브 AMCL 비교 사본(config/nav2_params_v45_B_amcl.yaml)의 amcl 절 + map_server(costmap 지도 0.01, frame mission_map)에 덮어쓸 값을 얹음.
사용: python3 mkparams.py <out.yaml> [키=값 ...]  (값은 yaml 로 읽음)"""
import sys

import yaml

B = '/home/pa4/git/lv2_next_module/261102_1646_nav2_B/config'
a = yaml.safe_load(open(f'{B}/nav2_params_v45_B_amcl.yaml'))['amcl']['ros__parameters']
for kv in sys.argv[2:]:
    k, v = kv.split('=', 1)
    if '.' in k:                       # initial_pose.x=0.7998 꼴
        k, sub = k.split('.', 1)
        a[k] = dict(a[k], **{sub: yaml.safe_load(v)})
    else:
        a[k] = yaml.safe_load(v)
d = {'amcl': {'ros__parameters': a},
     'map_server': {'ros__parameters': {'yaml_filename': f'{B}/map_mission_nav_001.yaml', 'frame_id': 'mission_map', 'topic_name': 'map'}}}
open(sys.argv[1], 'w').write(yaml.safe_dump(d, sort_keys=False))
