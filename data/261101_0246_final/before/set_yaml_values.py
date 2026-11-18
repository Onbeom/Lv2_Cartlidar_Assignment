"""설정 사본의 값만 줄 단위로 바꾼다(주석 · 순서 그대로). run_replay.sh 의 REPLAY_SET 이 부른다.

실행 (PC): python3 -I set_yaml_values.py <yaml> <절>.<이름>=<값> [...]
  예: python3 -I set_yaml_values.py out/config/perception.yaml signal.v_min=150 signal.s_min=75
  <절> 줄(예: '    signal:') 아래, 그보다 깊이 들여 쓴 줄 중 첫 '<이름>:' 줄의 값을 바꾼다. 못 찾으면 실패(종료 코드 1).
"""
import re
import sys


def set_value(lines, key, val):
    sec, name = key.split('.', 1)
    start = next((i for i, l in enumerate(lines) if re.match(rf'^\s*{re.escape(sec)}:(\s|$)', l)), None)
    if start is None:
        raise KeyError(f'{sec}: 절 없음')
    ind = len(lines[start]) - len(lines[start].lstrip())
    for i in range(start + 1, len(lines)):
        l = lines[i]
        depth = len(l) - len(l.lstrip())
        if l.strip() and not l.lstrip().startswith('#') and depth <= ind:
            break
        m = re.match(rf'^(\s+{re.escape(name)}:\s*)(\S+)(.*)$', l)
        if m:
            lines[i] = m.group(1) + val + m.group(3)
            return m.group(2)
    raise KeyError(f'{key} 없음')


def main():
    path, sets = sys.argv[1], sys.argv[2:]
    lines = open(path).read().split('\n')
    for kv in sets:
        key, val = kv.split('=', 1)
        try:
            old = set_value(lines, key, val)
        except KeyError as e:
            print(e)
            sys.exit(1)
        print(f'{key}: {old} → {val}')
    open(path, 'w').write('\n'.join(lines))


if __name__ == '__main__':
    main()
