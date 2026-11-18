"""bringup.launch.py 의 mission_node 파라미터 · 계획기 launch 인자만 계산 (노드 실행 없음). 사용: bringup_check.py <launch 파일> [이름:=값 ...]"""
import importlib.util, sys
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def text(x):
    from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
    from launch.substitution import Substitution
    if isinstance(x, ParameterFile):
        return 'FILE ' + text(x.param_file)
    if isinstance(x, ParameterValue):
        return text(x.value)
    if isinstance(x, Substitution):
        return x.describe()
    if isinstance(x, (list, tuple)):
        return ' '.join(text(v) for v in x)
    if isinstance(x, dict):
        return '{' + ', '.join(text(k) + ': ' + text(v) for k, v in x.items()) + '}'
    return str(x)

spec = importlib.util.spec_from_file_location('bl', sys.argv[1]); m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
ld = m.generate_launch_description()
ctx = LaunchContext()
given = dict(a.split(':=', 1) for a in sys.argv[2:])
for e in ld.entities:   # 선언 순서대로 기본값 (앞 인자를 참조하는 기본값 때문에)
    if isinstance(e, DeclareLaunchArgument):
        ctx.launch_configurations[e.name] = given.get(e.name, ''.join(s.perform(ctx) for s in e.default_value))
print('인자:', ' '.join(f'{k}:={v}' for k, v in given.items()) or '(기본)')
for e in ld.entities:
    if not isinstance(e, OpaqueFunction):
        continue
    try:
        out = e._OpaqueFunction__function(ctx)
    except Exception as ex:
        print('  오류:', ex); continue
    for c in out:
        for n in getattr(c, '_ComposableNodeContainer__composable_node_descriptions', []) or []:
            if 'mission' in ''.join(getattr(s, 'text', '') for s in n.node_name):
                for p in n.parameters:
                    if isinstance(p, dict):
                        print('  mission_node', text(p))
        la = getattr(c, '_IncludeLaunchDescription__launch_arguments', None)
        if la is not None:
            print('  planner', {k: text(v) for k, v in la if k in ('pucks_file', 'log_dir')})
