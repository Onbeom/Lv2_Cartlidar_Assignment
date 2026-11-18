"""bringup.launch.py 의 control_container 구성만 계산 (패키지 실행 없음). 사용: bringup_static.py <launch 파일> [mission_source]"""
import importlib.util, sys
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

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
args = {a.name: a for a in ld.entities if isinstance(a, DeclareLaunchArgument)}
if len(sys.argv) > 2:
    ctx.launch_configurations['mission_source'] = sys.argv[2]
else:
    ctx.launch_configurations['mission_source'] = ''.join(s.perform(ctx) for s in args['mission_source'].default_value)
print('mission_source =', ctx.launch_configurations['mission_source'])
up = ''.join(s.perform(ctx) for s in args['use_planner'].default_value)
print('use_planner 기본 =', up)
for e in ld.entities:
    if isinstance(e, OpaqueFunction):
        try:
            out = e._OpaqueFunction__function(ctx)
        except Exception as ex:
            print('OpaqueFunction 오류:', ex); continue
        for c in out:
            for n in getattr(c, '_ComposableNodeContainer__composable_node_descriptions', []) or []:
                if n.node_name and 'mission' in ''.join(getattr(s, 'text', '') for s in n.node_name):
                    print('mission_node params ='); [print('  ', text(p)) for p in n.parameters]
