import sys, yaml, copy
src = yaml.safe_load(open('/home/pa4/git/lv2_next_module/261031_1325_nav2/config/nav2_params_v40.yaml'))
class D(yaml.SafeDumper):
    def ignore_aliases(self, data): return True
V = {
 'cp5':  {'g.cost_penalty': 5.0},
 'cp10': {'g.cost_penalty': 10.0},
 'cs3':  {'i.cost_scaling_factor': 3.0},
 'cs3cp5': {'i.cost_scaling_factor': 3.0, 'g.cost_penalty': 5.0},
}
for name, ch in V.items():
    d = copy.deepcopy(src)
    for k, v in ch.items():
        w, key = k.split('.')
        t = d['planner_server']['ros__parameters']['GridBased'] if w == 'g' else d['global_costmap']['global_costmap']['ros__parameters']['inflation_layer']
        t[key] = v
    open(f'params_v40_{name}.yaml', 'w').write('# 도구 확인 사본(v40 기반, 판정 아님)\n' + yaml.dump(d, Dumper=D, allow_unicode=True, sort_keys=False))
    print(name, ch)
