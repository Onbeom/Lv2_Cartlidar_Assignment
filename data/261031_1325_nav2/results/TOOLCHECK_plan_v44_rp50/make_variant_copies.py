"""도구 확인 사본(판정 아님): v43 + 호 Lattice(lattice_straight_rot_arc25_0025.json) + 회전 벌점 5 · 20 · 50."""
import yaml, copy
N = '/home/pa4/git/lv2_next_module/261031_1325_nav2/config/'
src = yaml.safe_load(open(N + 'nav2_params_v43.yaml'))
class D(yaml.SafeDumper):
    def ignore_aliases(self, data): return True
for rp in (5.0, 20.0, 50.0):
    d = copy.deepcopy(src); g = d['planner_server']['ros__parameters']['GridBased']
    g['lattice_filepath'] = N + 'lattice_straight_rot_arc25_0025.json'; g['rotation_penalty'] = rp
    open(f'/tmp/claude-1000/-home-pa4-git-lv2-next-module/81c6149e-7d5d-4398-a95e-e9da232cc62b/scratchpad/v42/params_v44_rp{int(rp)}.yaml', 'w').write('# 도구 확인 사본(v43 + 호 Lattice, 판정 아님)\n' + yaml.dump(d, Dumper=D, allow_unicode=True, sort_keys=False))
    print(rp, g['non_straight_penalty'], g['reverse_penalty'])
