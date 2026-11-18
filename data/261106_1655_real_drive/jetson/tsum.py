"""tegrastats 기록 요약: CPU 코어 평균 % · GPU 평균/최대 % · RAM 최대 GB · 온도 최대 °C"""
import re, sys
cpu, gpu, ram, tmax = [], [], [], 0.0
for l in open(sys.argv[1], errors='ignore'):
    m = re.search(r'CPU \[([^\]]+)\]', l); g = re.search(r'GR3D_FREQ (\d+)%', l); r = re.search(r'RAM (\d+)/', l)
    if not (m and g and r): continue
    c = [int(x) for x in re.findall(r'(\d+)%@', m.group(1))]
    cpu.append(sum(c) / len(c)); gpu.append(int(g.group(1))); ram.append(int(r.group(1)))
    tmax = max([tmax] + [float(x) for x in re.findall(r'@(\d+(?:\.\d+)?)C/', l)])
if cpu: print(f'  부하: CPU 평균 {sum(cpu)/len(cpu):.1f} % · GPU 평균 {sum(gpu)/len(gpu):.1f} / 최대 {max(gpu)} % · RAM 최대 {max(ram)/1024:.2f} GB · 최고 {tmax:.1f} °C · 표본 {len(cpu)}')
