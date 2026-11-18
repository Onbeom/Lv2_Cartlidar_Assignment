import sys, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
def scans(p, topic):
    r=rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=p,storage_id='mcap'),rosbag2_py.ConverterOptions('cdr','cdr'))
    meta={v.name:v for v in r.get_all_topics_and_types()}; r.set_filter(rosbag2_py.StorageFilter(topics=[topic]))
    out=[]
    while r.has_next():
        t,d,s=r.read_next(); m=deserialize_message(d,get_message(meta[t].type))
        out.append((m.header.stamp.sec+m.header.stamp.nanosec*1e-9, np.array(m.ranges)))
    return out
R='/home/pa4/git/lv2_next_module/261030_1943/results/'
for topic in ['/scan_front','/scan_rear']:
    a=scans(R+'n261028_bright_261030_211830/bag',topic); b=scans(R+'n261028_dark_261030_213001/bag',topic)
    print(topic,len(a),len(b))
    ta=np.array([x[0] for x in a]); tb=np.array([x[0] for x in b])
    n=min(len(a),len(b)); print(' stamp diff max', np.abs(ta[:n]-tb[:n]).max())
    diffs=[]; ident=0
    for i in range(n):
        ra,rb=a[i][1],b[i][1]; f=np.isfinite(ra)&np.isfinite(rb)
        d=np.abs(ra[f]-rb[f]); diffs.append(d.max() if len(d) else 0); ident+= np.array_equal(np.nan_to_num(ra,posinf=-1),np.nan_to_num(rb,posinf=-1))
        if i in (100,1000,3000): print(' i',i,'finite mismatch',(np.isfinite(ra)!=np.isfinite(rb)).sum(),'max diff',d.max() if len(d) else 0, 'median', np.median(d) if len(d) else 0)
    diffs=np.array(diffs); print(' identical scans',ident,'/',n,' max abs diff per scan: median',np.median(diffs),'p95',np.percentile(diffs,95),'max',diffs.max())

print('--- 다른 스캔 구간')
for topic in ['/scan_front','/scan_rear']:
    a=scans(R+'n261028_bright_261030_211830/bag',topic); b=scans(R+'n261028_dark_261030_213001/bag',topic)
    bad=[]; nb=[]
    for i in range(len(a)):
        ra,rb=np.nan_to_num(a[i][1],posinf=-1),np.nan_to_num(b[i][1],posinf=-1)
        k=(ra!=rb).sum()
        if k: bad.append(i); nb.append(k)
    bad=np.array(bad); nb=np.array(nb)
    # 연속 구간
    segs=[]; s=bad[0]; p=bad[0]
    for x in bad[1:]:
        if x!=p+1: segs.append((s,p)); s=x
        p=x
    segs.append((s,p))
    print(topic,'구간 수',len(segs),'처음 10',[(round(a[s][0],2),round(a[e][0],2),e-s+1) for s,e in segs[:10]])
    print(' 다른 칸 수 median',np.median(nb),'max',nb.max(),'첫 다른 시각',round(a[bad[0]][0],2))
