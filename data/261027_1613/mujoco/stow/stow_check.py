import os, sys, math, json; os.environ.setdefault('MUJOCO_GL','egl')
sys.argv=[sys.argv[0], sys.argv[1], '/dev/null']
src_search=open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'stow_search.py')).read().split('t0=time.time()')[0]
exec(src_search)   # eval_pose 정의 (격자 검색은 실행 안 함)
src_r=open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'stow_render.py')).read()
src_r=src_r.split('res=json.load')[0].replace("m=mujoco.MjModel.from_xml_path","m2=mujoco.MjModel.from_xml_path")
# 렌더용 모델은 따로(그룹 바꿈)
ns={}; exec(src_r.replace("m2=","m=").replace("import mujoco, numpy as np, math, json, sys","import mujoco, numpy as np, math, json, sys"),ns)
cands=[(0.0,1.4,-1.0,-1.3,0.0),(math.pi-0.05,1.0,-0.8,-1.4,0.0),(-(math.pi-0.05),1.0,-0.8,-1.4,0.0),(math.pi-0.05,0.6,-0.5,-1.7,0.0),(math.pi-0.05,0.9,-0.7,-1.6,0.0)]
for q in cands:
    out,clear,occl,tau,zc,zmax,zmin=eval_pose(q)
    print([round(v,3) for v in q],'footprint_out_mm',round(out*1000,1),'clear_mm',round(min(clear,1)*1000,1),'tau',[round(t,3) for t in tau],'zc',round(zc,3),'zmax',round(zmax,3),'front',round(ns['frac'](q,'top_camera_front_color_optical_frame'),4),'rear',round(ns['frac'](q,'top_camera_rear_color_optical_frame'),4))
