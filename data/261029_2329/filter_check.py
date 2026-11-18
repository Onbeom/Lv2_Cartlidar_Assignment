"""새 모델 충돌형상 자기 필터: 전체 검색과 투영 ROI의 동등성/시간/오제거 확인."""
from pathlib import Path
import json, sys, time, os
import numpy as np
from scipy.spatial import ConvexHull

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'Cartlidar/sim_world'))
from robot_geom import Robot

BASE = ROOT / 'Cartlidar/sim_render/results/followup_261028_1457'
SOURCE = BASE / os.environ.get('SELF_FILTER_SOURCE', 'camera_new_model_spec')
OUT = BASE / os.environ.get('SELF_FILTER_REFERENCE', 'nvblox_self_filter_new_model')
OUT.mkdir(exist_ok=True)
meta = json.loads((SOURCE / 'summary.json').read_text())
urdf = Path(meta['urdf'])
started = time.perf_counter()
robot = Robot(urdf, urdf.parent.parent)
shapes = robot.world_shapes(meta['arm_q'])
load_s = time.perf_counter() - started

def prepare(row):
    """관측점과 형상을 광학좌표로 통일. ROI는 확장 AABB 투영으로 보수적으로 계산."""
    # 카메라 pose에는 월드 변환이 있으므로 URDF root->camera를 직접 계산한다.
    C = robot.link_poses(meta['arm_q'])[row['frame']]
    R, t = C[:3, :3], C[:3, 3]
    prepared = []
    k = row['K']
    for link, vertices in shapes:
        v = (vertices-t) @ R
        lo, hi = v.min(0)-.005, v.max(0)+.005
        eq = ConvexHull(v).equations
        if hi[2] <= 0:
            continue
        if lo[2] <= 0:
            bounds = (0, 640, 0, 480)
        else:
            corners = np.array([[x,y,z] for x in (lo[0],hi[0]) for y in (lo[1],hi[1]) for z in (lo[2],hi[2])])
            pixels = corners[:,:2]/corners[:,2,None]*[k['fx'],k['fy']]+[k['cx'],k['cy']]
            p0 = np.floor(pixels.min(0)).astype(int)-1
            p1 = np.ceil(pixels.max(0)).astype(int)+2
            bounds = (max(0,int(p0[0])),min(640,int(p1[0])),max(0,int(p0[1])),min(480,int(p1[1])))
        prepared.append((lo,hi,eq,bounds))
    return prepared

def filter_depth(depth, k, prepared, roi):
    v,u = np.indices(depth.shape)
    safe = np.where(np.isfinite(depth), depth, 0.)
    points = np.stack(((u-k['cx'])*safe/k['fx'],(v-k['cy'])*safe/k['fy'],safe),-1)
    valid = np.isfinite(depth)&(depth>0)
    mask = np.zeros(depth.shape,bool)
    for lo,hi,eq,bounds in prepared:
        x0,x1,y0,y1 = bounds if roi else (0,640,0,480)
        if x0>=x1 or y0>=y1:
            continue
        p = points[y0:y1,x0:x1]
        m = mask[y0:y1,x0:x1]
        keep = valid[y0:y1,x0:x1]&~m&np.all(p>=lo,axis=-1)&np.all(p<=hi,axis=-1)
        if keep.any():
            inside = np.all(p[keep]@eq[:,:3].T+eq[:,3]<=.005,axis=1)
            yy,xx = np.nonzero(keep)
            m[yy[inside],xx[inside]]=True
    return mask

def run():
    reports=[]
    for row in (v for v in meta['rows'] if v['visible']):
        name=row['frame'];depth=np.load(SOURCE/row['depth'])
        t=time.perf_counter();prepared=prepare(row);prepare_s=time.perf_counter()-t
        t=time.perf_counter();reference=filter_depth(depth,row['K'],prepared,False);reference_s=time.perf_counter()-t
        timings=[]
        for _ in range(3):
            t=time.perf_counter();mask=filter_depth(depth,row['K'],prepared,True);timings.append(time.perf_counter()-t)
        filtered=depth.copy();filtered[mask]=0
        np.save(OUT/(name+'_filtered_m.npy'),filtered)
        np.save(OUT/(name+'_mask.npy'),mask)
        # 정답은 여기부터 읽음. 필터 입력에는 로봇숨김 영상/깊이가 없다.
        hidden=next(v for v in meta['rows'] if v['frame']==name and not v['visible'])
        absent=np.load(SOURCE/hidden['depth'])
        difference=np.zeros(depth.shape,dtype=float)
        np.subtract(absent,depth,out=difference,where=np.isfinite(absent)&np.isfinite(depth))
        truth=np.isfinite(depth)&(depth>0)&(~np.isfinite(absent)|(difference>.001))
        np.save(OUT/(name+'_truth.npy'),truth)
        tp=int((truth&mask).sum());fp=int((~truth&mask).sum());fn=int((truth&~mask).sum())
        recall=tp/int(truth.sum()) if truth.any() else None
        rate=fp/max(1,int((~truth).sum()))
        unchanged=bool(np.array_equal(filtered[~mask],depth[~mask],equal_nan=True))
        result={'frame':name,'truth_robot_pixels':int(truth.sum()),'removed_pixels':int(mask.sum()),'tp':tp,'fp':fp,'fn':fn,'recall':recall,'non_robot_false_removal_rate':rate,'unchanged_elsewhere':unchanged,'reference_identical':bool(np.array_equal(mask,reference)),'prepare_s':prepare_s,'reference_s':reference_s,'roi_s':timings,'roi_median_s':float(np.median(timings))}
        result['quality_pass']=(recall is None or recall>=.95) and rate<=.001 and unchanged and result['reference_identical']
        reports.append(result);print(json.dumps(result),flush=True)
    (OUT/'summary.json').write_text(json.dumps({'simulation_only':True,'source':str(SOURCE),'load_s':load_s,'shapes':len(shapes),'margin_m':.005,'rows':reports,'all_quality_pass':all(v['quality_pass'] for v in reports),'realtime_30hz_pass':all(v['roi_median_s']<=1/30 for v in reports),'limitations':'drive 한 자세·같은 장면 6시야. 고정TF·노이즈없음. 준비시간은 관절/카메라 자세가 변하면 다시 필요.'},ensure_ascii=False,indent=2))

if __name__ == "__main__":
    run()
