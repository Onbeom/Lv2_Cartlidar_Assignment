"""고정 카메라/관절 자세의 자기형상 깊이구간 캐시. 이동 시 기존 캐시 재사용 금지."""
import json,time,os
import numpy as np
import filter_check as source

out=source.BASE/os.environ.get('SELF_FILTER_CACHED','nvblox_self_filter_cached')
out.mkdir(exist_ok=False)

def build_cache(k, prepared):
    chunks=[]
    for lo,hi,eq,(x0,x1,y0,y1) in prepared:
        if x0>=x1 or y0>=y1:
            continue
        yy,xx=np.mgrid[y0:y1,x0:x1]
        ids=(yy*640+xx).ravel()
        rays=np.stack(((xx.ravel()-k['cx'])/k['fx'],(yy.ravel()-k['cy'])/k['fy'],np.ones(xx.size)),-1)
        # 기존 판정의 확장 AABB와 hull halfspace를 모두 유지한다.
        normals=np.vstack((eq[:,:3],np.eye(3),-np.eye(3)))
        offsets=np.r_[.005-eq[:,3],hi,-lo]
        lower=np.zeros(len(ids));upper=np.full(len(ids),np.inf);possible=np.ones(len(ids),bool)
        for normal,limit in zip(normals,offsets):
            a=rays@normal
            positive=a>1e-12;negative=a < -1e-12
            upper[positive]=np.minimum(upper[positive],limit/a[positive])
            lower[negative]=np.maximum(lower[negative],limit/a[negative])
            possible &= (positive|negative|(limit>=0))
        keep=possible&(upper>=lower)&(upper>0)
        if keep.any():chunks.append((ids[keep],lower[keep],upper[keep]))
    return chunks

def apply_cache(depth,chunks):
    flat=depth.ravel();mask=np.zeros(flat.size,bool)
    for ids,lo,hi in chunks:
        values=flat[ids]
        selected=np.isfinite(values)&(values>0)&(values>=lo)&(values<=hi)
        mask[ids[selected]]=True
    return mask.reshape(depth.shape)

rows=[]
for row in (v for v in source.meta['rows'] if v['visible']):
    depth=np.load(source.SOURCE/row['depth']);name=row['frame']
    t=time.perf_counter();prepared=source.prepare(row);chunks=build_cache(row['K'],prepared);build_s=time.perf_counter()-t
    timings=[]
    for _ in range(20):
        t=time.perf_counter();mask=apply_cache(depth,chunks);timings.append(time.perf_counter()-t)
    ref=np.load(source.OUT/(name+'_mask.npy'));truth=np.load(source.OUT/(name+'_truth.npy'))
    filtered=depth.copy();filtered[mask]=0
    np.save(out/(name+'_filtered_m.npy'),filtered);np.save(out/(name+'_mask.npy'),mask)
    fp=int((~truth&mask).sum());fn=int((truth&~mask).sum());recall=float((truth&mask).sum()/truth.sum()) if truth.any() else None
    result={'frame':name,'build_cache_s':build_s,'cache_chunks':len(chunks),'cache_ray_entries':sum(len(v[0]) for v in chunks),'apply_median_s':float(np.median(timings)),'apply_p95_s':float(np.percentile(timings,95)),'reference_identical':bool(np.array_equal(mask,ref)),'fp':fp,'fn':fn,'recall':recall,'non_robot_false_removal_rate':fp/max(1,int((~truth).sum())),'unchanged_elsewhere':bool(np.array_equal(filtered[~mask],depth[~mask],equal_nan=True))}
    result['quality_pass']=result['reference_identical'] and (recall is None or recall>=.95) and result['non_robot_false_removal_rate']<=.001 and result['unchanged_elsewhere']
    rows.append(result);print(json.dumps(result),flush=True)
(out/'summary.json').write_text(json.dumps({'simulation_only':True,'rows':rows,'all_quality_pass':all(v['quality_pass'] for v in rows),'cached_apply_30hz_pass':all(v['apply_p95_s']<=1/30 for v in rows),'limitations':'고정 자세 캐시의 PC 오프라인 반복 처리만 확인. 관절/카메라/K/모델 변경 시 캐시 폐기 및 재생성 필요. 재생성 지연 중 입력을 지도에 통합하지 않는 ROS 연동은 아직 미구현.'},ensure_ascii=False,indent=2))
