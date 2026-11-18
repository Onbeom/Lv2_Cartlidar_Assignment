"""실제 nvblox 도구 출력의 거리 지도를 그린다."""
import json
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
from PIL import Image
root=Path(__file__).resolve().parents[2];base=root/'Cartlidar/sim_render/results/followup_261028_1457';font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['axes.unicode_minus']=False
p=base/'nvblox_tool';z=np.load(p/'distance_slice.npz');a=np.ma.array(z['distance'],mask=~z['known']);o=z['origin'];res=float(z['resolution']);h,w=a.shape;cmap=plt.colormaps['viridis'].copy();cmap.set_bad('lightgray')
fig,ax=plt.subplots(figsize=(9,6));im=ax.imshow(a,origin='lower',extent=[o[0],o[0]+w*res,o[1],o[1]+h*res],cmap=cmap,vmin=0,vmax=1,interpolation='nearest');ax.axvline(1,color='red',ls='--',label='합성 입력 평면 x=1m');ax.scatter([.5],[0],color='white',edgecolors='black');ax.annotate('출력 거리 0.525m',(.5,0),xytext=(.12,.4),arrowprops={'arrowstyle':'->'},fontproperties=font);ax.set_xlabel('x (m)');ax.set_ylabel('y (m)');ax.legend(prop=font);ax.set_title('nvblox 합성 평면 도구 확인 · 25mm 격자',fontproperties=font);fig.colorbar(im,ax=ax).set_label('거리 (m)',fontproperties=font);fig.text(.5,.015,'회색=미관측 / 물리 센서·실물 성능 시험 아님',ha='center',fontproperties=font);fig.tight_layout(rect=[0,.04,1,1]);fig.savefig(root/'photo/261029_nvblox_합성평면_거리지도.png',dpi=150);plt.close(fig)
p=base/'nvblox_isaac_frame';z=np.load(p/'distance_slice.npz');s=json.loads((p/'summary.json').read_text());meta=json.loads(Path(s['source']).read_text());pose=meta['poses'][0];rgb=Image.open(Path(s['source']).parent/pose['images']['off']['color']);depth=np.load(p/'input_depth_mm.npy')/1000.;a=np.ma.array(z['distance'],mask=~z['known']);o=z['origin'];res=float(z['resolution']);h,w=a.shape
fig,axes=plt.subplots(1,3,figsize=(15,5));axes[0].imshow(rgb);axes[0].set_title('기존 Isaac RGB 참고',fontproperties=font);axes[0].axis('off');di=axes[1].imshow(np.ma.masked_equal(depth,0),cmap='viridis',vmin=.15,vmax=1.5);axes[1].set_title('입력 깊이 · 16UC1(mm)→표시(m)',fontproperties=font);axes[1].axis('off');fig.colorbar(di,ax=axes[1],label='m',fraction=.04);mi=axes[2].imshow(a,origin='lower',extent=[o[0],o[0]+w*res,o[1],o[1]+h*res],cmap=cmap,vmin=0,vmax=.5,interpolation='nearest');axes[2].set_title('nvblox 거리 지도 · 회색=미관측',fontproperties=font);axes[2].set_xlabel('test_world x (m)');axes[2].set_ylabel('y (m)');fig.colorbar(mi,ax=axes[2],label='m',fraction=.04);fig.suptitle('정지 1시야 입력 연결 확인 · 기준 자세 TF 사용 · VO/실물 통합 아님',fontproperties=font);fig.tight_layout(rect=[0,0,1,.93]);fig.savefig(root/'photo/261029_nvblox_Isaac_깊이_거리지도.png',dpi=150)
