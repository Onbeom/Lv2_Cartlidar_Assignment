"""새 모델의 실제 저장 깊이·필터·nvblox 출력 비교 그림."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
root=Path(__file__).resolve().parents[1]
base=root/'Cartlidar/sim_render/results/followup_261028_1457'
plt.rcParams['font.family']='Noto Sans CJK JP'
plt.rcParams['axes.unicode_minus']=False
prefix='top_camera_front_color_optical_frame'
depth=np.load(base/'camera_new_model_spec'/(prefix+'_specified_near105mm_depth.npy'))
mask=np.load(base/'nvblox_self_filter_cached'/(prefix+'_mask.npy'))
filtered=np.load(base/'nvblox_self_filter_cached'/(prefix+'_filtered_m.npy'))
fig,ax=plt.subplots(2,3,figsize=(16,9))
ax[0,0].imshow(plt.imread(base/'camera_new_model_spec'/(prefix+'_specified_near105mm.png')))
ax[0,0].set_title('새 모델 · drive · 앞 RGB 참고')
ax[0,1].imshow(mask,cmap='gray',vmin=0,vmax=1);ax[0,1].set_title('제거한 로봇 영역 · 흰색 33,932화소')
im=ax[0,2].imshow(np.ma.masked_equal(filtered,0),cmap='viridis',vmin=.15,vmax=1.5);ax[0,2].set_title('필터 후 깊이 · 제거 영역은 무효');fig.colorbar(im,ax=ax[0,2],label='m',fraction=.04)
for a in ax[0]:a.axis('off')
im=ax[1,0].imshow(depth,cmap='viridis',vmin=.15,vmax=1.5);ax[1,0].set_title('필터 전 깊이 · 로봇 포함');ax[1,0].axis('off');fig.colorbar(im,ax=ax[1,0],label='m',fraction=.04)
for a,directory,title in [(ax[1,1],'nvblox_new_model_spec','필터 전 nvblox 거리 지도'),(ax[1,2],'nvblox_self_filtered_new_model','필터 후 nvblox 거리 지도')]:
    z=np.load(base/directory/'distance_slice.npz');v=np.ma.array(z['distance'],mask=~z['known']);o=z['origin'];res=float(z['resolution']);h,w=v.shape;cmap=plt.colormaps['viridis'].copy();cmap.set_bad('lightgray')
    im=a.imshow(v,origin='lower',extent=[o[0],o[0]+w*res,o[1],o[1]+h*res],cmap=cmap,vmin=0,vmax=.7,interpolation='nearest');a.set_title(title);a.set_xlabel('test_world x (m)');a.set_ylabel('y (m)');fig.colorbar(im,ax=a,label='m',fraction=.04)
fig.suptitle('로봇 자기 영역 제거 → nvblox 입력 연결 확인',fontsize=18)
fig.text(.5,.025,'정지 1시야 · 기준 자세 TF · 모의 결과 / 회색=미관측 / 제거한 뒤쪽 공간을 자유공간으로 확정하지 않음',ha='center',fontsize=12)
fig.subplots_adjust(top=.90,bottom=.10,hspace=.28,wspace=.28)
fig.savefig(root/'photo/261029_새모델_자기영역제거_nvblox.png',dpi=140)
