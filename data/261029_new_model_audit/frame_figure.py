from pathlib import Path
import sys,json,matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.font_manager import FontProperties
import numpy as np
from model_frames import ModelFrames
R=Path('/home/pa4/git/lv2_next_module');O=R/'Cartlidar/sim_render/results/new_model_261029';p=json.loads((O/'perception_model_profile.json').read_text());m=ModelFrames(p['urdf']);params=json.loads((R/'261029_1030/urdf_261028/isaac/robot_params.json').read_text());font=FontProperties(fname='/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc');plt.rcParams['font.family']=font.get_name();fig,axes=plt.subplots(1,3,figsize=(14,5),sharex=True,sharey=True)
for ax,name in zip(axes,['drive','stow','zero']):
 poses=m.poses(params['poses'][name]['value']);arm=[poses[k][:3,3] for k in ['arm_base_link','link2','link3','link4','link5','gripper_roll_link','tool_link']];arm=np.array(arm);ax.plot(arm[:,0]*1000,arm[:,2]*1000,'o-',color='#666',label='팔 링크 원점(정적 자세)')
 for frame,color,label in [('top_camera_front_color_optical_frame','#1f77b4','앞 D435'),('top_camera_rear_color_optical_frame','#2ca02c','뒤 D435'),('arm_camera_optical_frame','#d62728','팔 카메라')]:
  T=poses[frame];x,z=T[[0,2],3]*1000;dx,dz=T[[0,2],2]*80;ax.scatter(x,z,color=color,s=40);ax.arrow(x,z,dx,dz,color=color,head_width=9,length_includes_head=True);ax.text(x+7,z+7,label,fontsize=9,color=color)
 ax.axhline(-6.75,color='black',lw=1,label='바닥');ax.scatter(0,0,marker='+',color='black');ax.set_title(name);ax.set_xlabel('base_link x(mm)');ax.grid(alpha=.25);ax.set_aspect('equal');ax.set_xlim(-350,500);ax.set_ylim(-30,600)
axes[0].set_ylabel('base_link z(mm)');fig.suptitle('새 모델 카메라 좌표계 · 옆에서 본 광학 +Z 방향\nURDF 정적 계산이며 실제 보정·가시성·구동 검증이 아님',fontsize=14);fig.tight_layout();fig.savefig(O/'camera_frames.png',dpi=160);fig.savefig(R/'photo/261029_새모델_카메라좌표계.png',dpi=160)
