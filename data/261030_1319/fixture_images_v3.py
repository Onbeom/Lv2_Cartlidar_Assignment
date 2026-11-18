"""고정 K·카메라 자세의 이상적 합성 태그 영상. 실제 센서/Isaac 렌더 성능 자료가 아님."""
import cv2,numpy as np
CENTERS=((1.468,.950),(3.670,3.152),(2.936,5.354),(.734,5.354),(2.202,5.354),(3.670,3.886))
K=dict(fx=800.,fy=800.,cx=320.,cy=240.)
def create(folder,camera_height=.33):
 folder.mkdir(exist_ok=True);records=[];dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
 for box,(bx,by) in enumerate(CENTERS):
  image=np.full((480,640,3),80,np.uint8);truth=[]
  def pixel(x,y,z):return np.array([320+800*(x-bx)/(camera_height-z),240-800*(y-by)/(camera_height-z)])
  for side in [-1,1]:
   tid=2*box+(side+1)//2;x=bx+side*.04;y=by;z=.13;color='red' if tid%2==0 else 'green';shape='cylinder' if (tid//2)%2==0 else 'cuboid';bgr=(0,0,210) if color=='red' else (0,185,0)
   center=pixel(x,y,z);radius=800*.015/(camera_height-z)
   if shape=='cylinder':cv2.circle(image,tuple(np.rint(center).astype(int)),round(radius),bgr,-1)
   else:cv2.rectangle(image,tuple(np.rint(center-radius).astype(int)),tuple(np.rint(center+radius).astype(int)),bgr,-1)
   def corners(half):return np.float32([pixel(x-half,y+half,z),pixel(x+half,y+half,z),pixel(x+half,y-half,z),pixel(x-half,y-half,z)])
   cv2.fillConvexPoly(image,np.rint(corners(.010)).astype(np.int32),(235,235,235))
   marker=cv2.aruco.drawMarker(dictionary,tid,128);H=cv2.getPerspectiveTransform(np.float32([[0,0],[127,0],[127,127],[0,127]]),corners(.008));warp=cv2.warpPerspective(marker,H,(640,480),flags=cv2.INTER_NEAREST,borderValue=255);mask=cv2.warpPerspective(np.full((128,128),255,np.uint8),H,(640,480),flags=cv2.INTER_NEAREST);image[mask>0]=cv2.cvtColor(warp,cv2.COLOR_GRAY2BGR)[mask>0]
   truth.append(dict(id=tid,box=box+1,side=side,color=color,shape=shape,pose='standing',x_mm=x*1000,y_mm=y*1000))
  path=folder/f'box_{box+1}.png';cv2.imwrite(str(path),image);T=np.eye(4);T[:3,:3]=np.diag([1.,-1.,-1.]);T[:3,3]=[bx,by,camera_height];records.append(dict(path=str(path),T=T.tolist(),truth=truth))
 return records
