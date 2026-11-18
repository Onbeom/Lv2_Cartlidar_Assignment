"""팔 태그의 새 관측을 odom에서 합치는 R6 시제품. 목표 표는 판정에만 쓰며 관측 ID/위치를 만들지 않는다."""
import json
import math
from collections import Counter
import numpy as np

ERRORS = ['OK','MISSING','UNEXPECTED','MULTIPLE','OFFSET','WRONG_COLOR','WRONG_SHAPE']

class DiceObservations:
    def __init__(self, cells, expect, yellow_shape, tolerance=0.010):
        if len(cells)!=9 or len(expect)!=9:raise ValueError('격자9칸 필요')
        self.cells=np.asarray(cells,float);self.expect=list(expect);self.yellow_shape=yellow_shape;self.tolerance=tolerance
        self.samples={};self.covered=set();self.conflicts=[];self.frames=[]

    def observe(self, pucks, stamp, frame_name, coverage):
        if frame_name in self.frames:raise ValueError('같은 관측 재사용')
        self.frames.append(frame_name);self.covered.update(coverage)
        ids=[p['tag_id'] for p in pucks]
        if len(set(ids))!=len(ids):self.conflicts.append('한 관측에 같은ID둘')
        for p in pucks:
            if p['tag_id'] not in range(12):
                self.conflicts.append('미등록ID');continue
            pos=np.asarray(p['position'],float)
            if pos.shape!=(3,) or not np.isfinite(pos).all():self.conflicts.append('유효위치없음');continue
            # 작업 박스의 격자 바깥 다른 장소 관측은 제외. 목표 칸 점유 여부는 참조하지 않음.
            if np.min(np.linalg.norm(self.cells[:,:2]-pos[:2],axis=1))>0.060:continue
            self.samples.setdefault(p['tag_id'],[]).append({**p,'stamp':float(stamp),'view':frame_name})

    def finish(self):
        seen=[];conflicts=list(self.conflicts)
        for tag, samples in sorted(self.samples.items()):
            positions=np.array([p['position'] for p in samples]);median=np.median(positions,axis=0)
            if np.max(np.linalg.norm(positions[:,:2]-median[:2],axis=1))>0.020:conflicts.append(f'ID{tag} 위치상충')
            values={}
            for field in ('color','shape'):
                votes=Counter(p[field] for p in samples if p[field] is not None);rank=votes.most_common()
                values[field]=rank[0][0] if rank and (len(rank)==1 or rank[0][1]>rank[1][1]) else None
                if values[field] is None:conflicts.append(f'ID{tag} {field}불명')
            seen.append({'tag_id':tag,**values,'position':median.tolist(),'n_obs':len(samples),'views':[p['view'] for p in samples],
                         'first_stamp':min(p['stamp'] for p in samples),'last_stamp':max(p['stamp'] for p in samples)})
        bins=[[] for _ in range(9)]
        for p in seen:
            dist=np.linalg.norm(self.cells[:,:2]-np.array(p['position'])[:2],axis=1);idx=int(np.argmin(dist));bins[idx].append((float(dist[idx]),p))
        errors=[];offsets=[]
        for i,slot in enumerate(bins):
            want=self.expect[i];d,p=min(slot,key=lambda x:x[0]) if slot else (0,None)
            if want==0:e=2 if slot else 0
            elif not slot:e=1
            elif len(slot)>1:e=3
            elif d>self.tolerance:e=4
            elif want==3:e=0 if p['shape']==self.yellow_shape else 6
            else:e=0 if p['color']==('red' if want==1 else 'green') else 5
            errors.append(e);offsets.append(d)
        complete=len(self.covered)==9 and not conflicts
        return {'observed':complete,'passed':complete and not any(errors),'wrong':[i for i,e in enumerate(errors) if e],
                'error':errors,'offset':offsets,'pucks':seen,'covered_cells':sorted(self.covered),'conflicts':conflicts,'views':self.frames}


def quat_matrix(x,y,z,w):
    norm=x*x+y*y+z*z+w*w
    if norm<1e-12:raise ValueError('회전사원수없음')
    s=2./norm
    return np.array([[1-s*(y*y+z*z),s*(x*y-z*w),s*(x*z+y*w)],
                     [s*(x*y+z*w),1-s*(x*x+z*z),s*(y*z-x*w)],
                     [s*(x*z-y*w),s*(y*z+x*w),1-s*(x*x+y*y)]])

def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.time import Time
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from std_msgs.msg import String
    from sensor_msgs.msg import CameraInfo
    from robot_interfaces.msg import PuckArray
    from tf2_ros import Buffer,TransformListener
    class Verifier(Node):
        def __init__(self):
            super().__init__('dice_multiview');self.acc=None;self.info=None;self.serial=0;self.active=False;self.min_stamp=0.0
            self.buffer=Buffer();self.listener=TransformListener(self.buffer,self)
            self.pub=self.create_publisher(String,'/perception/dice_multiview',10)
            self.create_subscription(String,'/perception/dice_session',self.command,10)
            self.create_subscription(CameraInfo,'/arm_camera/camera_info',lambda m:setattr(self,'info',m),10)
            self.create_subscription(PuckArray,'/perception/pucks',self.pucks,QoSProfile(depth=30,reliability=ReliabilityPolicy.RELIABLE))
        def command(self,msg):
            try:
                cmd=json.loads(msg.data)
                if cmd['action']=='begin':
                    self.acc=DiceObservations(cmd['cells'],cmd['expect'],cmd['yellow_shape'],cmd.get('tolerance',.010));self.serial=0;self.active=True;self.min_stamp=cmd.get("start_stamp",0.0)
                    self.publish({'state':'ready'})
                elif cmd['action']=='finish':
                    self.active=False;self.publish({'state':'result',**self.acc.finish()})
            except (ValueError,KeyError,TypeError,AttributeError) as e:self.publish({'state':'error','message':str(e)})
        def publish(self,data):
            msg=String();msg.data=json.dumps(data,ensure_ascii=False);self.pub.publish(msg)
        def transform(self,source,stamp):
            tf=self.buffer.lookup_transform('odom',source,Time.from_msg(stamp),timeout=Duration(seconds=0.0)).transform
            T=np.eye(4);T[:3,:3]=quat_matrix(tf.rotation.x,tf.rotation.y,tf.rotation.z,tf.rotation.w);T[:3,3]=[tf.translation.x,tf.translation.y,tf.translation.z];return T
        def pucks(self,msg):
            if not self.active or self.acc is None or self.info is None:return
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            if stamp<self.min_stamp or f"{stamp:.9f}" in self.acc.frames:return
            try:
                T=self.transform(msg.header.frame_id,msg.header.stamp);camera=self.transform(self.info.header.frame_id,msg.header.stamp)
            except Exception:return
            pucks=[]
            for p in msg.pucks:
                point=T@np.array([p.pose.position.x,p.pose.position.y,p.pose.position.z,1.])
                pucks.append({'tag_id':p.tag_id,'color':{1:'red',2:'green'}.get(p.color),'shape':{1:'cylinder',2:'cuboid'}.get(p.shape),'position':point[:3].tolist(),'confidence':float(p.confidence)})
            coverage=[];inverse=np.linalg.inv(camera);K=self.info.k
            for i,cell in enumerate(self.acc.cells):
                corners=[inverse@np.array([cell[0]+dx,cell[1]+dy,cell[2]+.0602,1.]) for dx in (-.008,.008) for dy in (-.008,.008)]
                if all(q[2]>.025 and 32<=K[0]*q[0]/q[2]+K[2]<=self.info.width-32 and 32<=K[4]*q[1]/q[2]+K[5]<=235 for q in corners):coverage.append(i)
            stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
            self.acc.observe(pucks,stamp,f'{stamp:.9f}',coverage);self.serial+=1
            self.publish({'state':'observed','stamp':stamp,'n':self.serial,'ids':sorted(p['tag_id'] for p in pucks)})
    rclpy.init();node=Verifier()
    try:rclpy.spin(node)
    finally:node.destroy_node();rclpy.shutdown()

if __name__=='__main__':main()
