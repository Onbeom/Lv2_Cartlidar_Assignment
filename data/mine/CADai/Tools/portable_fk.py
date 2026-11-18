"""Portable URDF kinematics. All returned translations are mm; angles are rad.
No Fusion/FreeCAD/NumPy dependency. Input joint_values uses URDF SI conventions.
"""
import math
import xml.etree.ElementTree as ET

ARM_JOINTS=['RMX52_Base_Yaw','RMX52_Shoulder','RMX52_Elbow','RMX52_Wrist_Pitch','RMX52_Wrist_Roll_Reversed']
def identity():return [[float(i==j)for j in range(4)]for i in range(4)]
def multiply(A,B):return [[sum(A[i][k]*B[k][j]for k in range(4))for j in range(4)]for i in range(4)]
def point(T,p):return [sum(T[i][j]*p[j]for j in range(3))+T[i][3]for i in range(3)]
def origin(node):
    T=identity()
    if node is None:return T
    r,p,y=[float(x)for x in node.get('rpy','0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    R=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    for i in range(3):T[i][:3]=R[i];T[i][3]=float(node.get('xyz','0 0 0').split()[i])*1000
    return T
def rotation(a,q):
    n=math.sqrt(sum(x*x for x in a));x,y,z=[v/n for v in a];c=math.cos(q);s=math.sin(q);v=1-c;T=identity()
    R=[[c+x*x*v,x*y*v-z*s,x*z*v+y*s],[y*x*v+z*s,c+y*y*v,y*z*v-x*s],[z*x*v-y*s,z*y*v+x*s,c+z*z*v]]
    for i in range(3):T[i][:3]=R[i]
    return T
class Robot:
    def __init__(self,path):
        self.xml=ET.parse(path).getroot();self.links={x.get('name'):x for x in self.xml.findall('link')};self.joints={x.get('name'):x for x in self.xml.findall('joint')}
        roots=set(self.links)-{j.find('child').get('link')for j in self.joints.values()}
        if len(roots)!=1:raise ValueError('Expected one root')
        self.root=roots.pop()
    def fk(self,joint_values=None):
        values=dict(joint_values or {});F={self.root:identity()};todo=dict(self.joints)
        while todo:
            progressed=False
            for name,j in list(todo.items()):
                parent=j.find('parent').get('link');child=j.find('child').get('link')
                if parent not in F:continue
                q=values.get(name,0.0);mimic=j.find('mimic')
                if mimic is not None:q=values.get(mimic.get('joint'),0.0)*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
                A=origin(j.find('origin'));motion=identity();kind=j.get('type');axis=j.find('axis')
                a=[float(x)for x in axis.get('xyz').split()]if axis is not None else [0,0,1]
                if kind in ['revolute','continuous']:motion=rotation(a,q)
                elif kind=='prismatic':
                    for i in range(3):motion[i][3]=a[i]*q*1000
                F[child]=multiply(multiply(F[parent],A),motion);del todo[name];progressed=True
            if not progressed:raise ValueError('Disconnected/cyclic joint graph')
        return F
    def frame_values(self,row):
        values=dict(zip(ARM_JOINTS,row['arm_q_rad']))
        values['gripper_left_joint']=(row['aperture_mm']-19.62773936566)/2000
        return values

