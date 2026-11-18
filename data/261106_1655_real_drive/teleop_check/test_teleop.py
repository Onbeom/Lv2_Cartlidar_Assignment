#!/usr/bin/env python3
"""keyboard_teleop.py 도구 확인(PC, 모의). 가짜 터미널로 키를 넣고 /joy 를 받아 본다. 실물 · twist_mux 없음."""
import csv, os, pty, signal, subprocess, sys, time
import rclpy
from sensor_msgs.msg import Joy
from rclpy.qos import qos_profile_sensor_data
HERE=os.path.dirname(os.path.abspath(__file__)); TOOL=os.path.join(HERE,'..','jetson','keyboard_teleop.py')
rclpy.init(); n=rclpy.create_node('checker')
log=[]
n.create_subscription(Joy,'/joy',lambda m:log.append((time.monotonic(),m.header.frame_id,list(m.axes),list(m.buttons))),10)
be=[]; n.create_subscription(Joy,'/joy',lambda m:be.append(time.monotonic()),qos_profile_sensor_data)  # OpenCRHardware 와 같은 QoS
def spin(s):
    t=time.monotonic()+s
    while time.monotonic()<t: rclpy.spin_once(n,timeout_sec=0.01)
def start():
    m,sl=pty.openpty()
    p=subprocess.Popen([sys.executable,TOOL],stdin=sl,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,start_new_session=True)
    os.close(sl); return p,m
res=[]
def rec(name,ok,val): res.append((name,ok,val)); print(('통과' if ok else '실패'),name,val)
spin(1.0)
p,m=start(); spin(2.5)
t0=time.monotonic(); spin(2.0); w=[x for x in log if x[0]>=t0]
hz=len(w)/2.0; rec('T1 키 없음: heartbeat 18 Hz 이상, 모두 0', hz>=18 and all(not any(x[2]) and not any(x[3]) for x in w), f'{hz:.1f} Hz')
rec('T1b sensor-data QoS 구독도 받음', len(be)>0, f'{len(be)}개')
def press(k): os.write(m,k.encode())
def win(s):
    t=time.monotonic(); spin(s); return [x for x in log if x[0]>=t]
press('w'); w=win(1.2); on=[x for x in w if x[3][4]==1]
dur=on[-1][0]-on[0][0] if on else 0
rec('T2 w: 전진 0.25 · 데드맨 켬, 약 0.7 s 뒤 스스로 멈춤', on and all(x[2][1]==0.25 and x[2][0]==0 for x in on) and 0.55<dur<0.8 and w[-1][3][4]==0, f'{dur:.2f} s')
press('r'); spin(0.8)  # 잠금 시험: 리셋으로 풀고 시작
press('2'); press('s'); w=win(0.3); rec('T3 2 + s: 후진 -0.5', any(x[2][1]==-0.5 for x in w), '')
press('a'); w=win(0.3); rec('T3 a: 왼쪽 회전 axes[0] +0.5', any(x[2][0]==0.5 and x[2][1]==0 for x in w), '')
press('d'); w=win(0.3); rec('T3 d: 오른쪽 회전 axes[0] -0.5', any(x[2][0]==-0.5 for x in w), '')
press(' '); t=time.monotonic(); spin(0.15); s=[x for x in log if x[0]>=t+0.06]; rec('T4 space: 0.06 s 뒤부터 모두 0', s and all(x[3][4]==0 and not any(x[2]) for x in s), '')
spin(0.8); press('3'); press('w'); spin(0.1); press('e'); t=time.monotonic(); press('w'); spin(0.45); s=[x for x in log if x[0]>=t+0.06]
rec('T5 e: 버튼 6 켬, 그동안 w 를 눌러도 안 움직임', s and all(x[3][6]==1 and x[3][4]==0 and not any(x[2]) for x in s), f'{len(s)}개')
spin(1.0); press('r'); w=win(0.8); on=[x for x in w if x[3][7]==1]; rec('T6 r: 버튼 7 켬 약 0.5 s', on and 0.4<on[-1][0]-on[0][0]<0.6, f'{on[-1][0]-on[0][0]:.2f} s' if on else '')
rec('T6b 미션 버튼 8 · 9 안 냄, 버튼 12 · 축 8', all(len(x[3])==12 and len(x[2])==8 and x[3][8]==0 and x[3][9]==0 for x in log if x[1]=='keyboard'), '')
def wait_exit(p):
    t=time.monotonic()
    while p.poll() is None and time.monotonic()-t<5: spin(0.02)
    spin(0.3)  # 이미 보낸 것까지 받은 뒤 잰다
    return p.poll()
press('w'); spin(0.1); press('q'); rc=wait_exit(p); t=time.monotonic(); spin(0.5); after=[x for x in log if x[0]>t]
# 잠금 시험용 시간선 저장
with open(os.path.join(HERE,'joy_log.csv'),'w',newline='') as f:
    c=csv.writer(f); [c.writerow([f'{x[0]:.3f}',x[3][6],x[3][7]]) for x in log if x[1]=='keyboard']
last=[x for x in log if x[1]=='keyboard'][-1]
rec('T7 q: 종료 0, 마지막 메시지 정지, 그 뒤 /joy 없음', rc==0 and not after and not any(last[2]) and last[3][4]==0, f'rc={rc}')
p,m=start(); spin(2.0); os.kill(p.pid,signal.SIGHUP); rc=wait_exit(p); t=time.monotonic(); spin(0.5)
rec('T8 SIGHUP(ssh 끊김): 끝나고 /joy 없음', not [x for x in log if x[0]>t], f'rc={rc}')
fp=n.create_publisher(Joy,'/joy',10)
def foreign(): 
    j=Joy(); j.header.frame_id='joy'; j.axes=[0.0,0.0]; j.buttons=[0]*12; fp.publish(j)
p,m=start(); t=time.monotonic()
while p.poll() is None and time.monotonic()-t<6: foreign(); spin(0.05)
rec('T9 시작 때 진짜 조이스틱이 있으면 거부', p.poll()==3, f'rc={p.poll()}')
if p.poll() is None: p.kill()
p,m=start(); spin(2.5); t=time.monotonic()
while p.poll() is None and time.monotonic()-t<3: foreign(); spin(0.05)
rec('T10 도중에 조이스틱이 들어오면 끝냄', p.poll()==3, f'rc={p.poll()}')
if p.poll() is None: p.kill()
q=subprocess.run([sys.executable,TOOL],stdin=subprocess.DEVNULL,capture_output=True,text=True); rec('T11 터미널 아니면 거부', q.returncode==2, f'rc={q.returncode}')
n.destroy_node(); rclpy.shutdown()
print('통과',sum(1 for r in res if r[1]),'/',len(res)); sys.exit(0 if all(r[1] for r in res) else 1)
