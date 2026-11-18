#!/usr/bin/env python3
"""교안 L7 장애 주입 (PC 모의, 기준은 README). 인지 노드를 띄우고 렌더 영상을 벽시계 시각으로 재생하며 장애를 넣는다.
실행은 run_fault.sh. 결과: <out>/result.json, <out>/fault_cases.csv, <out>/logs/
"""
import argparse
import csv
import json
import os
import random
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import TransformStamped
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from tf2_msgs.msg import TFMessage
from tf2_ros import StaticTransformBroadcaster

from robot_interfaces.msg import PerceptionResult, PuckArray
from robot_interfaces.srv import SetPerceptionMode

ROOT = Path('/home/pa4/git/lv2_next_module')
sys.path.insert(0, str(ROOT / 'Cartlidar/sim_render'))
from replay_ros import odom_child, quat_from_matrix  # noqa: E402

SET_YAML = ROOT / 'Cartlidar/sim_render/set_yaml_values.py'
FRONT, ARM = SetPerceptionMode.Request.SEARCH_FRONT, SetPerceptionMode.Request.ARM_ALIGN
STOP_MAX, RECOVER_MAX, AGE_MAX = 0.3, 0.5, 0.3


def to_stamp(t):
    from builtin_interfaces.msg import Time
    return Time(sec=int(t), nanosec=int((t - int(t)) * 1e9))


def sec(st):
    return st.sec + st.nanosec * 1e-9


class Driver(Node):
    def __init__(self, a):
        super().__init__('fault_driver')
        self.a = a
        rel = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.pc = self.create_publisher(Image, '/top_camera_front/color/image_raw', rel)
        self.pi = self.create_publisher(CameraInfo, '/top_camera_front/color/camera_info', rel)
        self.pd = self.create_publisher(Image, '/top_camera_front/aligned_depth_to_color/image_raw', rel)
        self.pa = self.create_publisher(Image, '/arm_camera/image_raw', rel)
        self.pai = self.create_publisher(CameraInfo, '/arm_camera/camera_info', rel)
        self.tf = self.create_publisher(TFMessage, '/tf', 100)
        self.js = self.create_publisher(JointState, '/joint_states', 10)
        self.res, self.pucks, self.lock = [], [], threading.Lock()
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PerceptionResult, '/perception/result', self.on_res, q)
        self.create_subscription(PuckArray, '/perception/pucks', self.on_puck, q)
        self.mode_cli = self.create_client(SetPerceptionMode, '/perception/set_mode')
        self.static = StaticTransformBroadcaster(self)
        m = TransformStamped()
        m.header.frame_id, m.child_frame_id = 'map', 'odom'
        m.transform.rotation.w = 1.0
        self.static.sendTransform([m])
        self.st = dict(d435=True, depth=True, tf=True, arm=False, age=0.0, dup=None, swing=False, bad=None)
        self.load_frames()
        self.running = True
        self.thread = threading.Thread(target=self.loop, daemon=True)

    def load_frames(self):
        r = Path(self.a.render)
        self.meta = json.load(open(r / 'render_b0.json'))
        poses = {(p['id'], p['images']['color'].split('/')[0][-2:]): p for p in self.meta['poses']}
        self.pose = poses[(self.a.d435_id, 'L1')]
        self.arm_pose = poses[(self.a.arm_id, 'L1')]
        self.bgr = cv2.imread(str(r / self.pose['images']['color']), cv2.IMREAD_COLOR)
        dep = np.load(r / self.pose['images']['depth']).astype(np.float32)
        dep[~np.isfinite(dep)] = 0.0
        self.dep = dep
        self.arm_bgr = cv2.imread(str(r / self.arm_pose['images']['color']), cv2.IMREAD_COLOR)
        child, t_cb = odom_child()
        self.tf_child = child
        self.T = np.array(self.pose['T_world_base_footprint']) @ np.linalg.inv(t_cb)
        self.q = quat_from_matrix(self.T[:3, :3])

    def on_res(self, m):
        with self.lock:
            self.res.append((time.time(), sec(m.header.stamp), m.mode, m.object_found, m.offset_valid))

    def on_puck(self, m):
        with self.lock:
            self.pucks.append((time.time(), sec(m.header.stamp), len(m.pucks)))

    def info(self, st, cam, w, h):
        k = cam['K']
        i = CameraInfo()
        i.header.stamp, i.header.frame_id = st, cam['frame']
        i.width, i.height = w, h
        i.distortion_model = 'plumb_bob'
        i.d = [0.0] * 5
        i.k = [k['fx'], 0.0, k['cx'], 0.0, k['fy'], k['cy'], 0.0, 0.0, 1.0]
        i.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        i.p = [k['fx'], 0.0, k['cx'], 0.0, 0.0, k['fy'], k['cy'], 0.0, 0.0, 0.0, 1.0, 0.0]
        return i

    def image(self, st, frame, arr, enc):
        m = Image()
        m.header.stamp, m.header.frame_id = st, frame
        m.height, m.width = arr.shape[:2]
        m.encoding, m.step = enc, arr.shape[1] * (arr.shape[2] if arr.ndim == 3 else 1) * arr.dtype.itemsize
        m.data = arr.tobytes()
        return m

    def color_msg(self, st, frame):
        bad = self.st['bad']
        m = self.image(st, frame, self.bgr, 'bgr8')
        if bad == 'color_mono8':
            m = self.image(st, frame, cv2.cvtColor(self.bgr, cv2.COLOR_BGR2GRAY), 'mono8')
        elif bad == 'color_short':
            m.data = bytes(m.data[:len(m.data) // 2])
        elif bad == 'color_zero':
            m.height, m.width, m.step, m.data = 0, 0, 0, b''
        elif bad == 'color_weird':
            m.encoding = 'weird_enc'
        return m

    def depth_msg(self, st, frame):
        bad = self.st['bad']
        if bad == 'depth_size':
            return self.image(st, frame, cv2.resize(self.dep, (320, 240), interpolation=cv2.INTER_NEAREST), '32FC1')
        m = self.image(st, frame, self.dep, '32FC1')
        if bad == 'depth_short':
            m.data = bytes(m.data[:len(m.data) // 2])
        return m

    def arm_msg(self, st, frame):
        m = self.image(st, frame, self.arm_bgr, 'bgr8')
        if self.st['bad'] == 'arm_short':
            m.data = bytes(m.data[:len(m.data) // 2])
        return m

    def loop(self):
        period, nxt, flip = 1.0 / self.a.fps, time.time(), False
        d435, arm = self.meta['d435'], self.meta['arm']
        names, anames = list(self.pose['arm_q'].keys()), list(self.arm_pose['arm_q'].keys())   # 자세마다 관절 수가 다름
        pos, apos = [float(v) for v in self.pose['arm_q'].values()], [float(v) for v in self.arm_pose['arm_q'].values()]
        h, w = self.bgr.shape[:2]
        ah, aw = self.arm_bgr.shape[:2]
        while self.running:
            now, s = time.time(), self.st
            cur = to_stamp(now)
            js = JointState()
            js.header.stamp, js.name = cur, (anames if s['arm'] else names)
            js.position = apos if s['arm'] else pos
            self.js.publish(js)
            if s['tf']:
                t = TransformStamped()
                t.header.stamp, t.header.frame_id, t.child_frame_id = cur, 'odom', self.tf_child
                t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, self.T[:3, 3])
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = map(float, self.q)
                self.tf.publish(TFMessage(transforms=[t]))
            ts = now - s['age']
            if s['dup'] is not None:
                ts = s['dup']
            if s['swing']:
                flip = not flip
                ts = now - (0.1 if flip else 0.0)
            st = to_stamp(ts)
            if s['d435']:
                self.pi.publish(self.info(st, d435, w, h))
                if s['depth']:
                    self.pd.publish(self.depth_msg(st, d435['frame']))
                self.pc.publish(self.color_msg(st, d435['frame']))
            if s['arm']:
                # 팔은 촬영 시각 TF 를 기다린다(ARM_TF_PENDING). 관절 값이 그 시각을 지나 있어야 하므로 카메라 지연처럼 0.1 s 앞선 stamp 를 쓴다
                ast = to_stamp(ts - 0.1) if s['dup'] is None else st
                self.pai.publish(self.info(ast, arm, aw, ah))
                self.pa.publish(self.arm_msg(ast, arm['frame']))
            nxt += period
            time.sleep(max(0.0, nxt - time.time()))
            if time.time() - nxt > 1.0:
                nxt = time.time()

    def set_mode(self, mode, timeout=40.0):
        """재기동 직후에는 옛 서버 정보가 남아 요청이 사라질 수 있어(F12 첫 실행) 3 s 마다 다시 보낸다."""
        t0 = time.time()
        if not self.mode_cli.wait_for_service(timeout_sec=timeout):
            return False
        r = SetPerceptionMode.Request()
        r.mode = mode
        while time.time() - t0 < timeout:
            f = self.mode_cli.call_async(r)
            t1 = time.time()
            while not f.done() and time.time() - t1 < 3.0:
                time.sleep(0.02)
            if f.done():
                return f.result().success
            self.mode_cli.remove_pending_request(f)
        return False

    def between(self, t0, t1, src=None):
        with self.lock:
            return [r for r in (self.res if src is None else src) if t0 <= r[0] <= t1]


class Stack:
    """인지 launch 와 robot_state_publisher 를 띄우고 끈다."""

    def __init__(self, a, out):
        self.a, self.out, self.proc, self.n = a, out, None, 0
        (out / 'logs').mkdir(parents=True, exist_ok=True)
        params = out / 'rsp_params.yaml'
        yaml.safe_dump({'robot_state_publisher': {'ros__parameters': {'robot_description': open(a.urdf).read()}}}, open(params, 'w'))
        self.rsp = subprocess.Popen(['ros2', 'run', 'robot_state_publisher', 'robot_state_publisher', '--ros-args', '--params-file', str(params)],
                                    stdout=open(out / 'logs/rsp.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)

    def config(self, name, main, arm):
        c = self.out / f'config_{name}'
        shutil.copytree(self.a.share, c)
        p = c / 'perception.yaml'
        s = p.read_text().replace('    image_qos: sensor_data ', '    image_qos: reliable    ')
        p.write_text(s)
        assert '    image_qos: reliable' in p.read_text()
        subprocess.run([sys.executable, '-I', str(SET_YAML), str(p), f'detector.engine_path={main}', f'arm.engine_path={arm}'], check=True)
        return c

    def start(self, cfg, tag):
        self.n += 1
        self.log = self.out / f'logs/perception_{self.n:02d}_{tag}.log'
        self.t_start = time.time()
        self.proc = subprocess.Popen(['ros2', 'launch', 'perception', 'perception.launch.py', f'config_dir:={cfg}',
                                      f'survey_output:={self.out}/survey_{self.n}.json', 'use_tfmini:=false', 'use_obstacle_stop:=false'],
                                     stdout=open(self.log, 'w'), stderr=subprocess.STDOUT, start_new_session=True)

    def container(self):
        if self.proc is None:
            return None
        r = subprocess.run(['pgrep', '-g', str(self.proc.pid), '-f', 'component_container'], capture_output=True, text=True)
        p = r.stdout.split()
        return int(p[0]) if p else None

    def alive(self):
        return self.container() is not None

    def stop(self):
        if self.proc is None:
            return
        try:
            os.killpg(self.proc.pid, signal.SIGCONT)
            os.killpg(self.proc.pid, signal.SIGINT)
        except ProcessLookupError:
            pass
        t0 = time.time()
        while time.time() - t0 < 10 and self.proc.poll() is None:
            time.sleep(0.1)
        try:
            os.killpg(self.proc.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        self.proc.wait()
        self.proc = None
        time.sleep(1.0)

    def close(self):
        self.stop()
        try:
            os.killpg(self.rsp.pid, signal.SIGINT)
            self.rsp.wait(timeout=5)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.rsp.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass

    def log_count(self, *words):
        t = self.log.read_text(errors='replace')
        return sum(1 for ln in t.splitlines() if any(w in ln for w in words))

    def log_lines(self, word, n=2):
        return [ln[:260] for ln in self.log.read_text(errors='replace').splitlines() if word in ln][:n]


def base_rate(d, seconds=3.0):
    t0 = time.time()
    time.sleep(seconds)
    return len(d.between(t0, time.time())) / seconds


def first_after(d, t, src=None):
    with d.lock:
        c = [r[0] for r in (d.res if src is None else src) if r[1] >= t and r[0] >= t]
    return min(c) - t if c else None


def stop_resume(d, s, off, on, wait=2.0):
    """끊고 재개하며 멈춤 · 복구를 잰다."""
    rate = base_rate(d)
    stall0 = s.log_count('CAMERA_STALL')
    t_stop = time.time()
    d.st.update(off)
    time.sleep(wait)
    stall = s.log_count('CAMERA_STALL') - stall0
    after = d.between(t_stop, time.time())
    last = max((r[0] for r in after), default=t_stop) - t_stop
    late = sum(1 for r in after if r[0] > t_stop + STOP_MAX)
    t_res = time.time()
    d.st.update(on)
    time.sleep(3.0)
    return dict(base_hz=round(rate, 2), stop_s=round(last, 3), late=late, recover_s=_r(first_after(d, t_res)), stall_warn=stall, alive=s.alive())


def _r(v, n=3):
    return None if v is None else round(v, n)


def f1(d, s):
    m = stop_resume(d, s, dict(d435=False), dict(d435=True))
    m['pass'] = m['base_hz'] > 0 and m['stop_s'] <= STOP_MAX and m['late'] == 0 and m['recover_s'] is not None and m['recover_s'] <= RECOVER_MAX and m['alive']
    return m


def f2(d, s):
    rate = base_rate(d)
    out = dict(base_hz=round(rate, 2))
    for age, key in ((0.2, 'age02'), (0.4, 'age04')):
        d.st['age'] = age
        time.sleep(0.3)
        t0 = time.time()
        time.sleep(3.0)
        r = d.between(t0, time.time())
        out[key] = dict(hz=round(len(r) / 3.0, 2), max_age=_r(max((x[0] - x[1] for x in r), default=None)))
    d.st['age'] = 0.0
    time.sleep(1.0)
    out['alive'] = s.alive()
    out['pass'] = rate > 0 and out['age02']['hz'] >= 0.8 * rate and out['age04']['hz'] == 0 and out['alive']
    return out


def f3(d, s):
    base_rate(d, 1.0)
    t0 = time.time()
    d.st['dup'] = t0
    time.sleep(0.25)
    t1 = time.time()
    time.sleep(3.0)
    r = d.between(t1, time.time())
    d.st['dup'] = None
    time.sleep(1.0)
    return dict(new_after_first=len(r), alive=s.alive(), **{'pass': len(r) == 0 and s.alive()})


def f4(d, s):
    base_rate(d, 1.0)
    t0 = time.time()
    d.st['swing'] = True
    time.sleep(4.0)
    r = d.between(t0 + 0.2, time.time())
    d.st['swing'] = False
    back = sum(1 for a, b in zip(r, r[1:]) if b[1] < a[1])
    time.sleep(1.0)
    return dict(n=len(r), regress=back, alive=s.alive(), **{'pass': len(r) > 0 and back == 0 and s.alive()})


def f5(d, s):
    base_rate(d)
    with d.lock:
        base_found = sum(1 for r in d.res[-30:] if r[3])
    t0 = time.time()
    d.st['depth'] = False
    time.sleep(0.2)
    t1 = time.time()
    time.sleep(3.0)
    t2 = time.time()
    r, p = d.between(t1, t2), d.between(t1, t2, d.pucks)
    t3 = time.time()
    d.st['depth'] = True
    time.sleep(3.0)
    rec_found = next((x[0] - t3 for x in d.between(t3, time.time()) if x[3]), None)
    m = dict(base_found=base_found, found=sum(1 for x in r if x[3]), puck3d=sum(x[2] for x in p), n=len(r), recover_found_s=_r(rec_found), alive=s.alive(), t0=round(t1 - t0, 2))
    m['pass'] = base_found > 0 and m['found'] == 0 and m['puck3d'] == 0 and rec_found is not None and rec_found <= RECOVER_MAX and m['alive']
    return m


def f6(d, s):
    before = s.log_count('없음: base_link', '넘게 다름: base_link')
    rate = base_rate(d)
    d.st['tf'] = False
    t0 = time.time()
    time.sleep(12.0)
    n = len(d.between(t0 + 1.0, time.time())) / 11.0
    warn = s.log_count('없음: base_link', '넘게 다름: base_link') - before
    t_res = time.time()
    d.st['tf'] = True
    time.sleep(3.0)
    m = dict(base_hz=round(rate, 2), during_hz=round(n, 2), warn=warn, recover_s=_r(first_after(d, t_res)), alive=s.alive())
    m['pass'] = warn >= 1 and m['recover_s'] is not None and m['recover_s'] <= RECOVER_MAX and m['alive']
    return m


def f7(d, s):
    d.st.update(arm=True, d435=False)
    ok = d.set_mode(ARM)
    time.sleep(0.5)
    m = stop_resume(d, s, dict(arm=False), dict(arm=True))
    m['mode_ok'] = ok
    m['pass'] = ok and m['base_hz'] > 0 and m['stop_s'] <= STOP_MAX and m['late'] == 0 and m['recover_s'] is not None and m['recover_s'] <= RECOVER_MAX and m['alive']
    d.st.update(arm=False, d435=True)
    d.set_mode(FRONT)
    time.sleep(0.5)
    return m


def f8(d, s, cfg):
    out, ok = {}, True
    for bad in ('color_mono8', 'color_short', 'color_zero', 'color_weird', 'depth_size', 'depth_short', 'arm_short'):
        arm = bad == 'arm_short'
        if arm:
            d.st.update(arm=True, d435=False)
            d.set_mode(ARM)
        d.st['bad'] = bad
        time.sleep(2.5)
        alive = s.alive()
        t_res = time.time()
        d.st['bad'] = None
        time.sleep(2.5)
        rec = first_after(d, t_res)
        row = dict(alive=alive, recover_s=_r(rec))
        row['pass'] = alive and rec is not None and rec <= RECOVER_MAX
        if not alive:
            row['log'] = s.log.read_text(errors='replace').splitlines()[-4:]
            s.stop()
            s.start(cfg, f'after_{bad}')
            if arm:
                d.st.update(arm=False, d435=True)
            d.set_mode(FRONT)
            time.sleep(1.0)
        if arm and alive:
            d.st.update(arm=False, d435=True)
            d.set_mode(FRONT)
            time.sleep(0.5)
        out[bad] = row
        ok = ok and row['pass']
    return dict(cases=out, **{'pass': ok})


def f9(d, s):
    rate = base_rate(d)
    pid = s.container()
    os.kill(pid, signal.SIGSTOP)
    time.sleep(1.0)
    t_res = time.time()
    os.kill(pid, signal.SIGCONT)
    time.sleep(3.0)
    r = d.between(t_res, time.time())
    ages = [x[0] - x[1] for x in r]
    m = dict(base_hz=round(rate, 2), stale=sum(1 for a in ages if a > AGE_MAX), max_age=_r(max(ages, default=None)), recover_s=_r(first_after(d, t_res)), alive=s.alive())
    m['pass'] = m['stale'] == 0 and m['recover_s'] is not None and m['recover_s'] <= RECOVER_MAX and m['alive']
    return m


def f10(d, s):
    rate = base_rate(d)
    pid = s.container()
    t0 = time.time()
    while time.time() - t0 < 30.0:
        os.kill(pid, signal.SIGSTOP)
        time.sleep(0.15)
        os.kill(pid, signal.SIGCONT)
        time.sleep(0.15)
    r = d.between(t0, time.time())
    ages = np.array([x[0] - x[1] for x in r]) if r else np.array([0.0])
    over = float((ages > AGE_MAX).mean()) if r else 1.0
    time.sleep(1.0)
    m = dict(base_hz=round(rate, 2), hz=round(len(r) / 30.0, 2), age_p50=round(float(np.percentile(ages, 50)), 3), age_p99=round(float(np.percentile(ages, 99)), 3), over03=round(over, 4), alive=s.alive())
    m['pass'] = len(r) > 0 and over <= 0.005 and m['alive']
    return m


def f11(d, s, cfg):
    base_rate(d)
    t0 = time.time()
    os.kill(s.container(), signal.SIGKILL)
    time.sleep(5.0)
    after = d.between(t0, time.time())
    last = max((r[0] for r in after), default=t0) - t0
    auto = s.alive()
    launch_ended = s.proc.poll() is not None
    s.stop()
    t1 = time.time()
    s.start(cfg, 'after_kill')
    ok = d.mode_cli.wait_for_service(timeout_sec=40.0)
    time.sleep(2.0)
    no_mode = len(d.between(t1, time.time()))
    d.set_mode(FRONT)
    t_end = time.time() + 30.0
    first = None
    while time.time() < t_end:
        with d.lock:
            c = [r[0] for r in d.res if r[0] > t1]
        if c:
            first = min(c) - t1
            break
        time.sleep(0.05)
    m = dict(stop_s=round(last, 3), auto_restart=auto, launch_ended=launch_ended, service=ok, results_before_mode=no_mode, first_result_s=_r(first), alive=s.alive())
    m['pass'] = last <= STOP_MAX and first is not None and first <= 30.0 and m['alive']
    return m


def corrupt(src, dst, how):
    b = bytearray(Path(src).read_bytes())
    rnd = random.Random(20261106)
    if how == 'empty':
        b = bytearray()
    elif how == 'half':
        b = b[:len(b) // 2]
    elif how == 'random':
        b = bytearray(rnd.randbytes(len(b)))
    elif how == 'flip':
        for _ in range(2000):
            b[rnd.randrange(len(b) // 4, len(b))] ^= 0xFF
    Path(dst).write_bytes(b)


def f12(d, s, a, out):
    res, ok = {}, True
    (out / 'bad_engines').mkdir(exist_ok=True)
    for how in ('empty', 'half', 'random', 'flip', 'flip_nofile'):
        s.stop()
        bad = out / f'bad_engines/{how}.engine'
        corrupt(a.engine480, bad, how.split('_')[0])
        if how != 'flip_nofile':   # 배포 상태: 원래 엔진의 해시 파일이 옆에 있다. flip_nofile 은 해시 파일이 없을 때의 한계를 기록만 한다
            shutil.copy(str(a.engine480) + '.sha256', str(bad) + '.sha256')
        cfg = s.config(f'bad_{how}', bad, bad)
        t0 = time.time()
        s.start(cfg, f'bad_{how}')
        svc = d.set_mode(FRONT, timeout=30.0)
        time.sleep(2.0)
        t1 = time.time()
        time.sleep(3.0)
        got = d.between(t1, time.time())
        n = len(got)
        row = dict(alive=s.alive(), mode_ok=svc, warn=s.log_count('색 검출기로 대신함'), hz=round(n / 3.0, 2), found=sum(1 for x in got if x[3]), line=s.log_lines('색 검출기로 대신함', 1), boot_line=s.log_lines('검출기:', 1), boot_s=round(t1 - t0, 1))
        if not row['alive']:
            row['log'] = s.log.read_text(errors='replace').splitlines()[-5:]
        row['pass'] = row['alive'] and row['warn'] >= 1 and n > 0
        res[how] = row
        if how != 'flip_nofile':
            ok = ok and row['pass']
        bad.unlink()
        Path(str(bad) + '.sha256').unlink(missing_ok=True)
    s.stop()
    return dict(cases=res, **{'pass': ok})


CASE_TEXT = {
    'F1': ('앞 D435 중단 · 재개', '결과 발행 멈춤', '받는 쪽 300 ms 규칙으로 값 버림(코드)'),
    'F2': ('stamp 지연 0.2 · 0.4 s', '0.3 s 넘으면 결과 없음', '받는 쪽 300 ms 규칙(코드)'),
    'F3': ('같은 stamp 반복', '새 결과 없음', '받는 쪽 300 ms 규칙으로 값 만료(코드)'),
    'F4': ('stamp 앞뒤 번갈아', '결과 stamp 역행 없음', '해당 없음'),
    'F5': ('깊이만 중단', '3D 퍽 · 목표 없음', '목표 없음으로 처리(코드 미확인)'),
    'F6': ('odom TF 중단', 'WARN, base_link 기준 추적', '해당 없음'),
    'F7': ('팔 카메라 중단 · 재개', '결과 발행 멈춤', '받는 쪽 300 ms 규칙(코드)'),
    'F8': ('깨진 영상', '죽지 않음', '해당 없음'),
    'F9': ('노드 1 s 정지', '재개 뒤 오래된 결과 없음', '받는 쪽 300 ms 규칙(코드)'),
    'F10': ('정지 · 실행 반복(과부하 모사)', '나이 > 0.3 s ≤ 0.5 %', '받는 쪽 300 ms 규칙(코드)'),
    'F11': ('프로세스 강제 종료', '결과 멈춤, 재기동 후 복귀', '스킬 시간 초과(코드), 재기동은 사람'),
    'F12': ('엔진 파일 손상 기동', 'WARN, 색 검출기 대체', '해당 없음'),
}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--render', required=True)
    ap.add_argument('--share', required=True)
    ap.add_argument('--urdf', required=True)
    ap.add_argument('--engine480', default='')
    ap.add_argument('--engine384', default='')
    ap.add_argument('--d435_id', default='w101_b1_d0')
    ap.add_argument('--arm_id', default='w101_b1_a_puck_00_0')
    ap.add_argument('--fps', type=float, default=15.0)
    ap.add_argument('--cases', default='F1,F2,F3,F4,F5,F6,F7,F8,F9,F10,F11,F12')
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=False)
    rclpy.init()
    d = Driver(a)
    ex = SingleThreadedExecutor()
    ex.add_node(d)
    threading.Thread(target=ex.spin, daemon=True).start()
    s = Stack(a, out)
    results = {}
    main_e = a.engine480 or '""'
    arm_e = a.engine384 or '""'
    cfg = s.config('main', main_e, arm_e)
    try:
        d.thread.start()
        s.start(cfg, 'main')
        results['mode_ok'] = d.set_mode(FRONT)
        results['boot_line'] = s.log_lines('검출기:', 1)
        time.sleep(1.0)
        fns = dict(F1=lambda: f1(d, s), F2=lambda: f2(d, s), F3=lambda: f3(d, s), F4=lambda: f4(d, s), F5=lambda: f5(d, s), F6=lambda: f6(d, s),
                   F7=lambda: f7(d, s), F8=lambda: f8(d, s, cfg), F9=lambda: f9(d, s), F10=lambda: f10(d, s), F11=lambda: f11(d, s, cfg), F12=lambda: f12(d, s, a, out))
        for c in a.cases.split(','):
            t0 = time.time()
            try:
                if not s.alive():
                    s.stop()
                    s.start(cfg, f'before_{c}')
                    d.set_mode(FRONT)
                    time.sleep(1.0)
                results[c] = fns[c]()
            except Exception as e:  # 도구 오류와 판정 불통과를 구분한다
                results[c] = {'tool_error': repr(e), 'pass': False}
            results[c]['took_s'] = round(time.time() - t0, 1)
            print(c, json.dumps(results[c], ensure_ascii=False), flush=True)
            json.dump(results, open(out / 'result.json', 'w'), ensure_ascii=False, indent=1)
    finally:
        d.running = False
        d.thread.join(timeout=2.0)
        s.close()
    with open(out / 'fault_cases.csv', 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['fault', 'expected_diagnostic', 'expected_controller_action', 'measured_response_ms', 'result', 'evidence'])
        for c in a.cases.split(','):
            r = results.get(c, {})
            ms = r.get('stop_s', r.get('recover_s'))
            w.writerow([f'{c} {CASE_TEXT[c][0]}', CASE_TEXT[c][1], CASE_TEXT[c][2], '' if ms is None else round(ms * 1000), 'PASS' if r.get('pass') else 'FAIL', 'result.json'])
    os._exit(0)  # 결과를 모두 쓴 뒤 끝낸다. rclpy 종료 순서 때문에 코어 덤프가 나는 것을 피한다


if __name__ == '__main__':
    main()
