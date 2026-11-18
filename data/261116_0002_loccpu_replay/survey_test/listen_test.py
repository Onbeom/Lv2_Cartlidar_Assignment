#!/usr/bin/env python3
"""perception_survey listen() 시험 (PC, 11-16): start → finish → start → reset 순서로 불러,
① TransformListener.unregister() 가 오류 없이 도는지 ② 다시 start 한 뒤 TF 를 받아 퍽 관측이 받아들여지는지 본다.
퍽은 map → base_link 고정 TF(/tf_static) · /localization/status OK 와 함께 base_link 기준 가짜 퍽 2 개를 0.1 s 마다 보냄."""
import json
import threading
import time

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from robot_interfaces.msg import Puck, PuckArray
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import StaticTransformBroadcaster

from perception_survey.core import BOX_CENTERS
from perception_survey.node import SurveyNode

rclpy.init()
survey = SurveyNode()
survey.output = '/tmp/claude-1000/-home-pa4-git-lv2-next-module/f89e5d4b-7cc0-4ca9-b419-c0309f318e34/scratchpad/survey_test_out.json'
aux = rclpy.create_node('survey_test_aux')
latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
pub_loc = aux.create_publisher(String, '/localization/status', latched)
pub_pk = aux.create_publisher(PuckArray, '/perception/pucks', 10)
stb = StaticTransformBroadcaster(aux)
t = TransformStamped()
t.header.stamp = aux.get_clock().now().to_msg()
t.header.frame_id, t.child_frame_id = 'map', 'base_link'
BX, BY = BOX_CENTERS[0]   # 1 번 상자 양옆 칸(±0.04 m)에 퍽이 오게: 퍽은 base_link (0.34, 0) · (0.26, 0)
t.transform.translation.x, t.transform.translation.y = BX - 0.30, BY
t.transform.rotation.w = 1.0
stb.sendTransform(t)
ex = MultiThreadedExecutor(num_threads=4)
ex.add_node(survey)
ex.add_node(aux)
threading.Thread(target=ex.spin, daemon=True).start()


def send_pucks(sec):
    end = time.time() + sec
    while time.time() < end:
        pub_loc.publish(String(data='OK'))
        m = PuckArray()
        m.header.stamp = aux.get_clock().now().to_msg()
        m.header.frame_id = 'base_link'
        for x, y, c in ((0.34, 0.0, 1), (0.26, 0.0, 2)):
            p = Puck()
            p.tag_id, p.color, p.shape, p.camera, p.confidence = -1, c, 1, 0, 0.9
            p.pose.position.x, p.pose.position.y, p.pose.position.z = x, y, 0.03
            p.pose.orientation.w = 1.0
            m.pucks.append(p)
        pub_pk.publish(m)
        time.sleep(0.1)


def call(fn, name):
    res = fn(Trigger.Request(), Trigger.Response())
    print(f'{name}: success={res.success} · listener={"켜짐" if survey.listener else "꺼짐"} · {res.message[:60]}')


def report(tag):
    n = sum(t['n_obs'] for t in survey.core.tracks)
    print(f'  [{tag}] 받아들인 관측 {n} · 거부 {dict(survey.core.rejected)}')


time.sleep(1.0)
call(survey.start, 'start 1')
send_pucks(2.0)
report('start 1 뒤')
r1 = dict(survey.core.rejected)
for fn, nm in ((survey.finish, 'finish (조사 덜 됨 → 실패 예상, listener 그대로)'), (survey.reset, 'reset (listener 끔 = unregister)')):
    try:
        call(fn, nm)
    except Exception as e:   # noqa: BLE001
        print(f'  {nm} 오류: {type(e).__name__}: {e}')
send_pucks(0.5)
call(survey.start, 'start 2')
send_pucks(2.0)
report('start 2 뒤')
call(survey.reset, 'reset')
print('끝 — 거부 사유 · 수는 snapshot rejected (start 마다 초기화)')
ex.shutdown()
rclpy.shutdown()
