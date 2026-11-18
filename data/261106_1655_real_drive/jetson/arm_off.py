"""팔 토크 끄기(움직이는 명령 없음): 지금 자세로 토크를 잡은 뒤 TORQUE_ON 없는 명령으로 끈다. 팔이 떨어지니 받칠 것."""
import time
from teleop_common import ROLE_ARM, find_port, tool
import arm_control
c = arm_control.ArmController(find_port(ROLE_ARM))
t = time.monotonic()
while c.snapshot()["last"] is None and time.monotonic() - t < 3:
    time.sleep(0.05)
time.sleep(0.3)
c.request("start")
t = time.monotonic()
while c.state != "RUN" and time.monotonic() - t < 4:
    time.sleep(0.05)
c.request("off")
t = time.monotonic()
while c.state != "IDLE" and time.monotonic() - t < 3:
    time.sleep(0.05)
print("상태", c.state, c.msg)
c.close()
time.sleep(0.3)
l = tool.Link(c.port); s = l.wait_state(1.0); l.ser.close()
print("끝", tool.err_text(s["error"]) if s else "STATE 없음")
