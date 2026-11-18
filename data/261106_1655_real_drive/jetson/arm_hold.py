"""팔 토크를 켜고 지금 자세에서 HOLD 로 둔다(움직이는 명령 없음). 끝나면 송신을 멈춰 펌웨어가 토크를 유지한다."""
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
print("상태", c.state, c.msg)
if c.state == "RUN":
    time.sleep(0.5)
    c.request("stop")
    t = time.monotonic()
    while c.state != "STOPPED" and time.monotonic() - t < 2:
        time.sleep(0.05)
c.close()
time.sleep(0.4)
l = tool.Link(c.port); s = l.wait_state(1.0); l.ser.close()
print("끝", tool.err_text(s["error"]) if s else "STATE 없음")
