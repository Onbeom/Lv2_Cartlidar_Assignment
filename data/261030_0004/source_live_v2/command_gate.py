"""Nav2 명령의 정지·입력 유효성 우선 게이트. 개인 모의 검증용."""
import math,time
from collections import Counter
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool,String
from rclpy.node import Node

class CommandGate(Node):
    def __init__(self,require_path_valid=True):
        super().__init__('navigation_command_gate')
        self.command=None;self.cmd_t=None;self.health=False;self.health_t=None;self.stop=True;self.stop_t=None;self.clear_count=0;self.counts=Counter();self.reason='startup';self.require_path_valid=require_path_valid;self.path_valid=False;self.path_t=None
        self.pub=self.create_publisher(TwistStamped,'/cmd_vel_nav',10);self.state=self.create_publisher(String,'/navigation/command_gate_state',10)
        self.create_subscription(TwistStamped,'/cmd_vel_nav_raw',self.on_cmd,10)
        self.create_subscription(Bool,'/navigation/observations_valid',self.on_health,10)
        self.create_subscription(Bool,'/perception/obstacle_stop',self.on_stop,10)
        self.create_subscription(Bool,'/navigation/path_valid',self.on_path,10)
        self.create_timer(.05,self.tick)
    def on_cmd(self,msg):self.command=msg;self.cmd_t=time.monotonic()
    def on_health(self,msg):
        self.health=msg.data;self.health_t=time.monotonic()
        if not msg.data:self.clear_count=0
    def on_stop(self,msg):
        self.stop=msg.data;self.stop_t=time.monotonic()
        self.clear_count=self.clear_count+1 if not msg.data and self.health else 0
    def on_path(self,msg):self.path_valid=msg.data;self.path_t=time.monotonic()
    def tick(self):
        now=time.monotonic();reason='clear'
        if self.stop_t is None or now-self.stop_t>.30:reason='stop_input_stale'
        elif self.stop:reason='obstacle_stop'
        elif self.health_t is None or now-self.health_t>.30 or not self.health:reason='observations_invalid'
        elif self.require_path_valid and (not self.path_valid or self.path_t is None or now-self.path_t>.30):reason='path_invalid'
        elif self.clear_count<3:reason='release_pending'
        elif self.cmd_t is None or now-self.cmd_t>.25:reason='command_stale'
        elif not all(math.isfinite(v) for v in (self.command.twist.linear.x,self.command.twist.linear.y,self.command.twist.angular.z)):reason='invalid_command'
        msg=TwistStamped();msg.header.stamp=self.get_clock().now().to_msg();msg.header.frame_id='base_link'
        if reason=='clear':
            msg.twist.linear.x=max(0.,min(.15,self.command.twist.linear.x));msg.twist.angular.z=max(-.6,min(.6,self.command.twist.angular.z))
        else:self.clear_count=0 if reason in ('stop_input_stale','obstacle_stop','observations_invalid') else self.clear_count
        self.pub.publish(msg);self.state.publish(String(data=reason));self.reason=reason;self.counts[reason]+=1
