"""목표를 유지하며 경로 불가/관측 무효에는 대기하는 Nav2 연결 시제품."""
import time
from collections import Counter
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool,String
from nav2_msgs.action import ComputePathToPose,FollowPath

class NavigationCoordinator(Node):
    def __init__(self):
        super().__init__('navigation_coordinator_prototype')
        self.planner=ActionClient(self,ComputePathToPose,'/compute_path_to_pose');self.controller=ActionClient(self,FollowPath,'/follow_path')
        self.target=None;self.health=False;self.health_t=None;self.stop=True;self.stop_t=None;self.state='IDLE';self.path_valid=False;self.planning=False;self.last_plan=0.;self.follow=None;self.follow_pending=False;self.follow_result=None;self.cancel_pending=False;self.counts=Counter();self.plan_sizes=[]
        self.path_pub=self.create_publisher(Bool,'/navigation/path_valid',10);self.status=self.create_publisher(String,'/navigation/state',10)
        self.create_subscription(PoseStamped,'/navigation/goal_pose',self.on_goal,10)
        self.create_subscription(Bool,'/navigation/observations_valid',self.on_health,10);self.create_subscription(Bool,'/perception/obstacle_stop',self.on_stop,10);self.create_timer(.1,self.tick)
    def on_goal(self,msg):
        if self.target is not None and self.state not in ('SUCCEEDED','IDLE'):
            self.counts['new_goal_while_active_rejected']+=1;return
        self.target=msg;self.state='WAIT_INPUT';self.path_valid=False;self.last_plan=0.;self.counts['goals']+=1
    def on_health(self,msg):self.health=msg.data;self.health_t=time.monotonic()
    def on_stop(self,msg):self.stop=msg.data;self.stop_t=time.monotonic()
    def inputs_ok(self):
        now=time.monotonic()
        return self.health and not self.stop and self.health_t is not None and self.stop_t is not None and now-self.health_t<=.30 and now-self.stop_t<=.30
    def cancel(self):
        if self.follow is not None and not self.cancel_pending:
            self.cancel_pending=True;self.counts['cancels']+=1;self.follow.cancel_goal_async().add_done_callback(self.on_cancel)
    def on_cancel(self,future):
        self.follow=None;self.follow_result=None;self.cancel_pending=False;self.follow_pending=False
    def tick(self):
        if self.target is None:self.state='IDLE';self.path_valid=False
        elif self.state=='SUCCEEDED':self.path_valid=False
        elif not self.inputs_ok():self.state='WAIT_INPUT';self.path_valid=False;self.cancel()
        else:
            if self.follow_result is not None and self.follow_result.done() and not self.cancel_pending:
                result=self.follow_result.result();self.follow=None;self.follow_result=None
                if result.status==4:self.state='SUCCEEDED';self.path_valid=False;self.counts['success']+=1
                else:self.path_valid=False;self.state='WAIT_REPLAN';self.counts['follow_failed']+=1
            if self.state!='SUCCEEDED' and not self.planning and time.monotonic()-self.last_plan>=.5 and self.planner.server_is_ready():
                self.planning=True;self.last_plan=time.monotonic();request=ComputePathToPose.Goal();request.goal=self.target;request.goal.header.stamp=self.get_clock().now().to_msg();request.use_start=False;request.planner_id='GridBased';self.counts['plans']+=1;self.planner.send_goal_async(request).add_done_callback(self.on_plan_handle)
        self.path_pub.publish(Bool(data=self.path_valid));self.status.publish(String(data=self.state))
    def on_plan_handle(self,future):
        handle=future.result()
        if not handle.accepted:self.planning=False;self.path_valid=False;self.state='WAIT_PATH';return
        handle.get_result_async().add_done_callback(self.on_plan_result)
    def on_plan_result(self,future):
        self.planning=False;result=future.result()
        if self.state=='SUCCEEDED' or not self.inputs_ok():return
        if result.status!=4 or not result.result.path.poses:
            self.path_valid=False;self.state='WAIT_PATH';self.counts['no_path']+=1;self.cancel();return
        self.plan_sizes.append(len(result.result.path.poses));self.path_valid=True
        if self.follow is None and not self.follow_pending and not self.cancel_pending:
            goal=FollowPath.Goal();goal.path=result.result.path;goal.controller_id='FollowPath';goal.goal_checker_id='goal_checker';goal.progress_checker_id='progress_checker';goal.path_handler_id='PathHandler';self.follow_pending=True;self.controller.send_goal_async(goal).add_done_callback(self.on_follow_handle)
        self.state='FOLLOWING'
    def on_follow_handle(self,future):
        self.follow_pending=False;self.follow=future.result()
        if not self.follow.accepted:self.follow=None;self.path_valid=False;self.state='WAIT_REPLAN';return
        self.counts['follow_goals']+=1;self.follow_result=self.follow.get_result_async()
        if not self.inputs_ok() or not self.path_valid:self.cancel()
