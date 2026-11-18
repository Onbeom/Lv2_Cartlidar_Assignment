"""깊이·지도·촬영시각 TF의 유효성을 명령 게이트에 전달하는 시제품."""
import time,math
from collections import Counter
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Image,CameraInfo
from std_msgs.msg import Bool,String
from nvblox_msgs.msg import DistanceMapSlice
from tf2_ros import Buffer,TransformListener,TransformException

class ObservationMonitor(Node):
    def __init__(self):
        super().__init__('navigation_observation_monitor')
        self.info=None;self.info_t=None;self.depth=None;self.depth_t=None;self.map=None;self.map_t=None;self.valid_fraction=0.;self.counts=Counter();self.reason='startup';self.last_valid_depth_stamp=None
        self.tf=Buffer();self.listener=TransformListener(self.tf,self)
        self.pub=self.create_publisher(Bool,'/navigation/observations_valid',10);self.status=self.create_publisher(String,'/navigation/observation_state',10)
        self.create_subscription(Image,'/dynamic_test/depth',self.on_depth,qos_profile_sensor_data)
        self.create_subscription(DistanceMapSlice,'/nvblox_probe/combined_map_slice',self.on_map,qos_profile_sensor_data)
        self.create_subscription(CameraInfo,'/dynamic_test/camera_info',self.on_info,qos_profile_sensor_data)
        self.create_timer(.05,self.tick)
    def on_depth(self,msg):
        self.depth=msg;self.depth_t=time.monotonic();self.valid_fraction=0.
        if msg.encoding not in ('16UC1','32FC1') or not msg.width or not msg.height:return
        dtype=np.dtype(('>' if msg.is_bigendian else '<')+('u2' if msg.encoding=='16UC1' else 'f4'))
        if msg.step<msg.width*dtype.itemsize or len(msg.data)<msg.height*msg.step:return
        a=np.ndarray((msg.height,msg.width),dtype=dtype,buffer=msg.data,strides=(msg.step,dtype.itemsize));self.valid_fraction=float((np.isfinite(a)&(a>0)).mean())
    def on_info(self,msg):self.info=msg;self.info_t=time.monotonic()
    def on_map(self,msg):self.map=msg;self.map_t=time.monotonic()
    def tick(self):
        now=time.monotonic();ros_now=self.get_clock().now().nanoseconds*1e-9;reason='clear'
        if self.depth is None or self.depth_t is None or now-self.depth_t>.30:reason='depth_stale'
        elif self.valid_fraction<.05:reason='depth_invalid'
        elif self.info is None or self.info_t is None or now-self.info_t>.30:reason='camera_info_stale'
        elif self.info.header.frame_id!=self.depth.header.frame_id or (self.info.width,self.info.height)!=(self.depth.width,self.depth.height) or not np.isfinite(self.info.k).all() or min(self.info.k[0],self.info.k[4])<=0:reason='camera_info_invalid'
        elif self.map is None or self.map_t is None or now-self.map_t>.50:reason='map_stale'
        elif not self.map.width or not self.map.height or len(self.map.data)!=self.map.width*self.map.height or not math.isfinite(self.map.resolution) or self.map.resolution<=0:reason='map_invalid'
        else:
            for message,ttl in [(self.depth,.30),(self.map,.50)]:
                age=ros_now-message.header.stamp.sec-message.header.stamp.nanosec*1e-9
                if age<-.05 or age>ttl:reason='stamp_invalid';break
            if reason=='clear':
                try:
                    self.tf.lookup_transform('map',self.depth.header.frame_id,Time.from_msg(self.depth.header.stamp))
                    self.last_valid_depth_stamp=self.depth.header.stamp.sec+self.depth.header.stamp.nanosec*1e-9
                except TransformException:
                    if self.last_valid_depth_stamp is None or ros_now-self.last_valid_depth_stamp>.30:reason='camera_tf_unavailable'
                    else:self.counts['waiting_tf_previous_valid_observation']+=1
            if reason=='clear':
                try:self.tf.lookup_transform('map',self.map.header.frame_id,Time.from_msg(self.map.header.stamp))
                except TransformException:reason='map_tf_unavailable'
            if reason=='clear' and not any(np.isfinite(v) and v!=self.map.unknown_value for v in self.map.data):reason='map_unknown'
        self.reason=reason;self.counts[reason]+=1;self.pub.publish(Bool(data=reason=='clear'));self.status.publish(String(data=reason))
