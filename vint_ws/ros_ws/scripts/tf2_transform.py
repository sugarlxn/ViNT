#!/usr/bin/env python3
import math
import sys
from geometry_msgs.msg import TransformStamped, PoseStamped
import numpy as np
import rclpy
import rclpy.logging
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

class TF_transformer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_dynamic_broadcaster = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.posestamped_sub = self.create_subscription(PoseStamped, "/ble_controler/pose", self.posestamped_sub_callback, 1)


    def send_static_tf(self,T : TransformStamped):
        self.tf_static_broadcaster.sendTransform(T)

    def posestamped_sub_callback(self,msg : PoseStamped):
        posestamped_tmp = msg
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "baselink"
        t.transform.translation.x = posestamped_tmp.pose.position.x
        t.transform.translation.y = posestamped_tmp.pose.position.y
        t.transform.translation.z = posestamped_tmp.pose.position.z
        t.transform.rotation.x = posestamped_tmp.pose.orientation.x
        t.transform.rotation.y = posestamped_tmp.pose.orientation.y
        t.transform.rotation.z = posestamped_tmp.pose.orientation.z
        t.transform.rotation.w = posestamped_tmp.pose.orientation.w

        self.tf_dynamic_broadcaster.sendTransform(t)


if __name__=="__main__":
    logger = rclpy.logging.get_logger("logger")
    rclpy.init()
    tf_transformer = TF_transformer("TF_transformer")
    logger.info("TF_transformer init, sending TF transform ...")
    t = TransformStamped()
    t.header.frame_id = "map"
    t.header.stamp = tf_transformer.get_clock().now().to_msg()
    t.child_frame_id = "odom"
    t.transform.translation.x = float(0)
    t.transform.translation.y = float(0)
    t.transform.translation.z = float(0)
    t.transform.rotation.x = float(0)
    t.transform.rotation.y = float(0)
    t.transform.rotation.z = float(0)
    t.transform.rotation.w = float(1)

    tf_transformer.send_static_tf(t)
    
    rclpy.spin(tf_transformer)
    rclpy.shutdown()



