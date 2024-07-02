#!/usr/bin/env python3
import argparse
import os
# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
import quaternion
import numpy as np

#pose topic 
POSE_TOPIC = "ble_controler/pose"




class PoseWriter(Node):
    def __init__(self, name):
        super().__init__(name)
        self.poseStamped_sub = self.create_subscription(PoseStamped,"/ble_controler/pose",self.pose_callback,10)
        self.file = open("pose.csv","w")

    # 四元素转化成欧拉角
    def quaternion_to_euler(self, poseStamp: PoseStamped):
        q = np.quaternion(
            poseStamp.pose.orientation.w,
            poseStamp.pose.orientation.x,
            poseStamp.pose.orientation.y,
            poseStamp.pose.orientation.z)
        euler = quaternion.as_euler_angles(q)

        return euler
    
    def pose_callback(self, msg: PoseStamped):
        euler = self.quaternion_to_euler(msg)
        print("x: ", msg.pose.position.x, "y: ", msg.pose.position.y, "yaw: ", euler[-1])
        if self.file is not None:
            self.file.write(str(msg.pose.position.x) + "," + str(msg.pose.position.y) + "," + str(euler[-1]) + ",\n")

def main(args: argparse.Namespace):
    #ROS
    rclpy.init()
    pose_writer = PoseWriter("pose_writer")
    print("Node initialized, waiting for pose....")
    rclpy.spin(pose_writer)
    pose_writer.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    args = parser.parse_args()
    main(args)