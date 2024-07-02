#!/usr/bin/env python3

import pandas as pd
import numpy as np
import os

# ROS
import rclpy
from rclpy.node import Node
import rclpy.time
import tf2_ros
from geometry_msgs.msg import TransformStamped 
from tf_transformations import quaternion_matrix

# file = pd.read_csv("pose.csv")
# print(file)
# print(file.values[0][0])
# print(file.values[-1][0])
# print(file.values.shape)
# sub_target_pose = file.values[1][0:3]
# print(sub_target_pose)

# path = "/root/vint_ws/ros_ws/topomaps/images/pose_image_min_bag9"
# topomap_file_name = sorted(os.listdir(path))
# topomap_file_name.remove("poses.csv")
# topomap_file_name = sorted(topomap_file_name, key=lambda x: int(x.split(".")[0]))
# print(topomap_file_name)
# print(type(topomap_file_name))
# print(len(topomap_file_name))

class TransformListener(Node):
    def __init__(self):
        super().__init__('transform_listener')
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

    def get_transform(self, target_frame, source_frame):
        try:
            now = rclpy.time.Time()
            transform = self.tfBuffer.lookup_transform(target_frame, source_frame, now)
            return transform
        except Exception as e:
            print(e)
            return None
    
    def get_transform_matrix(self, target_frame, source_frame, vector):
        transform = self.get_transform(target_frame, source_frame)
        if transform is None:
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        matrix = quaternion_matrix([rotation.x, rotation.y, rotation.z, rotation.w])[:3,:3]
        transform_vector =  np.dot(matrix, vector) 
        return transform_vector
    
if __name__=="__main__":
    rclpy.init()
    listener = TransformListener()
    target_frame = "map"
    source_frame = "baselink"
    vector = np.array([1.0, 0.0, 0.0])
    transform_vector = listener.get_transform_matrix(target_frame, source_frame, vector)
    print(transform_vector)
    rclpy.shutdown()