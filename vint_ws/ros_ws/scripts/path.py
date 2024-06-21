#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

class Node_path(Node):
    def __init__(self,name):
        super().__init__(name)
        self.poseStamped_sub = self.create_subscription(PoseStamped,"/ble_controler/pose",self.pose_callback,10)
        self.path_pub = self.create_publisher(Path,"/vis/path",1)
        self.path_msg = Path()


    def pose_callback(self,msg : PoseStamped):
        # print("sub PoseStamped")
        temp_pose = msg
        self.path_msg.poses.append(temp_pose)
        self.path_pub.publish(self.path_msg)


if __name__=="__main__":

    #ROS
    rclpy.init()
    node = Node_path("VIS_PATH")
    node.path_msg.header.frame_id = "map"
    
    print("Node initialized, waiting for pose....")
    rclpy.spin(node=node)
    node.destroy_node()
    rclpy.shutdown()


