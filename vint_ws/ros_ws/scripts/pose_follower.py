#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import os 
import numpy as np
import quaternion
import threading
from geometry_msgs.msg import Twist
import pandas as pd

#pose topic
POSE_TOPIC = "ble_controler/pose"
CMD_VEL_TOPIC = "ble_controler/cmd_vel"
obs_pose = None
sub_target_pose = None
index = 2

class PoseFollower(Node):
    def __init__(self, name):
        super().__init__(name)
        self.poseStamped_sub = self.create_subscription(PoseStamped,POSE_TOPIC,self.pose_callback,1)
        # self.path_pub = self.create_publisher(Path,"/vis/path",1)
        # self.path_msg = Path()
        self.cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 1)

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
        global obs_pose
        euler = self.quaternion_to_euler(msg)
        print("x: ", msg.pose.position.x, "y: ", msg.pose.position.y, "yaw: ", euler[-1])
        obs_pose = np.array([msg.pose.position.x, msg.pose.position.y, euler[-1]])
        # self.path_msg.poses.append(msg)
        # self.path_pub.publish(self.path_msg)

    def controler(self,obs_pose, sub_target_pose):
        global index
        if len(obs_pose)==3 and len(sub_target_pose)==3:
            x_diff = sub_target_pose[0] - obs_pose[0]
            y_diff = sub_target_pose[1] - obs_pose[1]
            distance_diff = np.sqrt(x_diff**2 + y_diff**2)
            yaw_diff = sub_target_pose[2] - obs_pose[2]
            print("x_diff: ", x_diff, "y_diff: ", y_diff, "yaw_diff: ", yaw_diff)
            if distance_diff < 0.1 and np.abs(yaw_diff) < 0.1:
                #到达sub_target_pose
                index+=1

            elif distance_diff < 0.1 and np.abs(yaw_diff) > 0.1:
                #原地旋转
                twist = Twist()
                twist.linear.x = 0.0
                if(yaw_diff > 0):
                    twist.angular.z = 0.5
                else:
                    twist.angular.z = -0.5
                self.cmd_vel_pub.publish(twist)
                print(twist.linear.x, " ", twist.angular.z)
            else:
                #前进or后退，dx为x方向的距离，dy为y方向的距离
                twist = Twist()
                twist.angular.z = yaw_diff * 1.5
                twist.linear.x = x_diff * 2.5
                self.cmd_vel_pub.publish(twist)
                print(twist.linear.x," ", twist.angular.z)

        

if __name__=="__main__":
    #ROS
    rclpy.init()
    pose_follower = PoseFollower("pose_follower")
    print("Node initialized, waiting for pose....")
    path = "/root/vint_ws/ros_ws/topomaps/images/pose_image_min_bag8_test/poses.csv"
    file = pd.read_csv(path)
    (max_row, max_col) = file.values.shape
    print("max_row: ", max_row, "max_col: ", max_col)

    spin_thread = threading.Thread(target=rclpy.spin, args=(pose_follower,))
    spin_thread.start()

    rate = pose_follower.create_rate(4)
    while rclpy.ok():
        if(index < max_row):
            print("index: ", index)
            sub_target_pose = file.values[index][1:4]
        else:
            print("Reached end of path")
            break
        
        #辅助控制
        if obs_pose is not None and sub_target_pose is not None:
            pose_follower.controler(obs_pose, sub_target_pose)
        
        rate.sleep()

    pose_follower.destroy_node()
    rclpy.shutdown()
    spin_thread.join()
    print("Node destroyed and shutdown")
