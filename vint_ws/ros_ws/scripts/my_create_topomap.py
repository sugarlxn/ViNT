#!/usr/bin/env python3
'''
for ros2 foxy
before run this script, you should run the following command:
1) source /opt/ros/foxy/setup.bash
'''
import argparse
import os
from utils import msg_to_pil
import time
import threading
from PIL import Image as PILImage

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.clock import Clock
from geometry_msgs.msg import PoseStamped
import quaternion
import numpy as np

# IMAGE_TOPIC = "/Hololens/Image"
IMAGE_TOPIC = "/ble_controler/Image"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
POSE_TOPIC = "ble_controler/pose"
obs_img = None
obs_pose = None

class CreateTopomap(Node):
    def __init__(self, name):
        super().__init__(name)
        self.image_curr_msg = self.create_subscription(
            Image,
            IMAGE_TOPIC,
            self.callback_obs,
            10
        )
        self.subgoals_pub = self.create_publisher(
            Image,
            "/subgoals",
            10
        )
        self.poseStamped_sub = self.create_subscription(PoseStamped,"/ble_controler/pose",self.pose_callback,1)

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
        # euler = self.quaternion_to_euler(msg)
        # print("x: ", msg.pose.position.x, "y: ", msg.pose.position.y, "yaw: ", euler[-1])
        obs_pose = np.array([msg.pose.position.x, msg.pose.position.y,msg.pose.orientation.w,msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z])
        # if self.file is not None:
        #     self.file.write(str(msg.pose.position.x) + "," + str(msg.pose.position.y) + "," + str(euler[-1]) + ",\n")

    def callback_obs(self, msg: Image):
        global obs_img
        obs_img = msg_to_pil(msg)

def remove_files_in_dir(dir_path: str):
    for f in os.listdir(dir_path):
        file_path = os.path.join(dir_path, f)
        try:
            if os.path.isfile(file_path) or os.path.islink(file_path):
                os.unlink(file_path)
            elif os.path.isdir(file_path):
                shutil.rmtree(file_path)
        except Exception as e:
            print("Failed to delete %s. Reason: %s" % (file_path, e))




def main(args: argparse.Namespace):
    global obs_img, obs_pose
    rclpy.init()
    node = CreateTopomap("CREATE_TOPOMAP")
    topomap_name_dir = os.path.join(TOPOMAP_IMAGES_DIR, args.dir)
    if not os.path.isdir(topomap_name_dir):
        os.makedirs(topomap_name_dir)
    else:
        print(f"{topomap_name_dir} already exists. Removing previous images...")
        remove_files_in_dir(topomap_name_dir)

    assert args.dt > 0, "dt must be positive"
    rate = node.create_rate(1/args.dt)

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    file = open(os.path.join(topomap_name_dir, "poses.csv"), "w")
    file.write("index,x,y,w,x,y,z,else\n")
    print("Registered with master node. Waiting for images...")
    i = 0
    start_time = float("inf")
    while rclpy.ok():
        if obs_img is not None and obs_pose is not None:
            # ROS2 图片格式 bgr8, PIL 图片格式 rgb
            # 转换图片格式
            # b,g,r = obs_img.split()
            # rgb_image = PILImage.merge('RGB',(r,g,b))
            # rgb_image.save(os.path.join(topomap_name_dir, f"{i}.png"))
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            print(f"Saved image {i}.png")
            file.write(f"{i},{obs_pose[0]},{obs_pose[1]},{obs_pose[2]},{obs_pose[3]},{obs_pose[4]},{obs_pose[5]},\n")
            print(f"Saved pose {i}")
            i += 1
            obs_img = None
            obs_pose = None
            start_time = Clock().now().nanoseconds/(10**9)
            rate.sleep()
        if Clock().now().nanoseconds/(10**9) - start_time > 5:
            print(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            break
    file.close()
    rclpy.shutdown()
    spin_thread.join()
    print("Node shutdown")
        
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description=f"Code to generate topomaps from the {IMAGE_TOPIC} topic"
    )
    parser.add_argument(
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topological map images in ../topomaps/images directory (default: topomap)",
    )
    parser.add_argument(
        "--dt",
        "-t",
        default=1.,
        type=float,
        help=f"time between images sampled from the {IMAGE_TOPIC} topic (default: 3.0)",
    )
    args = parser.parse_args()

    main(args) 