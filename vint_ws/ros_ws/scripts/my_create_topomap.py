#!/usr/bin/env python
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

# IMAGE_TOPIC = "/Hololens/Image"
IMAGE_TOPIC = "/ble_controler/Image"
TOPOMAP_IMAGES_DIR = "../topomaps/images"
obs_img = None

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
    global obs_img
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
    print("Registered with master node. Waiting for images...")
    i = 0
    start_time = float("inf")
    while rclpy.ok():
        if obs_img is not None:
            # ROS2 图片格式 bgr8, PIL 图片格式 rgb
            # 转换图片格式
            # b,g,r = obs_img.split()
            # rgb_image = PILImage.merge('RGB',(r,g,b))
            # rgb_image.save(os.path.join(topomap_name_dir, f"{i}.png"))
            obs_img.save(os.path.join(topomap_name_dir, f"{i}.png"))
            print(f"Saved image {i}.png")
            i += 1
            obs_img = None
            start_time = Clock().now().nanoseconds/(10**9)
            rate.sleep()
        if Clock().now().nanoseconds/(10**9) - start_time > 5:
            print(f"Topic {IMAGE_TOPIC} not publishing anymore. Shutting down...")
            break
    
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