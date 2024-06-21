#!/usr/bin/env python
'''
for ros2 foxy
before run this script, you should run the following command:
1) source /opt/ros/foxy/setup.bash
'''

import matplotlib.pyplot as plt
import os
from typing import Tuple, Sequence, Dict, Union, Optional, Callable
import numpy as np
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

import matplotlib.pyplot as plt
import yaml

import threading

# ROS
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from utils import msg_to_pil, to_numpy, transform_images, load_model, pil_to_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# torch
from vint_train.training.train_utils import get_action
import torch
from PIL import Image as PILImage
from PIL import ImageDraw
import numpy as np
import argparse
import yaml
import time



# UTILS
IMAGE_TOPIC = "/Hololens/Image"
# IMAGE_TOPIC = "/ble_controler/Image"
WAYPOINT_TOPIC = "/waypoint"
SAMPLED_ACTIONS_TOPIC = "/sampled_actions"

# CONSTANTS
ROBOT_CONFIG_PATH ="../config/robot.yaml"
MODEL_CONFIG_PATH = "../config/models.yaml"

with open(ROBOT_CONFIG_PATH, "r") as f:
    robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
RATE = robot_config["frame_rate"] 

# GLOBALS
context_queue = []
context_size = None  
index = 0

# Load the model 
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

class MyNode(Node):
    def __init__(self,name):
        super().__init__(name)
        self.image_curr_msg = self.create_subscription(Image,IMAGE_TOPIC,self.callback_obs,1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, WAYPOINT_TOPIC, 1)
        self.sampled_actions_pub = self.create_publisher(Float32MultiArray, SAMPLED_ACTIONS_TOPIC, 1)
        self.naction_image_pub  = self.create_publisher(Image, "/naction_image", 1)
        self.naction_image = None
        self.naction_path_pub = self.create_publisher(Path, "/naction_path", 1)

    def callback_obs(self,msg):
        obs_img = msg_to_pil(msg)
        if context_size is not None:
            if len(context_queue) < context_size + 1:
                context_queue.append(obs_img)
            else:
                context_queue.pop(0)
                context_queue.append(obs_img)
        self.naction_image = obs_img
        print("Received image observation")



def main(args: argparse.Namespace):
    global context_size, index

    # load model parameters
    with open(MODEL_CONFIG_PATH, "r") as f:
        model_paths = yaml.safe_load(f)

    model_config_path = model_paths[args.model]["config_path"]
    with open(model_config_path, "r") as f:
        model_params = yaml.safe_load(f)

    context_size = model_params["context_size"]

    # load model weights
    ckpth_path = model_paths[args.model]["ckpt_path"]
    if os.path.exists(ckpth_path):
        print(f"Loading model from {ckpth_path}")
    else:
        raise FileNotFoundError(f"Model weights not found at {ckpth_path}")
    model = load_model(
        ckpth_path,
        model_params,
        device,
    )
    model = model.to(device)
    model.eval()

    num_diffusion_iters = model_params["num_diffusion_iters"]
    noise_scheduler = DDPMScheduler(
        num_train_timesteps=model_params["num_diffusion_iters"],
        beta_schedule='squaredcos_cap_v2',
        clip_sample=True,
        prediction_type='epsilon'
    )

    # ROS
    rclpy.init(args=None)
    node = MyNode("EXPLORATION")
    rate = node.create_rate(RATE)
    print("Node Initialized. Waiting for image observations...")

    # spin in a separate thread
    spin_thread =  threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()


    while rclpy.ok():
        # Exploration mode
        #waypoint_msg = Float32MultiArray()
        if (
                len(context_queue) > model_params["context_size"]
            ):

            obs_images = transform_images(context_queue, model_params["image_size"], center_crop=False)
            # obs_images = torch.split(obs_images, 3, dim=1)
            # obs_images = torch.cat(obs_images, dim=1)
            obs_images = obs_images.to(device)
            fake_goal = torch.randn((1, 3, *model_params["image_size"])).to(device)
            mask = torch.ones(1).long().to(device) # ignore the goal

            # infer action
            with torch.no_grad():
                # encoder vision features
                obs_cond = model('vision_encoder', obs_img=obs_images, goal_img=fake_goal, input_goal_mask=mask)
                
                # (B, obs_horizon * obs_dim)
                if len(obs_cond.shape) == 2:
                    obs_cond = obs_cond.repeat(args.num_samples, 1)
                else:
                    obs_cond = obs_cond.repeat(args.num_samples, 1, 1)
                
                # initialize action from Gaussian noise
                noisy_action = torch.randn(
                    (args.num_samples, model_params["len_traj_pred"], 2), device=device)
                naction = noisy_action

                # init scheduler
                noise_scheduler.set_timesteps(num_diffusion_iters)

                start_time = time.time()
                for k in noise_scheduler.timesteps[:]:
                    # predict noise
                    noise_pred = model(
                        'noise_pred_net',
                        sample=naction,
                        timestep=k,
                        global_cond=obs_cond
                    )

                    # inverse diffusion step (remove noise)
                    naction = noise_scheduler.step(
                        model_output=noise_pred,
                        timestep=k,
                        sample=naction
                    ).prev_sample
                print("time elapsed:", time.time() - start_time)

            naction = to_numpy(get_action(naction))

            #可视化8条naction，将黑色图片self.naction_image 发布出去
            color_list = ["red","green","blue","yellow","black","white","purple","orange"]
            color_index = 0
            if node.naction_image != None:
                origin_x = int(node.naction_image.width/2)
                origin_y = int(node.naction_image.height*3/4)
                for items in naction:
                    for item in items:
                        x1 = int(origin_x - item[1]*10)
                        y1 = int(origin_y - item[0]*10)
                        x2 = x1 + 1
                        y2 = y1 + 1
                        #在node.naction_image 上画点
                        image_draw = ImageDraw.Draw(node.naction_image)
                        image_draw.rectangle(xy=(x1,y1,x2,y2),fill=color_list[color_index%(len(color_list))])
                    color_index += 1
                image_msg = pil_to_msg(node.naction_image)
                node.naction_image_pub.publish(image_msg)



            # print(type(naction))
            # print(naction.shape)
            # print(naction)

            # print(type(naction.flatten()))
            # print(naction.flatten().shape)
            # print(naction.flatten())

            # try:
            #     action_msg_data = np.concatenate((np.array([0]), naction.flatten()))
            #     sampled_actions_msg = Float32MultiArray(data=action_msg_data)
            #     node.sampled_actions_pub.publish(sampled_actions_msg)
            # except Exception as e:
            #     print(e)

            naction = naction[0] # change this based on heuristic

            # 选取一条最好的naction, 通过消息类型nav_msgs/path 可视化在rviz上
            path = Path()   
            path.header.frame_id = "baselink"
            path.header.stamp = node.get_clock().now().to_msg()
            for i in range(naction.shape[0]):
                pose = PoseStamped()
                pose.header.frame_id = "baselink"
                pose.header.stamp = node.get_clock().now().to_msg()
                pose.pose.position.x = naction[i][0]*0.2
                pose.pose.position.y = naction[i][1]*0.2
                pose.pose.orientation.w = 1.0
                path.poses.append(pose)
            node.naction_path_pub.publish(path)


            # 将 naction 写到csv格式文件中
            # with open("vis/naction"+str(index)+".csv", "w") as f:
            #     for i in range(naction.shape[0]):
            #     #遍历naction的每一个元素
            #         for j in range(naction.shape[1]):
            #             f.write(str(naction[i][j]))
            #             f.write(",")
            #         f.write("\n")
            # index += 1

            #使用opencv将naction的点绘制到图片上


            chosen_waypoint = naction[args.waypoint]

            if model_params["normalize"]:
                chosen_waypoint *= (MAX_V / RATE)
            try:
                waypoint_msg_data = chosen_waypoint
                waypoint_msg = Float32MultiArray(data=waypoint_msg_data)
                node.waypoint_pub.publish(waypoint_msg)
                print("Published waypoint")
            except Exception as e:
                print(e)
        rate.sleep()
    
    rclpy.shutdown()
    spin_thread.join()
    print("Node shutdown")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNM DIFFUSION EXPLORATION on the locobot")
    parser.add_argument(
        "--model",
        "-m",
        default="nomad",
        type=str,
        help="model name (hint: check ../config/models.yaml) (default: nomad)",
    )
    parser.add_argument(
        "--waypoint",
        "-w",
        default=2, # close waypoints exihibit straight line motion (the middle waypoint is a good default)
        type=int,
        help=f"""index of the waypoint used for navigation (between 0 and 4 or 
        how many waypoints your model predicts) (default: 2)""",
    )
    parser.add_argument(
        "--num-samples",
        "-n",
        default=8,
        type=int,
        help=f"Number of actions sampled from the exploration model (default: 8)",
    )
    args = parser.parse_args()
    print(f"Using {device}")
    main(args)    