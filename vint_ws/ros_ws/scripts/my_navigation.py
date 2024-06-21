#!/usr/bin/env python
'''
for ros2 foxy
before run this script, you should run the following command:
1) source /opt/ros/foxy/setup.bash
'''
import numpy as np
import yaml
import matplotlib.pyplot as plt
import os
from typing import Tuple, Sequence, Dict, Union, Optional, Callable
import threading
import torch
import torch.nn as nn
from diffusers.schedulers.scheduling_ddpm import DDPMScheduler

# ROS
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Float32MultiArray
from utils import msg_to_pil, to_numpy, transform_images, load_model, pil_to_msg
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

from vint_train.training.train_utils import get_action
from PIL import Image as PILImage
from PIL import ImageDraw
import argparse
import time

# UTILS
# IMAGE_TOPIC = "/Hololens/Image"
IMAGE_TOPIC = "/ble_controler/Image"
WAYPOINT_TOPIC = "/waypoint"
SAMPLED_ACTIONS_TOPIC = "/sampled_actions"

# CONSTANTS
TOPOMAP_IMAGES_DIR = "../topomaps/images"
MODEL_WEIGHTS_PATH = "/root/vint_ws/visualnav-transformer-main/train_release/deployment/model_weights/"
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
subgoal = []


# Load the model 
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)


class VisualNav(Node):
    def __init__(self,name):
        super().__init__(name)
        self.image_curr_msg = self.create_subscription(Image, IMAGE_TOPIC, self.callback_obs, 1)
        self.waypoint_pub = self.create_publisher(Float32MultiArray, WAYPOINT_TOPIC, 1)
        self.sampled_actions_pub = self.create_publisher(Float32MultiArray, SAMPLED_ACTIONS_TOPIC, 1)
        self.goal_pub = self.create_publisher(Bool, "/topoplan/reached_goal", 1)
        self.naction_image_pub  = self.create_publisher(Image, "/naction_image", 1)
        self.naction_image = None
        self.naction_path_pub = self.create_publisher(Path, "/naction_path", 1)

    def callback_obs(self,msg):
        obs_img = msg_to_pil(msg)
        # ROS2 图片格式 bgr8, PIL 图片格式 rgb
        # 转换图片格式
        # b,g,r = obs_img.split()
        # rgb_image = PILImage.merge('RGB',(r,g,b))
        if context_size is not None:
            if len(context_queue) < context_size + 1:
                context_queue.append(obs_img)
            else:
                context_queue.pop(0)
                context_queue.append(obs_img)
        self.naction_image = obs_img
        print("Received image observation")

def main(args: argparse.Namespace):
    global context_size

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

    
     # load topomap
    topomap_filenames = sorted(os.listdir(os.path.join(
        TOPOMAP_IMAGES_DIR, args.dir)), key=lambda x: int(x.split(".")[0]))
    topomap_dir = f"{TOPOMAP_IMAGES_DIR}/{args.dir}"
    num_nodes = len(os.listdir(topomap_dir))
    topomap = []
    for i in range(num_nodes):
        image_path = os.path.join(topomap_dir, topomap_filenames[i])
        topomap.append(PILImage.open(image_path))

    closest_node = 0
    assert -1 <= args.goal_node < len(topomap), "Invalid goal index"
    if args.goal_node == -1:
        goal_node = len(topomap) - 1
    else:
        goal_node = args.goal_node
    reached_goal = False    

    # ROS
    rclpy.init(args=None)
    node = VisualNav("visual_nav")
    rate = node.create_rate(RATE)
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    print("VisualNav node initialized, Waiting for image observation ...")

    if model_params["model_type"] == "nomad":
        num_diffusion_iters = model_params["num_diffusion_iters"]
        noise_scheduler = DDPMScheduler(
            num_train_timesteps=model_params["num_diffusion_iters"],
            beta_schedule='squaredcos_cap_v2',
            clip_sample=True,
            prediction_type='epsilon'
        )


    #navigation loop
    while rclpy.ok():
        # Exploartion mode
        chosen_waypoint = np.zeros(4)
        if len(context_queue) > model_params["context_size"]:
            if model_params["model_type"] == "nomad":
                obs_images = transform_images(context_queue, model_params["image_size"], center_crop=False)
                # obs_images = torch.split(obs_images, 3, dim=1)
                # obs_images = torch.cat(obs_images, dim=1) 
                obs_images = obs_images.to(device)
                mask = torch.zeros(1).long().to(device)  

                start = max(closest_node - args.radius, 0)
                end = min(closest_node + args.radius + 1, goal_node)
                goal_image = [transform_images(g_img, model_params["image_size"], center_crop=False).to(device) for g_img in topomap[start:end + 1]]
                goal_image = torch.concat(goal_image, dim=0)

                obsgoal_cond = model('vision_encoder', obs_img=obs_images.repeat(len(goal_image), 1, 1, 1), goal_img=goal_image, input_goal_mask=mask.repeat(len(goal_image)))
                dists = model("dist_pred_net", obsgoal_cond=obsgoal_cond)
                dists = to_numpy(dists.flatten())
                min_idx = np.argmin(dists)
                closest_node = min_idx + start
                print("closest node:", closest_node)
                sg_idx = min(min_idx + int(dists[min_idx] < args.close_threshold), len(obsgoal_cond) - 1)
                obs_cond = obsgoal_cond[sg_idx].unsqueeze(0)

                # infer action
                with torch.no_grad():
                    # encoder vision features
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
                    # print("time elapsed:", time.time() - start_time)

                naction = to_numpy(get_action(naction))
                sampled_actions_msg = Float32MultiArray(data = np.concatenate((np.array([0]), naction.flatten())))
                # print("published sampled actions")
                node.sampled_actions_pub.publish(sampled_actions_msg)
                
                # #可视化8条naction
                # color_list = ["red","green","blue","yellow","black","white","purple","orange"]
                # color_index = 0
                # if node.naction_image != None:
                #     origin_x = int(node.naction_image.width/2)
                #     origin_y = int(node.naction_image.height*3/4)
                #     for items in naction:
                #         for item in items:
                #             x1 = int(origin_x - item[1]*10)
                #             y1 = int(origin_y - item[0]*10)
                #             x2 = x1 + 1
                #             y2 = y1 + 1
                #             #在node.naction_image 上画点
                #             image_draw = ImageDraw.Draw(node.naction_image)
                #             image_draw.rectangle(xy=(x1,y1,x2,y2),fill=color_list[color_index%(len(color_list))])
                #         color_index += 1
                #     image_msg = pil_to_msg(node.naction_image)
                #     node.naction_image_pub.publish(image_msg)

                naction = naction[0] 
                chosen_waypoint = naction[args.waypoint]

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
                
            elif (len(context_queue) > model_params["context_size"]):
                start = max(closest_node - args.radius, 0)
                end = min(closest_node + args.radius + 1, goal_node)
                distances = []
                waypoints = []
                batch_obs_imgs = []
                batch_goal_data = []
                for i, sg_img in enumerate(topomap[start: end + 1]):
                    transf_obs_img = transform_images(context_queue, model_params["image_size"])
                    goal_data = transform_images(sg_img, model_params["image_size"])
                    batch_obs_imgs.append(transf_obs_img)
                    batch_goal_data.append(goal_data)
                    
                # predict distances and waypoints
                batch_obs_imgs = torch.cat(batch_obs_imgs, dim=0).to(device)
                batch_goal_data = torch.cat(batch_goal_data, dim=0).to(device)

                distances, waypoints = model(batch_obs_imgs, batch_goal_data)
                distances = to_numpy(distances)
                waypoints = to_numpy(waypoints)
                # look for closest node
                closest_node = np.argmin(distances)
                # chose subgoal and output waypoints
                if distances[closest_node] > args.close_threshold:
                    chosen_waypoint = waypoints[closest_node][args.waypoint]
                    sg_img = topomap[start + closest_node]
                else:
                    chosen_waypoint = waypoints[min(
                        closest_node + 1, len(waypoints) - 1)][args.waypoint]
                    sg_img = topomap[start + min(closest_node + 1, len(waypoints) - 1)]     
        # RECOVERY MODE
        if model_params["normalize"]:
            chosen_waypoint[:2] *= (MAX_V / RATE)  
        waypoint_msg = Float32MultiArray(data = chosen_waypoint)
        node.waypoint_pub.publish(waypoint_msg)
        reached_goal = (closest_node == goal_node)
        reached_goal_msg = Bool(data = bool(reached_goal))
        node.goal_pub.publish(reached_goal_msg)

        if reached_goal:
            print("Reached goal! Stopping...")
            break
        rate.sleep()
    
    rclpy.shutdown()
    spin_thread.join()
    print("Node shutdown")

if __name__=="__main__":
    parser = argparse.ArgumentParser(
        description="Code to run GNM DIFFUSION EXPLORATION on the locobot")
    parser.add_argument(
        "--model",
        "-m",
        default="nomad",
        type=str,
        help="model name (only nomad is supported) (hint: check ../config/models.yaml) (default: nomad)",
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
        "--dir",
        "-d",
        default="topomap",
        type=str,
        help="path to topomap images",
    )
    parser.add_argument(
        "--goal-node",
        "-g",
        default=-1,
        type=int,
        help="""goal node index in the topomap (if -1, then the goal node is 
        the last node in the topomap) (default: -1)""",
    )
    parser.add_argument(
        "--close-threshold",
        "-t",
        default=3,
        type=int,
        help="""temporal distance within the next node in the topomap before 
        localizing to it (default: 3)""",
    )
    parser.add_argument(
        "--radius",
        "-r",
        default=4,
        type=int,
        help="""temporal number of locobal nodes to look at in the topopmap for
        localization (default: 2)""",
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

