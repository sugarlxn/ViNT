#!/usr/bin/env python
'''
for ros2 foxy
before run this script, you should run the following command:
1) source /opt/ros/foxy/setup.bash
'''
import numpy as np
import yaml
from typing import Tuple
import threading

# ROS
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Float32MultiArray
from utils import clip_angle


# ROSData
class ROSData:
    def __init__(self,timeout: int = 3, queue_size: int = 1, name: str = ""):
        self.timout = timeout
        self.last_time_received = float("-inf")
        self.queue_size = queue_size
        self.data = None
        self.name = name
        self.phantom = False

    def get(self):
        return self.data

    def set(self,data):
        time_waited = Clock().now().nanoseconds/(10**9) - self.last_time_received
        if self.queue_size == 1:
            self.data = data
        else:
            if self.data is None or time_waited > self.timout: #reset queue if timeout
                self.data = []
            if len(self.data) == self.queue_size:
                self.data.pop(0)
            self.data.append(data)
        self.last_time_received = Clock().now().nanoseconds/(10**9)

    def is_valid(self, verbose: bool = False):
        time_waited = Clock().now().nanoseconds/(10**9) - self.last_time_received
        valid = time_waited < self.timout
        if self.queue_size > 1:
            valid = valid and len(self.data) == self.queue_size
        if verbose and not valid:
            print(f"Not receiving {self.name} data for {time_waited} seconds (timeout: {self.timout} seconds)")
        return valid

# CONSTANTS
WAYPOINT_TOPIC = "/waypoint"
REACHED_GOAL_TOPIC = "/topoplan/reached_goal"

# CONSTS
CONFIG_PATH = "../config/robot.yaml"
with open(CONFIG_PATH, "r") as f:
	robot_config = yaml.safe_load(f)
MAX_V = robot_config["max_v"]
MAX_W = robot_config["max_w"]
VEL_TOPIC = robot_config["vel_navi_topic"]
DT = 1/robot_config["frame_rate"]
# RATE = robot_config["frame_rate"]
RATE = 4
EPS = 1e-8
WAYPOINT_TIMEOUT = 1 # seconds # TODO: tune this
FLIP_ANG_VEL = np.pi/4

# Globals
vel_msg = Twist()
waypoint = ROSData(WAYPOINT_TIMEOUT, queue_size=1, name="waypoint")
reached_goal = False
reverse_mode = False
current_yaw = None

def clip_angle(theta) -> float:
	"""Clip angle to [-pi, pi]"""
	theta %= 2 * np.pi
	if -np.pi < theta < np.pi:
		return theta
	return theta - 2 * np.pi
      
def pd_controller(waypoint: np.ndarray) -> Tuple[float]:
	"""PD controller for the robot"""
	assert len(waypoint) == 2 or len(waypoint) == 4, "waypoint must be a 2D or 4D vector"
	if len(waypoint) == 2:
		dx, dy = waypoint
	else:
		dx, dy, hx, hy = waypoint
	# this controller only uses the predicted heading if dx and dy near zero
	if len(waypoint) == 4 and np.abs(dx) < EPS and np.abs(dy) < EPS:
		v = 0
		w = clip_angle(np.arctan2(hy, hx))/DT		
	elif np.abs(dx) < EPS:
		v =  0
		w = np.sign(dy) * np.pi/(2*DT)
	else:
		v = dx / DT
		w = np.arctan(dy/dx) / DT
	v = np.clip(v, 0, MAX_V)
	w = np.clip(w, -MAX_W, MAX_W)
	return v, w      


def callback_drive(waypoint_msg: Float32MultiArray):
	"""Callback function for the waypoint subscriber"""
	global vel_msg
	# print("seting waypoint")
	waypoint.set(waypoint_msg.data)
	
	
def callback_reached_goal(reached_goal_msg: Bool):
	"""Callback function for the reached goal subscriber"""
	global reached_goal
	reached_goal = reached_goal_msg.data

class Controler(Node):
    def __init__(self,name):
        super().__init__(name)
        self.waypoint_sub = self.create_subscription(Float32MultiArray, WAYPOINT_TOPIC, callback_drive, 1)
        self.reached_goal_sub = self.create_subscription(Bool, REACHED_GOAL_TOPIC, callback_reached_goal, 1)
        self.vel_pub = self.create_publisher(Twist, "/ble_controler/cmd_vel", 1)

def main():
    global vel_msg, reverse_mode
    rclpy.init()
    node = Controler("PD_CONTROLLER")
    rate = node.create_rate(RATE)
    logger = node.get_logger()
     
    # spin in a seqarate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    while rclpy.ok():
        vel_msg = Twist()
        if reached_goal:
            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0
            node.vel_pub.publish(vel_msg)
            print("Reached goal! stopping.....")
            break
        
        elif waypoint.is_valid(verbose=True):
            v, w = pd_controller(waypoint.get())
            if reverse_mode:
                v *= -1
            vel_msg.linear.x = v
            vel_msg.angular.z = w
            # print(f"Published velocity:{v},{w}")
            #小数点后两位 log info
            logger.info(f"Published velocity:{v:.2f},{w:.2f}")
            node.vel_pub.publish(vel_msg)
            rate.sleep()

    rclpy.shutdown()
    spin_thread.join()
    print("Node shutdown")

if __name__ == '__main__':
    main()
