#!/usr/bin/env python
'''
for ros2 foxy
before run this script, you should run the following command:
1) source /opt/ros/foxy/setup.bash
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import time
import sys

class PubImage(Node):
    def __init__(self, name):
        super().__init__(name)
        self.pub = self.create_publisher(Image, '/Hololens/Image', 10)
       
        self.bridge = CvBridge()
        self.timer = self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        img = cv2.imread('/root/vint_ws/ros_ws/topomaps/images/topomap3/39.png', 1)
        # img = cv2.resize(img, (160, 120))
        self.get_logger().info(str(img.shape))
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image_msg = self.bridge.cv2_to_imgmsg(img, encoding="passthrough")
        self.pub.publish(image_msg)
        self.get_logger().info("Image Published")

def main(args=None):
    rclpy.init(args=args)
    pub_image = PubImage('pub_image')
    rclpy.spin(pub_image)
    pub_image.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()