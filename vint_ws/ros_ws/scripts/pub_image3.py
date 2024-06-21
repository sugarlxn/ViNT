#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import sys
from PIL import Image as PILImage
from utils import pil_to_msg


class PubImage(Node):
    def __init__(self,name):
        super().__init__(name)
        self.image_pub = self.create_publisher(Image,"/image_test",10)
        self.timer = self.create_timer(1, self.timer_callback)
    
    def timer_callback(self):
        image = PILImage.open('/root/vint_ws/ros_ws/topomaps/images/topomap3/38.png')
        image_msg = pil_to_msg(image)
        self.image_pub.publish(image_msg)
        print("pub image")




def main():
    rclpy.init()
    node = PubImage("pub_image")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__=="__main__":
    main()
