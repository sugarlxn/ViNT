#!/usr/bin/env python

import numpy as np
import torch
from std_msgs.msg import Bool, Float32MultiArray
import rclpy
from rclpy.clock import Clock
from PIL import Image


a = np.array([1, 2, 3])
b = np.array([4, 5, 6])
c = np.concatenate((a, b))
print(c)
print(type(c))
print(c.shape)

msg = Float32MultiArray(data=c)

print(msg.data)
print(Clock().now().nanoseconds)

image = Image.open('/root/vint_ws/ros_ws/topomaps/images/topomap_loop7/12.png')
image.show()
# b,g,r = image.split()
# rgb_image = Image.merge('RGB',(r,g,b))
# rgb_image.show()
# rgb_image.save("test_rgb.png")

