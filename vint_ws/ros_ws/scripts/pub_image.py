#!/usr/bin/env python
'''
for ros1 noetic
before run this script, you should run the following command:
1) source /opt/ros/noetic/setup.bash
2) roscore
'''

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np
import time
import sys


def waypointcallback(data):
    #可视化waypoint
    print(data.data)



if __name__ == '__main__':
    rospy.init_node('pub_image', anonymous=True)
    pub = rospy.Publisher('/usb_cam/image_raw', Image, queue_size=10)
    #sub = rospy.Subscriber('/waypoint', Float32MultiArray, waypointcallback)

    rate = rospy.Rate(1) # 10hz
    bridge = CvBridge()
    #读取1.jpg图片
    img = cv2.imread('2.jpg', 1)
    img = cv2.resize(img, (160, 120))
    rospy.loginfo(img.shape)
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2R)
    image_msg = bridge.cv2_to_imgmsg(img, encoding="passthrough")
    #将图片发布到/image话题
    pub.publish(image_msg)
    rospy.loginfo("Image Published")


    while not rospy.is_shutdown():
        pub.publish(image_msg)
        rospy.loginfo("Image Published")
        print(rospy.get_time())
        print(type(rospy.get_time()))
        rate.sleep()