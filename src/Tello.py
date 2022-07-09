#!/usr/bin/python3.8
from djitellopy import tello
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String,Float32
from geometry_msgs.msg import Twist
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters


#
me=tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

frame=0
# 创建发布者node 定义发布消息数据类型
rospy.init_node('tello_node',anonymous=True)
cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=1)

rate = rospy.Rate(20)
bridge=CvBridge()

#count==0 take off
count=1

def callback(data):
    me.send_command_without_return(data.data)



while not rospy.is_shutdown():
    #pub img
    img = me.get_frame_read().frame
    pubimg = bridge.cv2_to_imgmsg(img, 'bgr8')
    pubimg.header.stamp = rospy.Time.now()
    cam_pub.publish(pubimg)

    #pub tof
    gettof=me.get_distance_tof()
    tof_pub=rospy.Publisher("/tof",Float32,queue_size=1)
    tof_pub.publish(gettof)

    sub=rospy.Subscriber('/message',String,callback,queue_size=1)

    if count==0:
        me.send_command_without_return("takeoff")
        count=1

    #rospy.loginfo(' publisher')
    rate.sleep()

