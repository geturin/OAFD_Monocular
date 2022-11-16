#!/usr/bin/python3.8
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


DATE_PATH='/mnt/d/data/NYUV2/basement_0001a/rgb'
rgb_list = os.listdir(DATE_PATH)

if __name__ =='__main__':
    frame=0
    # 创建发布者node 定义发布消息数据类型
    rospy.init_node('NYU_node',anonymous=True)
    cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=10)

    rate = rospy.Rate(20)
    bridge=CvBridge()
    while frame <= 1400:
        pubimg=Image()
        header=Header()
        header.stamp=rospy.Time.now()
        header.frame_id="world"
        # 发布摄像机图片
        img=cv2.imread(os.path.join(DATE_PATH,rgb_list[frame]))
        pubimg =  bridge.cv2_to_imgmsg(img,'bgr8')
        pubimg.header=header
        cam_pub.publish(pubimg)

        rate.sleep()
        #
        frame=frame+1
