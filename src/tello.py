#!/usr/bin/python3.8
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

if __name__ =='__main__':
    frame=0
    # 创建发布者node 定义发布消息数据类型
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=10)

    rate = rospy.Rate(5)
    bridge=CvBridge()
    while not rospy.is_shutdown():
        # 发布摄像机图片

        img = cv2.imread(os.path.join('/mnt/d/code/monodepth/monodepth2/TE/images/%010d.png'%frame))
        pubimg =  bridge.cv2_to_imgmsg(img,'bgr8')
        pubimg.header.stamp=rospy.Time.now()
        cam_pub.publish(pubimg)

        rospy.loginfo(' publisher')
        rate.sleep()
        #
        frame = frame + 1
        frame %= 787



