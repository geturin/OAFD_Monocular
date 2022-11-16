#!/usr/bin/python3.8
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np


DATE_PATH='/home/kero/data/kitti/2011_09_26/2011_09_26_drive_0005_sync/'

#DATE_PATH='/home/kero/data/kitti/'

if __name__ =='__main__':
    frame=1
    # 创建发布者node 定义发布消息数据类型
    rospy.init_node('kitti_node',anonymous=True)
    cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=10)
    #cam_pub = rospy.Publisher('kitti_cam', Image, queue_size=10)
    pcl_pub =rospy.Publisher('kitti_pcl',PointCloud2,queue_size=10)

    rate = rospy.Rate(10)
    bridge=CvBridge()
    while frame <= 153:
        pubimg=Image()
        header=Header()
        header.stamp=rospy.Time.now()
        header.frame_id="world"
        # 发布摄像机图片
        img=cv2.imread(os.path.join(DATE_PATH,'image_02/data/%010d.png'%frame))
        pubimg =  bridge.cv2_to_imgmsg(img,'bgr8')
        pubimg.header=header
        cam_pub.publish(pubimg)

        # 发布点云
        #pointcloud = np.fromfile(os.path.join(DATE_PATH,'velodyne_points/data/%010d.bin'%frame),dtype=np.float32).reshape(-1,4)

        #header=Header()
        #header.stamp=rospy.Time.now()
        #header.frame_id='map'
        # 去除点云数据第四位反射率信息
        #pcldata=pcl2.create_cloud_xyz32(header,pointcloud[:,:3])
        #pcl_pub.publish(pcldata)




        rate.sleep()
        #
        frame=frame+1

