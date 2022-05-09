#!/usr/bin/python3.8
import message_filters
from sensor_msgs.msg import Image, CameraInfo,PointCloud2
import rospy
import ros_numpy
import numpy as np
from cv_bridge import CvBridge
import cv2
import os
import tf2_geometry_msgs
import tf2_ros
from simple_tf import *
from projectPointcloud import*



frame=0
DATE_PATH='/mnt/d/data/pointcloud'


def callback(image, pcd):
    global frame,DATE_PATH
    #transform pcd form world frame to camera frame
    pcd=tflistener.transform_pcd(pcd)
    #transform msg type
    imgdata=CvBridge().imgmsg_to_cv2(image,"bgr8")
    pcddata=ros_numpy.numpify(pcd)    
    #image.data=CvBridge().cv2_to_imgmsg(imgdata,"bgr8")
    #cam_pub.publish(image)

    cv2.imwrite(os.path.join(DATE_PATH,'%010d.png'%frame),imgdata,[int(cv2.IMWRITE_JPEG_QUALITY), 100])
    np.save(os.path.join(DATE_PATH,'%010d.npy'%frame),pcddata)

    if frame == 1000:
        rospy.signal_shutdown("mission over")

    frame +=1

    



rospy.init_node('pcd_calibration',anonymous=True)
pub_img = Image()
cam_pub =rospy.Publisher('project',Image,queue_size=10)
tflistener = simpele_TF("world","camera")

camera_sub = message_filters.Subscriber('/camera/image_raw', Image)
pcd_sub = message_filters.Subscriber('/orb_slam3_ros/map_points', PointCloud2)

ts = message_filters.TimeSynchronizer([camera_sub, pcd_sub], 1)
ts.registerCallback(callback)
rospy.spin()