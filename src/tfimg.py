#!/usr/bin/python3.8
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
import sensor_msgs.point_cloud2 as pcl2
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

img_pub = rospy.Publisher("/camera/image_raw0", Image, queue_size=10)

def publish_image(imgdata,img_pub):
    img =imgdata
    img.header.stamp=rospy.Time.now()

    img_pub.publish(img)


def callback(data):
    #imgdata=CvBridge().imgmsg_to_cv2(data)
    img_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=1)
    
    publish_image(data, img_pub)
   # imgdata=CvBridge().imgmsg_to_cv2(imgdata,"rgb8")
    #cv2.imshow("1",imgdata)
    #cv2.waitKey(1)
def sub_image():
    rospy.init_node("img_tf", anonymous=True)
    rospy.Subscriber("/camera/notime",Image,callback)
    rospy.spin()

if __name__ == "__main__":
    sub_image()
