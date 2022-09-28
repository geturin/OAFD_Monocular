#!/usr/bin/python3.8
from matplotlib import image
from pyrsistent import field
import rospy
from sensor_msgs.msg import PointCloud2,Image,PointField
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header
import numpy as np
import cv2
from depth2pcd import *
import pcl
import message_filters
from simple_tf import *
from cv_bridge import CvBridge



rospy.init_node('pcd_pub', anonymous=True)

frame =0
rate = rospy.Rate(20)
mesh_scal=10
depth_transform = depth_to_pcd(8)
depth_calibra = Clibration()

start = 1
bridge = CvBridge()



# RGB8
fields = [  PointField("x",0,PointField.FLOAT32,1),
            PointField("y",4,PointField.FLOAT32,1),
            PointField("z",8,PointField.FLOAT32,1),
            PointField("rgb",12,PointField.FLOAT32,1)]
pcd_publish = rospy.Publisher("test_pcd",PointCloud2,queue_size=1)


def callback(image,depth):
    global depth_transform , depth_calibra , start , bridge , fields , pcd_publish

    if start==0:
        try:
            tflistener = simpele_TF("world","camera")
            start = 1
        except:
            print("cant get tranform from world to camera  ,wait transform")
    else:
        rgb = bridge.imgmsg_to_cv2(image,"rgb8")
        #test scale
        depth = bridge.imgmsg_to_cv2(depth,"32FC1")
        depth = 40/depth
        # depth =  depth[:,:,0]


        rgbpcd = depth_transform.get_rgbpcd(depth,rgb)

        header = Header()
        header.frame_id = "map"

        msg = pcd2.create_cloud(header=header,fields=fields,points=rgbpcd )
        pcd_publish.publish(msg)



camera_sub = message_filters.Subscriber('/camera/image_raw', Image)
depth_sub = message_filters.Subscriber('/camera/depth', Image)

ts = message_filters.TimeSynchronizer([camera_sub, depth_sub], 1)
ts.registerCallback(callback)
rospy.spin()