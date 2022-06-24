#!/usr/bin/python3.8
import rospy
from sensor_msgs.msg import PointCloud2,Image
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header
import numpy as np
import cv2
from depth2pcd import *
import pcl





class pointcloud_pub(object):

    def __init__(self) -> None:
        rospy.init_node('pcd_pub', anonymous=True)
        return
    
    def read_pcd(self,pcd) -> str:
        self.pcd = pcd

    def pub(self,frame_id) -> str:
        self.header = Header()
        self.header.frame_id = frame_id
        self.pcldata = pcd2.create_cloud_xyz32(self.header, self.pcd)
        rospy.Publisher('/pointcloud_pub', PointCloud2, queue_size=1).publish(self.pcldata)




def pcd_filter(pcd):
    p = pcl.PointCloud(np.array(pcd, dtype=np.float32))
    k=p.to_array()


    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.1)


    cloud_filtered = fil.filter()

    pcd=cloud_filtered.to_array()

    return pcd


publisher = pointcloud_pub()

frame =0
rate = rospy.Rate(20)

depth_transform = depth_to_pcd(10)

while not rospy.is_shutdown():
    pcd = depth_transform.get_pcd(depth=10*np.load("/home/kero/catkin_ws/src/kitti/data/ai_depth.npy"))
    pcd = pcd_filter(pcd)
    #栅格化（test）
    pcd =(100*pcd).astype(int)
    pcd = (pcd.astype(float)/100)
    pcd = np.unique(pcd,axis=0)

    publisher.read_pcd(pcd)
    publisher.pub(frame_id="map")

    
    rate.sleep() 


        

