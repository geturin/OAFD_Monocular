#!/usr/bin/python3.8
import rospy
import pcl
from sensor_msgs.msg import PointCloud2,Image
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header
import ros_numpy
import numpy as np
import cv2
import pcl.pcl_visualization





class pointcloud_pub(object):

    def __init__(self) -> None:
        rospy.init_node('pcd_pub', anonymous=True)
        return
    
    def read_pcd_path(self,path) -> str:
        self.path = path
        self.pcd = np.load(self.path)

    def pub(self,frame_id) -> str:
        self.header = Header()
        self.header.frame_id = frame_id
        self.pcldata = pcd2.create_cloud_xyz32(self.header, self.pcd)
        rospy.Publisher('/pointcloud_pub', PointCloud2, queue_size=1).publish(self.pcldata)




publisher = pointcloud_pub()

frame =0
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    publisher.read_pcd_path("/mnt/d/code/DPT/%010d.npy"%frame)
    publisher.pub("map")

    
    img = cv2.imread("/home/kero/image/%010d.png"%frame)
    cv2.imshow("1",img)
    cv2.waitKey(10)
    frame=frame+1
    frame%=1000 
    rate.sleep() 


        

