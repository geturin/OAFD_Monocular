#!/usr/bin/python3.8
import rospy
from sensor_msgs.msg import PointCloud2,Image,PointField
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

    def pub(self,frame_id,topic_name) -> str:
        self.header = Header()
        self.header.frame_id = frame_id
        self.pcldata = pcd2.create_cloud_xyz32(self.header, self.pcd)
        rospy.Publisher(topic_name, PointCloud2, queue_size=1).publish(self.pcldata)
    


publisher = pointcloud_pub()
orb_pub = pointcloud_pub()

frame =0
rate = rospy.Rate(20)
mesh_scal=10
depth_transform = depth_to_pcd(5)
depth_calibra = Clibration()

pcd_publish = rospy.Publisher("test_pcd",PointCloud2,queue_size=1)

while not rospy.is_shutdown():
    depth=np.load("/home/kero/catkin_ws/src/kitti/data/ai_depth.npy")
    orb_pcd=np.load("/home/kero/catkin_ws/src/kitti/data/orb_pcd.npy")

    #最小二乘法计算orb点云和ai深度的比例 并缩放ai深度图
    scale = depth_calibra.depth_calibration(depth.copy(),orb_pcd)
    depth = scale * depth

    bgr = cv2.imread("./data/raw_image.png") 
    rgb = cv2.cvtColor(bgr,cv2.COLOR_BGR2RGB)


    rgbpcd = depth_transform.get_rgbpcd(depth,rgb)

    # RGB8
    fields = [  PointField("x",0,PointField.FLOAT32,1),
                PointField("y",4,PointField.FLOAT32,1),
                PointField("z",8,PointField.FLOAT32,1),
                PointField("rgb",12,PointField.FLOAT32,1)]
    



    header = Header()
    header.frame_id = "map"

    msg = pcd2.create_cloud(header=header,fields=fields,points=rgbpcd )
    pcd_publish.publish(msg)
    rate.sleep()