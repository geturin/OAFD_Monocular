#!/usr/bin/python3.8
import rospy
from sensor_msgs.msg import PointCloud2,Image,PointField
import sensor_msgs.point_cloud2 as pcd2
from std_msgs.msg import Header
import numpy as np
from depth2pcd import *
import message_filters
from simple_tf import *
from cv_bridge import CvBridge
import ros_numpy




rospy.init_node('pcd_pub', anonymous=True)

depth_transform = depth_to_pcd(5)
depth_calibra = Clibration()

start = 0
bridge = CvBridge()



# RGB8
fields = [  PointField("x",0,PointField.FLOAT32,1),
            PointField("y",4,PointField.FLOAT32,1),
            PointField("z",8,PointField.FLOAT32,1),
            PointField("rgb",12,PointField.FLOAT32,1)]
pcd_publish = rospy.Publisher("test_pcd",PointCloud2,queue_size=1)


def callback(image,depth,pcd):
    global depth_transform , depth_calibra , start , bridge , fields , pcd_publish,tflistener

    if start==0:
        try:
            tflistener = simpele_TF("world","camera")
            start = 1
        except:
            print("cant get tranform from world to camera  ,wait transform")
            return
    else:
        #get depth map and rgb image
        rgb = bridge.imgmsg_to_cv2(image,"rgb8")
        #test scale
        depth = bridge.imgmsg_to_cv2(depth,"32FC1")     
        depth = 1/depth

        # depth =  depth[:,:,0]

        #orb_pcd world->camera
        tflistener.time = image.header.stamp
        try:
          pcd=tflistener.transform_pcd(pcd)
        except:
            print("tf lost")
            return

        #最小二乘法计算orb点云和ai深度的比例 并缩放ai深度图
        pcd = ros_numpy.numpify(pcd)#pointcloud2 msg -> numpy arry
        scale = depth_calibra.depth_calibration(depth,pcd)
        depth = 6*scale * depth

        # #with out calibration test
        # depth[depth>0.55]=0
        # depth = 20*depth


        # to pointlcoud
        rgbpcd = depth_transform.get_rgbpcd(depth,rgb)
        header = Header()
        header.frame_id = "camera"
        msg = pcd2.create_cloud(header=header,fields=fields,points=rgbpcd)

        #取得world->camera的逆矩阵
        msg = tflistener.inverse_transform_pcd(msg)
        pcd_publish.publish(msg)
        return



camera_sub = message_filters.Subscriber('/camera/image_raw', Image)
depth_sub = message_filters.Subscriber('/camera/depth', Image)
pcd_sub = message_filters.Subscriber('/orb_slam3_ros/map_points', PointCloud2)

ts = message_filters.TimeSynchronizer([camera_sub, depth_sub,pcd_sub], 3)
ts.registerCallback(callback)
rospy.spin()