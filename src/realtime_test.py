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
# import pcl.pcl_visualization



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

#get map2world transform
map2world = simpele_TF("map","world")

def callback(image,depth,pcd):
    global depth_transform , depth_calibra , start , bridge , fields , pcd_publish,tflistener

    if start==0:
        try:
            tflistener = simpele_TF("map","camera")
            start = 1
        except:
            print("cant get tranform from map to camera  ,wait transform")
            return
    else:
        #get depth map and rgb image
        rgb = bridge.imgmsg_to_cv2(image,"rgb8")
        #test scale
        depth = bridge.imgmsg_to_cv2(depth,"32FC1")     
        depth = 1/depth

        # depth =  depth[:,:,0]

        #orb_pcd map->camera
        tflistener.time = image.header.stamp
        try:
          pcd=tflistener.transform_pcd(pcd)
        except:
            print("tf lost")
            return

        #最小二乘法计算orb点云和ai深度的比例 并缩放ai深度图
        pcd = ros_numpy.numpify(pcd)#pointcloud2 msg -> numpy arry
        scale = depth_calibra.depth_calibration(depth,pcd)
        depth = scale * depth

        # #with out calibration test
        # depth[depth>0.55]=0
        # depth = 20*depth


        # to rgb pointlcoud
        pcd = depth_transform.get_rgbpcd(depth,rgb)
        header = Header()
        header.frame_id = "camera"
        msg = pcd2.create_cloud(header=header,fields=fields,points=pcd)
       
        # to pointcloud
        # pcd = depth_transform.get_pcd(depth)

        # #pcl filter test
        # p = pcl.PointCloud(np.array(pcd, dtype=np.float32))
        # fil = p.make_statistical_outlier_filter()
        # fil.set_mean_k(40)
        # fil.set_std_dev_mul_thresh(1.5)
        # pcd = fil.filter()
        # pcd=pcd.to_array()

        # header = Header()
        # header.frame_id = "camera"
        # msg =pcd2.create_cloud_xyz32(header,pcd)



        #取得map->camera的逆矩阵
        msg = tflistener.inverse_transform_pcd(msg)
        msg = map2world.transform_pcd(msg)
        msg.header.frame_id = "world"
        pcd_publish.publish(msg)
        return



camera_sub = message_filters.Subscriber('/camera/image_raw', Image)
depth_sub = message_filters.Subscriber('/camera/depth', Image)
pcd_sub = message_filters.Subscriber('/orb_slam3_ros/map_points', PointCloud2)

ts = message_filters.TimeSynchronizer([camera_sub, depth_sub,pcd_sub], 3)
ts.registerCallback(callback)
rospy.spin()