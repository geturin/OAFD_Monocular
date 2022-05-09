#!/usr/bin/python3.8
import rospy
import pcl
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pcd2
import ros_numpy
import numpy as np
import pcl.pcl_visualization




def callback(data):
    header=data.header
    pc = ros_numpy.numpify(data)
    points=np.zeros((pc.shape[0],3))
    points[:,0]=pc['x']
    points[:,1]=pc['y']
    points[:,2]=pc['z']

    p = pcl.PointCloud(np.array(points, dtype=np.float32))
    k=p.to_array()


    fil = p.make_statistical_outlier_filter()
    fil.set_mean_k(50)
    fil.set_std_dev_mul_thresh(1.1)


    cloud_filtered = fil.filter()

    pcd=cloud_filtered.to_array()

    pcldata = pcd2.create_cloud_xyz32(header, pcd)

    rospy.Publisher('/filterpcl', PointCloud2, queue_size=1).publish(pcldata)





rospy.init_node('filter', anonymous=True)
rospy.Subscriber("/orb_slam3_ros/map_points", PointCloud2, callback)
rospy.spin()
