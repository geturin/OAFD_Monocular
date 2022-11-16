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


#kitti version
class Clibration(object):

    def __init__(self) -> None:
        self.rotation=np.array([0,0,0],dtype=np.float32).reshape(3,1)
        self.translation=np.array([0,0,0],dtype=np.float32).reshape(1,3)
        self.distortion = np.array([[0,0,0,0]],dtype="float")
        # kitti
        # self.camera = np.array((721.5377,0,609.5593,
        #     0,721.5377,172.854,
        #     0,0,1),dtype="float").reshape(3,3)

        #NYUV2
        self.camera = np.array((5.1885790117450188e+02,0,3.2558244941119034e+02,
            0,5.1946961112127485e+02,2.5373616633400465e+02,
            0,0,1),dtype="float").reshape(3,3)
        
        pass
    
    def orb_pcd_reprojet(self,orb_pcd):
        camera_pcd=np.zeros((orb_pcd.shape[0],3))
        camera_pcd[:,0]=-1*orb_pcd["y"]
        camera_pcd[:,1]=-1*orb_pcd["z"]
        camera_pcd[:,2]=orb_pcd["x"]

        reTransform=cv2.projectPoints(camera_pcd,self.rotation,self.translation,self.camera,self.distortion)
        reTransform = reTransform[0][:,0].astype(int)

        pixel = reTransform
        # filter = np.where((pixel[:,0]<1242)&(pixel[:,1]<375)&(pixel[:,0]>=0)&(pixel[:,1]>=0)) #kitti
        filter = np.where((pixel[:,0]<640)&(pixel[:,1]<480)&(pixel[:,0]>=0)&(pixel[:,1]>=0)) #NYU
 
        pixel = pixel[filter]
        depth = camera_pcd[:,2].reshape(-1,1)[filter]

        # depth_image=np.zeros((375, 1242)) #kitti
        depth_image=np.zeros((480, 640)) #NYU

        depth_image[pixel[:,1],pixel[:,0]] = depth[:,0]

        return depth_image


start = 0
bridge = CvBridge()
project = Clibration()


def callback(image,pcd):
    global  start , bridge,tflistener ,project

    # DATE_PATH='/mnt/d/data/kitti/orb_depth/'
    DATE_PATH='/mnt/d/data/NYUV2'

    if start==0:
        try:
            tflistener = simpele_TF("world","camera")
            start = 1
        except:
            print("cant get tranform from world to camera  ,wait transform")
            return
    else:
        #get image frame number
        frame = image.header.seq

        # depth =  depth[:,:,0]

        #orb_pcd world->camera
        tflistener.time = image.header.stamp
        try:
          pcd=tflistener.transform_pcd(pcd)
        except:
            print("tf lost")
            return

        pcd = ros_numpy.numpify(pcd)
        depth = project.orb_pcd_reprojet(pcd)
        np.save(os.path.join(DATE_PATH,'%010d.npy'%frame),depth)
        print(frame)

        return
if __name__ == "__main__":
    rospy.init_node('pcd_pub', anonymous=True)
    camera_sub = message_filters.Subscriber('/camera/image_raw', Image)
    pcd_sub = message_filters.Subscriber('/orb_slam3_ros/map_points', PointCloud2)

    ts = message_filters.TimeSynchronizer([camera_sub, pcd_sub], 3)
    ts.registerCallback(callback)
    rospy.spin()
