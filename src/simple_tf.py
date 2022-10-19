#!/usr/bin/python3.8
import rospy
import tf2_geometry_msgs
import tf2_ros
import tf2_sensor_msgs
import ros_numpy
import numpy as np

class simpele_TF(object):
    
    def __init__(self,source_frame,target_frame):
        self.source_frame = source_frame
        self.target_frame = target_frame
        self.creat_listener()
        self.time = rospy.Time(0)
        self.wating_time = rospy.Duration(1)
        return



    def creat_listener(self):
        self.tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
        tf2_ros.TransformListener(self.tf_buffer)
        return

    def get_transformation(self):
    # get the tf at first available time
        try:
            self.transformation = self.tf_buffer.lookup_transform(self.target_frame,
                    self.source_frame, self.time, self.wating_time)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            rospy.logerr('Unable to find the transformation from %s to %s'
                            % self.source_frame, self.target_frame)
        return 

    def get_inverse_transformation(self):
        self.inverse_transformation = self.transformation
        self.inverse_transformation.header.frame_id = self.source_frame
        tf = ros_numpy.geometry.transform_to_numpy(self.inverse_transformation.transform)
        tf = np.matrix(tf).I
        self.inverse_transformation.transform = ros_numpy.geometry.numpy_to_transform(tf)



    def transform_pose(self,pose):
        self.get_transformation()   
        self.tfpose =  tf2_geometry_msgs.do_transform_pose(pose,self.transformation)
        return self.tfpose

    def transform_pcd(self,pcd):
        self.get_transformation()   
        self.pcd =  tf2_sensor_msgs.do_transform_cloud(pcd,self.transformation)
        return self.pcd

    def inverse_transform_pcd(self,pcd):
        self.get_inverse_transformation()   
        self.pcd =  tf2_sensor_msgs.do_transform_cloud(pcd,self.inverse_transformation)
        return self.pcd
