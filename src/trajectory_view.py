#!/usr/bin/env python3
import rospy
import numpy as np
from std_msgs.msg import String,Float32
import message_filters
from geometry_msgs.msg import PoseStamped,Pose,Vector3
from nav_msgs.msg import Path
import tf2_ros
import math
from idPD import *
from simple_tf import *
from visualization_msgs.msg import Marker


path = []
planner=0
tello_waypoints = 0
tello_path = []

def callback(data):
    global planner
    target = data.points[0]
    planner = data.points
    target_point =[target.x,target.y]
    path.append(target_point)

def path_callback(data):
    global tello_waypoints
    tello_waypoints = data

def poses2np():
    for i in tello_waypoints.poses:
        point = i.pose.position
        target = [point.x,point.y]
        tello_path.append(target)

def add_final_path():
    for i in planner:
        target = i
        target_point =[target.x,target.y]
        path.append(target_point)



rospy.init_node('trajectory_view',anonymous=True)
sub = rospy.Subscriber("/ego_planner_node/optimal_list",Marker,callback)
tello_point_sub = rospy.Subscriber("/orb_slam3_ros/trajectory",Path,path_callback)
rospy.spin()
poses2np()
add_final_path()
path_array = np.array(path)
tello_array = np.array(tello_path)
np.save("./data/path.npy",path_array)
np.save("./data/tello_path.npy",tello_array)