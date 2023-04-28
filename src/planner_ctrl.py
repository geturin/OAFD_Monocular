#!/usr/bin/python3.8
import rospy
import numpy as np
from std_msgs.msg import String,Float32
import message_filters
from geometry_msgs.msg import PoseStamped,Pose,Vector3
import tf2_ros
import math
from idPD import *
from simple_tf import *
from visualization_msgs.msg import Marker




#初始化ros
rospy.init_node('tello_contrl',anonymous=True)

ctrl =rospy.Publisher('/message',String,queue_size=1)

#PD结果publish
pdresult = rospy.Publisher('/PDctrl_result',Vector3,queue_size=1)
dresult = rospy.Publisher('/de_result',Vector3,queue_size=1)

pd_vector = Vector3()
d_vector = Vector3()


#初始化tf2
tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
tf2_ros.TransformListener(tf_buffer)


rate = rospy.Rate(20)

alpha = 0
# world frame origin
origin = PoseStamped()
origin.pose.position.x = 0
origin.pose.position.y = 0
origin.pose.position.z = 0

#path
star = 0
pointnumber = 0
map_pose = PoseStamped()
goal_pose = PoseStamped()

def callback(data):
    global map_pose,wait_point,pointnumber,star,waypoint_limt


    if pointnumber!=len(data.points):
        pointnumber = len(data.points)
        if pointnumber <= 11:
            # waypoint_limt <= 9
            star = pointnumber - 1
        else:
            # waypoint_limt = 2
            star = 0

        # star = star-3
        # star = np.clip(star,0,waypoint_limt)


    map_pose.pose.position=data.points[star]
    goal_pose.pose.position = data.points[pointnumber-1]
    wait_point=1




def getPoint():
    tf_linster.time = rospy.Time.now()
    tf_linster.get_transformation()
    tfpose = tf_linster.transform_pose(map_pose)
    goal = tf_linster.transform_pose(goal_pose)
    point = tfpose.pose.position
    qutar = tfpose.pose.orientation

    x = point.x
    y = point.y
    z = point.z
    # yaw =  math.atan2(2*(qutar.w*qutar.z+qutar.x*qutar.y),1-2*(qutar.z*qutar.z+qutar.y*qutar.y))
    # yaw = yaw*180/math.pi
    yaw = goal.pose.position.y
    return x,y,z,yaw,tfpose.pose

def calibration(data):
    global alpha
    alpha = abs(data.data/tello_origin.pose.position.z) 




tf_linster = simpele_TF(source_frame="world",target_frame="tello")
#calibration slam world frame and real world frame 
rospy.sleep(3.)

# tf_linster.get_transformation()
tello_origin = tf_linster.transform_pose(origin)

while alpha == 0:
    tof = rospy.Subscriber("/tof",Float32,calibration)



s = 0.35
#idPDcontrll set
x=PD(P=0.92, D=1.74, scal=s*alpha)
y=PD(P=0.92, D=1.74, scal=s*alpha)
z=PD(P=0.92, D=1.74, scal=s*alpha)

yaw=PD(P=0.3, D=1.2, scal=s*alpha)


wait_point=0
waypoint_limt =2 

#set subscriber
bspline = message_filters.Subscriber("/ego_planner_node/optimal_list", Marker)
bspline.registerCallback(callback)

while not rospy.is_shutdown():

        if wait_point ==1:
            error_x,error_y,error_z,error_yaw,error_pose=getPoint()

            sp_x =x.ctrl(error_x)
            sp_y = y.ctrl(error_y)
            sp_z = z.ctrl(error_z)

            sp_yaw = yaw.ctrl(error_yaw)

            #set speed max and min
            sp_x = round(np.clip(sp_x,-30,30))
            sp_y = round(np.clip(sp_y, -30, 30))
            sp_z = round(np.clip(sp_z, -30, 30))

            sp_yaw = round(np.clip(sp_yaw, -15, 15))

            #pub pd控制器结算结果
            pd_vector.x , pd_vector.y , pd_vector.z = sp_x , sp_y , sp_z
            d_vector.x , d_vector.y , d_vector.z = x.de , y.de , z.de

            pdresult.publish(pd_vector)
            dresult.publish(d_vector)
            
            #pub tello控制指令
            msg = 'rc {} {} {} {}'.format(
                    -1*sp_y,
                    sp_x,
                    sp_z,
                    -1*sp_yaw
            )

            ctrl.publish(msg)
        else   :
            pass


        rate.sleep()
