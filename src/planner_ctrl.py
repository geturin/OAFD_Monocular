#!/usr/bin/python3.8
import rospy
import numpy as np
from std_msgs.msg import String,Float32
import message_filters
from geometry_msgs.msg import PoseStamped,Pose
import tf2_ros
import math
from idPD import *
from simple_tf import *
from visualization_msgs.msg import Marker




#初始化ros
rospy.init_node('tello_contrl',anonymous=True)

ctrl =rospy.Publisher('/message',String,queue_size=1)


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

def callback(data):
    global map_pose,wait_point,pointnumber
    if pointnumber == 0:
        pointnumber = len(data.points)
    else:
        pass

    map_pose.pose.position=data.points[0]
    wait_point=1




def getPoint():
    tf_linster.get_transformation()
    tfpose = tf_linster.transform_pose(map_pose)
    point = tfpose.pose.position
    qutar = tfpose.pose.orientation

    x = point.x
    y = point.y
    z = point.z
    yaw =  math.atan2(2*(qutar.w*qutar.z+qutar.x*qutar.y),1-2*(qutar.z*qutar.z+qutar.y*qutar.y))
    yaw = yaw*180/math.pi
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




#idPDcontrll set
x=idPD(P=0.83, D=1.65, scal=0.1*alpha, alpha=0.12)
y=idPD(P=0.83, D=1.65, scal=0.1*alpha, alpha=0.12)
z=idPD(P=0.81, D=1.78, scal=0.1*alpha, alpha=0.12)
yaw=idPD(P=0.08, D=0.6, scal=0.05*alpha, alpha=0.1)


wait_point=0

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
            sp_yaw = round(np.clip(sp_yaw, -20, 20))


            msg = 'rc {} {} {} {}'.format(
                    -1*sp_y,
                    sp_x,
                    sp_z,
                    -1*sp_yaw
            )

            if abs(sp_y)+abs(sp_x)+abs(sp_z) <= 20:
                star = star +1
            else:
                pass
            # if star >= pointnumber-5:
            #     msg = "land"  
            #     rate = rospy.Rate(1)
            if star >= pointnumber-7:
                star = 0
            else:
                pass

            ctrl.publish(msg)

        else   :
            pass


        rate.sleep()