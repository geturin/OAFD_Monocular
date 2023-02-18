#!/usr/bin/python3.8
import rospy
import numpy as np
from std_msgs.msg import String,Float32
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped,Pose
import tf2_geometry_msgs
import tf2_ros
import math
from idPD import *
from simple_tf import *




#初始化ros
rospy.init_node('tello_contrl',anonymous=True)

ctrl =rospy.Publisher('/message',String,queue_size=1)
error_vector_pub=rospy.Publisher("/error_vector",Pose,queue_size=1)


#初始化tf2
tf_buffer = tf2_ros.Buffer(rospy.Duration(10))
tf2_ros.TransformListener(tf_buffer)


rate = rospy.Rate(20)

alpha = 0
wait_point = 0
# world frame origin
origin = PoseStamped()
origin.pose.position.x = 0
origin.pose.position.y = 0
origin.pose.position.z = 0

def callback(data):
    global map_pose,wait_point
    map_pose=data
    wait_point=1




def getPoint():
    tf_linster.get_transformation()
    tfpose = tf_linster.transform_pose(map_pose.pose)
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




tf_linster = simpele_TF(source_frame="map",target_frame="tello")
#calibration slam world frame and real world frame 
rospy.sleep(3.)

tf_linster.get_transformation()
tello_origin = tf_linster.transform_pose(origin)

while alpha == 0:
    tof = rospy.Subscriber("/tof",Float32,calibration)




#PDcontrll set
# x=PD(P=0.9, D=1.2, scal=0.15*alpha)
# y=PD(P=0.9, D=1.2, scal=0.15*alpha)
# z=PD(P=0.9, D=1.2, scal=0.15*alpha)
# yaw=PD(P=0.08, D=0.02, scal=0.05*alpha)

#idPDcontrll set
x=idPD(P=0.9, D=2.3, scal=0.15*alpha, alpha=0.1)
y=idPD(P=0.9, D=2.3, scal=0.15*alpha, alpha=0.1)
z=idPD(P=0.9, D=2.3, scal=0.15*alpha, alpha=0.1)
yaw=idPD(P=0.08, D=2.3, scal=0.05*alpha, alpha=0.1)






#set subscriber
rviz = message_filters.Subscriber("/initialpose", PoseWithCovarianceStamped)
rviz.registerCallback(callback)
while not rospy.is_shutdown():

        if wait_point ==1:
            error_x,error_y,error_z,error_yaw,error_pose=getPoint()

            #误差速度话题

            error_vector_pub.publish(error_pose)


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
                    0,
                    -1*sp_yaw
            )
            ctrl.publish(msg)
        else   :
            msg = 'rc {} {} {} {}'.format(
                0,
                0,
                0,
                0
            )
            ctrl.publish(msg)


        rate.sleep()




