#!/usr/bin/python3.8
import rospy
import numpy as np
from std_msgs.msg import String
import message_filters
from geometry_msgs.msg import PoseWithCovarianceStamped,PoseStamped
import tf2_geometry_msgs
import tf2_ros
import math




#初始化ros
rospy.init_node('tello_contrl',anonymous=True)

ctrl =rospy.Publisher('/message',String,queue_size=1)

#初始化tf2
tf_buffer = tf2_ros.Buffer(rospy.Duration(2))
tf2_ros.TransformListener(tf_buffer)


rate = rospy.Rate(20)


x=0
y=0
z=0
X=0
Y=0
Z=0
errox=0
erroy=0
erroz=0
wait_point=0
erroyaw=0



def callback(data):
        #global x, y, z
        #tfpose=transform_pose(map_tf_tello,data.pose)
        #point=tfpose.pose.position
        #x=point.x
        #y=point.y
        #z=point.z
        global map_pose,wait_point
        map_pose=data
        wait_point=1

def tellopoint(data):
        global X,Y,Z
        tfpose=transform_pose(transformation, data)
        point=tfpose.pose.position
        X = point.x
        Y = point.y
        Z = point.z



def get_transformation(source_frame, target_frame,
                       tf_cache_duration=2.0):


    # get the tf at first available time
    try:
        transformation = tf_buffer.lookup_transform(target_frame,
                source_frame, rospy.Time(0), rospy.Duration(2))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException):
        rospy.logerr('Unable to find the transformation from %s to %s'
                     % source_frame, target_frame)
    return transformation

def transform_pose(transformation, pose):
    tfpose = \
        tf2_geometry_msgs.do_transform_pose(pose,transformation)
    return tfpose



def getPoint():
    map_tf_tello = get_transformation("map", "tello")
    tfpose = transform_pose(map_tf_tello, map_pose.pose)
    point = tfpose.pose.position
    qutar = tfpose.pose.orientation

    x = point.x
    y = point.y
    z = point.z
    yaw =  math.atan2(2*(qutar.w*qutar.z+qutar.x*qutar.y),1-2*(qutar.z*qutar.z+qutar.y*qutar.y))
    yaw = yaw*180/math.pi
    return x,y,z,yaw

transformation=get_transformation("world","map")


while not rospy.is_shutdown():

        rviz = message_filters.Subscriber("/initialpose", PoseWithCovarianceStamped)
        rviz.registerCallback(callback)
        #tello=message_filters.Subscriber("/orb_slam3_ros/camera", PoseStamped)
        #tello.registerCallback(tellopoint)

        if wait_point ==1:
            x,y,z,yaw=getPoint()


            b=50*x + 200*(x-errox)
            errox=x

            a=50*y + 200*(y-erroy)
            erroy=y

            c = 20 * z + 200 * (z - erroz)
            erroz = z

            d=0.8*yaw + 15*(yaw-erroyaw)
            erroyaw=yaw

            #set speed max and min
            a = round(np.clip(a,-35,35))
            b = round(np.clip(b, -35, 35))
            c = round(np.clip(c, -35, 35))
            d = round(np.clip(d, -20, 20))


            msg = 'rc {} {} {} {}'.format(
                    -1*a,
                    b,
                    0,
                    -1*d
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




