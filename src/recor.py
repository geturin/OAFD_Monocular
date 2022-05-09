#!/usr/bin/python3.8
import rospy
import message_filters
from nav_msgs.msg import Path
import math

rospy.init_node('wait_path',anonymous=True)

flypath=[]
rate=rospy.Rate(20)
count =0
def callback(data):
    global flypath,count
    flypath = data.poses
    count=1




while not rospy.is_shutdown():
    rviz = message_filters.Subscriber("/orb_slam3_ros/trajectory", Path)
    rviz.registerCallback(callback)

    if count ==1:
        #print(flypath[1].pose.position.x)
        point=flypath[100].pose.position
        distance=math.sqrt((point.x**2+point.y**2+point.z**2))
        print(distance)
    rate.sleep()