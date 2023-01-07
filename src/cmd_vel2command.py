#!/usr/bin/python3.8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np


rospy.init_node("cmd_vel_to_command")
ctrl =rospy.Publisher('/message',String,queue_size=1)
rate = rospy.Rate(20)
msg = "rc 0 0 0 0"
def callback(data):
    global ctrl,msg

    data = Twist()
    x = round(data.linear.x)
    y = round(-data.linear.y)
    yaw = round(-data.angular.z/np.pi*180.0)

    msg = 'rc {} {} {} {}'.format(
        y,
        x,
        0,
        yaw
        )

    
sub = rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=1)

while not rospy.is_shutdown():
    ctrl.publish(msg)
    rate.sleep()