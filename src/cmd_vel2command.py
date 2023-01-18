#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np


rospy.init_node("cmd_vel_to_command")
ctrl =rospy.Publisher('/message',String,queue_size=1)
msg = "rc 0 0 0 0"
def callback(data):
    global ctrl,msg
    x = int(data.linear.x)
    y = int(-data.linear.y)
    yaw = int(-data.angular.z/np.pi*180.0)

    msg = 'rc {} {} {} {}'.format(
        y,
        x,
        0,
        yaw
        )
    ctrl.publish(msg)

    
sub = rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=1)
rospy.spin()