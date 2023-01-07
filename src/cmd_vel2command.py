#!/usr/bin/python3.8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import numpy as np


rospy.init_node("cmd_vel_to_command")
ctrl =rospy.Publisher('/message',String,queue_size=1)


def callback(data):
    global ctrl
    data = Twist()
    x = data.linear.x
    y = -data.linear.y
    yaw = -data.angular.z/np.pi*180.0

    msg = 'rc {} {} {} {}'.format(
        y,
        x,
        0,
        yaw
        )

    ctrl.publish(msg)

    

sub = rospy.Subscriber("/cmd_vel", Twist, callback, queue_size=3)
rospy.spin()