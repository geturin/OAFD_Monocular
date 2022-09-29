#!/usr/bin/python3.8
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("/visual_slam/odom", Odometry, queue_size=1)
odom = Odometry()
odom.child_frame_id = "camera"

def callback(data):
    global odom_pub,odom
    odom.header = data.header
    # odom.header.frame_id = "odom"
    odom.pose.pose = data.pose
    odom_pub.publish(odom)


rospy.Subscriber("/orb_slam3_ros/camera",PoseStamped,callback)
rospy.spin()