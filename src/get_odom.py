#!/usr/bin/python3.8
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import ros_numpy

rospy.init_node('odometry_publisher')
odom_pub = rospy.Publisher("/visual_slam/odom", Odometry, queue_size=1)
pose_pub = rospy.Publisher("/fake/camera",PoseStamped,queue_size=1)

odom = Odometry()
odom.child_frame_id = "camera"

fake_pose = PoseStamped()

def callback(data):
    global odom_pub,odom
    odom.header = data.header
    odom.pose.pose = data.pose

    fake_pose.header = data.header


    #缩放坐标轴
    point = ros_numpy.geometry.point_to_numpy(odom.pose.pose.position)
    point *=1
    odom.pose.pose.position = ros_numpy.geometry.numpy_to_point(point)

    fake_pose.pose = odom.pose.pose

    odom_pub.publish(odom)
    pose_pub.publish(fake_pose)


rospy.Subscriber("/orb_slam3_ros/camera",PoseStamped,callback)
rospy.spin()