#!/usr/bin/python3.8
from djitellopy import tello
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String,Float32
from cv_bridge import CvBridge



#
me=tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()
command = "rc 0 0 0 0"
frame=0
# 创建发布者node 定义发布消息数据类型
rospy.init_node('tello_node',anonymous=True)
cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=1)
tof_pub=rospy.Publisher("/tof",Float32,queue_size=1)

rate = rospy.Rate(20)
bridge=CvBridge()

#count==0 take off
count=0

def callback(data):
    global command
    command = data.data

command_sub=rospy.Subscriber('/message',String,callback,queue_size=1)

while not rospy.is_shutdown():
    #pub img
    img = me.get_frame_read().frame
    pubimg = bridge.cv2_to_imgmsg(img, 'bgr8')
    pubimg.header.stamp = rospy.Time.now()
    cam_pub.publish(pubimg)

    #pub tof
    gettof=me.get_distance_tof()
    tof_pub.publish(gettof) 

    if count==0:
        me.send_command_without_return("takeoff")
        count=1

    me.send_command_without_return(command)
    rate.sleep()

