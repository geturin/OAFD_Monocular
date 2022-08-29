#!/usr/bin/python3.8
from djitellopy import tello
import os
import rospy
from sensor_msgs.msg import Image,PointCloud2
from std_msgs.msg import String,Float32
import sensor_msgs.point_cloud2 as pcl2
from std_msgs.msg import Header
import sys
import cv2
import numpy as np
from cv_bridge import CvBridge
import message_filters
import pygame







me=tello.Tello()
me.connect()
print(me.get_battery())
me.streamon()

frame=0
# 创建发布者node 定义发布消息数据类型
rospy.init_node('tello_node',anonymous=True)
#cam_pub =rospy.Publisher('/camera/image_raw',Image,queue_size=1)
cam_pub =rospy.Publisher('/camera/notime',Image,queue_size=1)

rate = rospy.Rate(20)
bridge=CvBridge()


def init():
    pygame.init()
    win = pygame.display.set_mode((400, 400))

def getKey(keyName):
    ans = False
    for eve in pygame.event.get(): pass
    keyInput = pygame.key.get_pressed()
    myKey = getattr(pygame,'K_{}'.format(keyName))
    if keyInput[myKey]:
        ans = True
    pygame.display.update()
    return ans

def getKeyboardInput(speed):
    global rate

    lr, fb, ud, yv,count = 0, 0, 0, 0,0
    key_pressed = 0
    if getKey("UP"):
        count=2
    if getKey("DOWN"):
        count=1

    if getKey("j"):
        key_pressed = 1
        lr = -speed
    elif getKey("l"):
        key_pressed = 1
        lr = speed

    if getKey("i"):
        key_pressed = 1
        fb = speed
    elif getKey("k"):
        key_pressed = 1
        fb = -speed

    if getKey("w"):
        key_pressed = 1
        ud = speed
    elif getKey("s"):
        key_pressed = 1
        ud = -speed

    if getKey("a"):
        key_pressed = 1
        yv = -speed*0.6
    elif getKey("d"):
        key_pressed = 1
        yv = speed*0.6

    if key_pressed == 1:
        InfoText = "rc {a} {b} {c} {d}".format(a=lr, b=fb, c=ud, d=yv)
    else:
        InfoText="rc 0 0 0 0 0"

    if count == 1:
        InfoText="land"
        rate = rospy.Rate(1)
    if count ==2:
        InfoText="takeoff"

    return InfoText


init()

def callback(data):
    me.send_command_without_return(data.data)


def tof():
    gettof=me.get_distance_tof()
    #msg="{a}".format(a=gettof)
    tof_pub=rospy.Publisher("/tof",Float32,queue_size=2)
    tof_pub.publish(gettof)

while not rospy.is_shutdown():
    img = me.get_frame_read().frame
    pubimg = bridge.cv2_to_imgmsg(img, 'bgr8')
    pubimg.header.stamp = rospy.Time.now()
    cam_pub.publish(pubimg)

    #rospy.Subscriber("/message",String,callback,queue_size=2)
    if frame ==1:
        msg = getKeyboardInput(speed=40)
        me.send_command_without_return(msg)
        tof()

    if frame==0:
        me.send_command_without_return("takeoff")
        frame=frame+1

    rospy.loginfo(' publisher')
    rate.sleep()

