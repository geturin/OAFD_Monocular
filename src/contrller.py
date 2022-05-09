#!/usr/bin/python3.8
import rospy
from std_msgs.msg import String
import pygame
from sensor_msgs.msg import Image

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
    lr, fb, ud, yv = 0, 0, 0, 0
    key_pressed = 0
    if getKey("UP"):
        msg="takeoff"
    elif getKey("DOWN"):
        msg="land"

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
    return InfoText

def callback(data):
    msg = getKeyboardInput(speed=40)
    ctrl_pub.publish(msg)



init()
rospy.init_node('contraller',anonymous=True)
ctrl_pub =rospy.Publisher('/message',String,queue_size=1)

while True:


    rospy.Subscriber("/camera/notime", Image, callback)
    rospy.spin()