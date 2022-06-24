import rospy
from geometry_msgs.msg import Twist
import pygame

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

def getKeyboardInput():
    speed  = Twist()
    if getKey("w"):
        speed.linear.x = -3
    elif getKey("s"):
        speed.linear.x = 3
    else:
        speed.linear.x = 0

    if getKey("a"):
        speed.angular.z = -3
    elif getKey("d"):
        speed.angular.z = 3
    else:
        speed.angular.z = 0

    return speed

rospy.init_node('light_keybord',anonymous=True)
rate = rospy.Rate(20)
pub = rospy.Publisher("rover_drive",Twist,queue_size=1)
init()

while not rospy.is_shutdown():
    speed = getKeyboardInput()
    pub.publish(speed)
    rate.sleep()

