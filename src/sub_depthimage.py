import rospy
from sensor_msgs.msg import CompressedImage
import numpy as np
from scipy.ndimage import filters
import cv2
import  roslib
import matplotlib.pyplot as plt
import base64



def callback(data):
    np_img = np.frombuffer(data.data, np.uint8)
    img = cv2.imdecode(np_img, cv2.IMREAD_COLOR)
    #plt.imshow(np_arr)
    #plt.pause(0.0001)
    #plt.clf()
    #cv2.imshow(image_np)
    #cv2.waitKey(1)
    print(len(np_img))


rospy.init_node('image_feature', anonymous=True)
rospy.Subscriber("/camera/re/depth", CompressedImage, callback)
rospy.spin()