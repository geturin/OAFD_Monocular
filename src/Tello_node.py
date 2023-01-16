#!/usr/bin/env python3
from djitellopy import tello
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String,Float32
from cv_bridge import CvBridge
from threading import Thread


class tello_ros:
    def __init__(self) -> None:
        # Initialize drone
        self.drone = tello.Tello()
        try:
            self.drone.connect()
        except:
            print("can't connect tello")
            return
        

        # Initialize parameter
        self.cam_pub = rospy.Publisher('/camera/image_raw',Image,queue_size=1)
        self._camera_rate = rospy.Rate(rospy.get_param("~camera_rate", 20))
        self._command_rate = rospy.Rate(rospy.get_param("~command_rate", 20))
        self._tof_rate = rospy.Rate(rospy.get_param("~tof_rate", 4))
        self.tof_pub = rospy.Publisher("/tof",Float32,queue_size=1)
        self.bridge = CvBridge()
        self.take_off = rospy.get_param("~take_off", True)
        self.command = "rc 0 0 0 0"

        #cam_pub
        self.drone.streamon()
        self.drone.get_frame_read().frame #wait stream
        self._cam_thread = Thread(target=self.cam_loop)
        self._cam_thread.start()


        #take off
        if self.take_off:
            self.drone.send_command_with_return("takeoff")
        else:
            pass

        #tof pub 
        self._tof_thread = Thread(target=self.tof_data)
        self._tof_thread.start()

        #send vel
        command_sub = rospy.Subscriber('/message',String,self.get_vel_command,queue_size=1)
        self.send_vel_command()

 


    def cam_loop(self):
        while not rospy.is_shutdown():
            try:
                #get image frome video stream and publish
                img = self.drone.get_frame_read().frame
                pubimg = self.bridge.cv2_to_imgmsg(img,"bgr8")
                pubimg.header.stamp = rospy.Time.now()
                self.cam_pub.publish(pubimg)
            except:
                print("video stream failed")
                pass

            self._camera_rate.sleep()



    def tof_data(self):
        #get&pub tof 
        while not rospy.is_shutdown():
            tof = self.drone.get_distance_tof()
            self.tof_pub.publish(tof)
            self._tof_rate.sleep()

    
    def send_vel_command(self):
        while not rospy.is_shutdown():
            self.drone.send_command_without_return(self.command)
            self._command_rate.sleep()
    
    def get_vel_command(self,data):
        self.command = data.data

    def shutdown(self):
        self.drone.streamoff()
        self.drone.send_command_without_return("land")

if __name__ == "__main__":
    rospy.init_node("tello_driver")
    node = tello_ros()
    rospy.spin()
    node.shutdown()