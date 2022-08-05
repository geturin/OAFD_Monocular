import numpy as np
import cv2

class reProject(object):
    #camera parament



    def __init__(self) -> None:
        self.rotation=np.array([0,0,0],dtype="float").reshape(3,1)
        self.translation=np.array([0,0,0],dtype="float").reshape(1,3)
        self.distortion=np.array([[-0.005941,0.055161,-0.006094,0.003192]])
        self.camera=np.array((910.072,0,485.523,
                        0,914.094,336.718,
                        0,0,1),dtype="float").reshape(3,3)
        return
    
    def ros_to_camera(self):
        camera_pcd=np.zeros((self.pcd.shape[0],3))
        #ros coordinate to camera coordinate
        camera_pcd[:,0]=-1*self.pcd["y"]
        camera_pcd[:,1]=-1*self.pcd["z"]
        camera_pcd[:,2]=self.pcd["x"]

        self.pcd = camera_pcd
        return

    def project(self,pcd):
        self.pcd = pcd
        self.ros_to_camera()
        reTransform=cv2.projectPoints(self.pcd,self.rotation,self.translation,self.camera,self.distortion)
        self.pixel=reTransform[0]
        return self.pixel
    
    
