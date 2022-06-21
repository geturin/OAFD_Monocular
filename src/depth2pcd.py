import numpy as np
import cv2

class depth_to_pcd(object):

    def __init__(self, resize_scale):
        self.resize_scale = resize_scale
        self.resize_camera = np.array((910.072 / self.resize_scale, 0, 485.523 / self.resize_scale, 0,
                                       0, 914.094 / self.resize_scale, 336.718 / self.resize_scale, 0,
                                       0, 0, 1, 0), dtype="float").reshape(3, 4)

        self.pixel = np.array([0,0,1]).reshape(3,-1)
        self.resize_camera = np.matrix(self.resize_camera).I
        
        self.pcd_list = np.zeros((720//self.resize_scale,960//self.resize_scale,3))
        self.get_depth_vector()
        
        self.pcd_list = self.pcd_list.reshape(-1,1,3)
        self.pcd_list[:,:,0],self.pcd_list[:,:,1],self.pcd_list[:,:,2]=self.pcd_list[:,:,2],-self.pcd_list[:,:,0],-self.pcd_list[:,:,1]
        self.pcd_list = self.pcd_list.reshape(720//self.resize_scale,960//self.resize_scale,3)
        return

    def get_depth_vector(self):
        fake_depth =np.zeros((720//self.resize_scale,960//self.resize_scale))
        it = np.nditer(fake_depth, flags=['multi_index'])
        with it:
            while not it.finished:
                self.pixel[0] = it.multi_index[1]
                self.pixel[1] = it.multi_index[0]
                point = np.dot(self.resize_camera, self.pixel)          
                self.pcd_list[it.multi_index] = point[0:3].T[0]
                it.iternext()

    def get_pcd(self, depth):
        self.depth = cv2.resize(depth, (int(960 / self.resize_scale), int(720 / self.resize_scale)),interpolation=cv2.INTER_NEAREST)
        vector_array = self.pcd_list.copy()
        vector_array *= self.depth.reshape((int(720 / self.resize_scale), int(960 / self.resize_scale),1))
        
            
        pointcloud = vector_array.reshape(-1,3)
        return pointcloud