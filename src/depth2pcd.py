#用numpy将连续的深度图转为点云以及和AI点云和稀疏点云的标定
import numpy as np
import cv2

#点云投影
class depth_to_pcd(object):

    def __init__(self, resize_scale):
        self.resize_scale = resize_scale
        self.resize_camera = np.array((910.072 / self.resize_scale, 0, 485.523 / self.resize_scale, 0,
                                       0, 914.094 / self.resize_scale, 336.718 / self.resize_scale, 0,
                                       0, 0, 1, 0), dtype="float").reshape(3, 4)

        self.pixel = np.array([0,0,1]).reshape(3,-1)
        self.resize_camera = np.matrix(self.resize_camera).I
        
        self.pcd_list = np.zeros((720//self.resize_scale,960//self.resize_scale,3))
        self.get_depth_vector_v2()
        
        self.pcd_list = self.pcd_list.reshape(-1,1,3)
        self.pcd_list[:,:,0],self.pcd_list[:,:,1],self.pcd_list[:,:,2]=self.pcd_list[:,:,2],-self.pcd_list[:,:,0],-self.pcd_list[:,:,1]
        self.pcd_list = self.pcd_list.reshape(720//self.resize_scale,960//self.resize_scale,3)
        return

    # def get_depth_vector(self):
    #     fake_depth =np.zeros((720//self.resize_scale,960//self.resize_scale))
    #     it = np.nditer(fake_depth, flags=['multi_index'])
    #     with it:
    #         while not it.finished:
    #             self.pixel[0] = it.multi_index[1]
    #             self.pixel[1] = it.multi_index[0]
    #             point = np.dot(self.resize_camera, self.pixel)          
    #             self.pcd_list[it.multi_index] = point[0:3].T[0]
    #             it.iternext()

        #new
    def get_depth_vector_v2(self):
        u = 960//self.resize_scale
        v = 720//self.resize_scale
        u_vector = np.ones((1,u,3))
        v_vector = np.ones((u,1,3))

        for i in range(u):
            self.pixel[0] = self.pixel[1] = i
            point = np.dot(self.resize_camera, self.pixel)
            u_vector[0,i,0] = point[0]
            v_vector[i,0,1] = point[1]

        v_vector = v_vector[:v,:,:]
        v_vector = np.tile(v_vector,(u,1))
        u_vector = np.tile(u_vector ,(v,1,1))
        self.pcd_list = v_vector * u_vector

    def get_pcd(self, depth):
        self.depth = cv2.resize(depth, (int(960 / self.resize_scale), int(720 / self.resize_scale)),interpolation=cv2.INTER_NEAREST)
        vector_array = self.pcd_list.copy()
        vector_array *= self.depth.reshape((int(720 / self.resize_scale), int(960 / self.resize_scale),1))
        
            
        pointcloud = vector_array.reshape(-1,3)
        return pointcloud

#标定

class Clibration(object):

    def __init__(self) -> None:
        self.rotation=np.array([0,0,0],dtype="float").reshape(3,1)
        self.translation=np.array([0,0,0],dtype="float").reshape(1,3)
        self.distortion=np.array([[-0.005941,0.055161,-0.006094,0.003192]])
        self.camera=np.array((910.072,0,485.523,
                0,914.094,336.718,
                0,0,1),dtype="float").reshape(3,3)
        pass
    
    def orb_pcd_reprojet(self,orb_pcd):
        camera_pcd=np.zeros((orb_pcd.shape[0],3))
        camera_pcd[:,0]=-1*orb_pcd["y"]
        camera_pcd[:,1]=-1*orb_pcd["z"]
        camera_pcd[:,2]=orb_pcd["x"]

    #之前写好的重投影代码效率不佳 下次重写


