#用numpy将连续的深度图转为点云以及和AI点云和稀疏点云的标定
import numpy as np
import cv2

#点云投影
class depth_to_pcd(object):

    def __init__(self, resize_scale):
        self.resize_scale = resize_scale
        self.resize_camera = np.array((910.072 / self.resize_scale, 0, 485.523 / self.resize_scale, 0,
                                       0, 914.094 / self.resize_scale, 336.718 / self.resize_scale, 0,
                                       0, 0, 1, 0), dtype=np.float32).reshape(3, 4)

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

        #去除所有的0，0，0点
        pointcloud = pointcloud[pointcloud.sum(axis=1)!=0,:]


        return pointcloud

    def get_rgbpcd(self,depth,rgb):
        self.depth = cv2.resize(depth, (int(960 / self.resize_scale), int(720 / self.resize_scale)),interpolation=cv2.INTER_NEAREST)
        vector_array = self.pcd_list.copy()
        vector_array *= self.depth.reshape((int(720 / self.resize_scale), int(960 / self.resize_scale),1))
        
            
        pointcloud = vector_array.reshape(-1,3)
        

        rgb = cv2.resize(rgb, (int(960 / self.resize_scale), int(720 / self.resize_scale)),interpolation=cv2.INTER_NEAREST)
        rgb_list = rgb.reshape(-1,3)

        #去除所有的0，0，0点
        rgb_list = rgb_list[pointcloud.sum(axis=1)!=0,:]
        pointcloud = pointcloud[pointcloud.sum(axis=1)!=0,:]


        r,g,b = rgb_list[:,0].astype(np.uint32), rgb_list[:,1].astype(np.uint32), rgb_list[:,2].astype(np.uint32)
        rgb = np.array((r << 16) | (g << 8 ) | (b << 0),dtype=np.uint32)
        color = rgb.reshape(-1,1)
        color.dtype = np.float32

        rgbpointcloud = np.hstack((pointcloud,color)).astype(np.float32)

        return rgbpointcloud

#标定

class Clibration(object):

    def __init__(self) -> None:
        self.rotation=np.array([0,0,0],dtype=np.float32).reshape(3,1)
        self.translation=np.array([0,0,0],dtype=np.float32).reshape(1,3)
        self.distortion=np.array([[-0.005941,0.055161,-0.006094,0.003192]])
        self.camera=np.array((910.072,0,485.523,
                0,914.094,336.718,
                0,0,1),dtype=np.float32).reshape(3,3)
        pass
    
    def orb_pcd_reprojet(self,orb_pcd):
        camera_pcd=np.zeros((orb_pcd.shape[0],3))
        camera_pcd[:,0]=-1*orb_pcd["y"]
        camera_pcd[:,1]=-1*orb_pcd["z"]
        camera_pcd[:,2]=orb_pcd["x"]

        reTransform=cv2.projectPoints(camera_pcd,self.rotation,self.translation,self.camera,self.distortion)
        reTransform = reTransform[0][:,0].astype(int)

        pixel = reTransform
        filter = np.where((pixel[:,0]<960)&(pixel[:,1]<720)&(pixel[:,0]>=0)&(pixel[:,1]>=0))
        pixel = pixel[filter]
        depth = camera_pcd[:,2].reshape(-1,1)[filter]

        self.depth_image=np.zeros((720,960))
        self.depth_image[pixel[:,1],pixel[:,0]] = depth[:,0]

        return self.depth_image

    def depth_calibration(self,ai_depth,orb_pcd):
        orb_depth = self.orb_pcd_reprojet(orb_pcd)
        #消除前20%远的点
        filter = np.percentile(ai_depth,80)
        ai_depth[ai_depth>filter]=0

        #去除无穷大值
        ai_depth[np.isinf(ai_depth)] = 0
        #选出ai深度图以及orb稀疏深度图都有深度值的地方  
        filter = np.where((ai_depth!=0)&(orb_depth!=0))
        ai_list = ai_depth[filter]
        orb_list = orb_depth[filter]

        ai_list = ai_list.reshape(-1,1)
        orb_list = orb_list.reshape(-1,1)

        #最小二乘计算尺度
        scale=np.linalg.lstsq(ai_list,orb_list)[0]

        return scale


