import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import cv2
from scipy.spatial.transform import Rotation as Rot



class Camera:
    def __init__(self):
        self.camera_width = 512                                             #image width
        self.camera_height = 512                                            #image height
        self.camera_fov = 120                                                #field of view of camera
        self.camera_focal_depth = 0.5*self.camera_height/np.tan(0.5*np.pi/180*self.camera_fov) #focal depth in pixel space
        self.camera_aspect = self.camera_width/self.camera_height                     #aspect ratio
        self.camera_near = 0.02                                             #near clipping plane in meters, do not set non-zero
        self.camera_far = 100


    def get_camera_img_float(self,cameraPos, camereaOrn):
        ''' Gets the image and depth map from a camera at a position cameraPos (3) and cameraOrn (3x3) in space. '''
        __camera_view_matrix_opengl = p.computeViewMatrix(cameraEyePosition=cameraPos,
                                                    cameraTargetPosition=cameraPos+camereaOrn[:,2],
                                                    cameraUpVector=-camereaOrn[:,1])

        __camera_projection_matrix_opengl = p.computeProjectionMatrixFOV(self.camera_fov, self.camera_aspect, self.camera_near, self.camera_far)        
        width, height, rgbImg, nonlinDepthImg, _ = p.getCameraImage(self.camera_width, 
                                                    self.camera_height, 
                                                    __camera_view_matrix_opengl,
                                                    __camera_projection_matrix_opengl, 
                                                    renderer=p.ER_BULLET_HARDWARE_OPENGL)

        #adjust for clipping and nonlinear distance i.e., 1/d (0 is closest, i.e., near, 1 is furthest away, i.e., far
        depthImgLinearized = self.camera_far*self.camera_near/(self.camera_far+self.camera_near-(self.camera_far-self.camera_near)*nonlinDepthImg)

        rgb_image = np.array(rgbImg[:,:,:3], dtype=np.uint8)
        depth_image = np.array(depthImgLinearized, dtype=np.float32)

        return rgb_image, depth_image


class Environment:
    def __init__(self,time_step = 1/240):
        self.client = p.connect(p.GUI)
        self.time_step = time_step

        # Set the path to the URDF files included with PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())

        p.resetSimulation()
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf")

        self.objects = []
        self.camera = Camera()
        self.cameraPos = np.array([0, 0, 0.5])                              #camera position
        self.cameraOrn = Rot.from_euler('xyz', [90, 0, 0], degrees=True).as_matrix() #camera orientation

        

    def AddObject(self, dimension, position, orientation, color):

        """
        This function adds a box object to the environment.
        dimension: [box_length, box_width, box_depth]
        position: [x, y, z]
        orientation: [roll, pitch, yaw]
        color: [r, g, b] # 0-1
        """
        
        [box_length, box_width, box_depth] = dimension
        object_center = position
        object_orientation = orientation
        object_color = [*color,1]

        geomBox = p.createCollisionShape(p.GEOM_BOX, halfExtents=[box_length/2, box_width/2, box_depth/2])
        visualBox = p.createVisualShape(p.GEOM_BOX, halfExtents=[box_length/2, box_width/2, box_depth/2], rgbaColor=object_color)
        boxId = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=geomBox,
            baseVisualShapeIndex=visualBox,
            basePosition=np.array(object_center),
            baseOrientation=p.getQuaternionFromEuler(object_orientation)
        )

        self.objects.append(boxId)

    def RemoveObject(self, object_id):
        p.removeBody(object_id)
        self.objects.remove(object_id)
        
    def MoveObject(self, object_id, position, orientation):
        """
        This function moves the object to the given position and orientation.
        """
        p.resetBasePositionAndOrientation(object_id, position, orientation)





if __name__ == "__main__":
    env = Environment()
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.1], [0, 0, 0], [1, 0, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.2], [0, 0, 0], [0, 1, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.3], [0, 0, 0], [0, 0, 1])

    while True:
        p.stepSimulation()
        time.sleep(1/240)

    p.disconnect()