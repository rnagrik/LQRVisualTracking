import numpy as np
import pybullet as p
import pybullet_data
import os
import time
import cv2
from scipy.spatial.transform import Rotation as Rot


class Camera:
    def __init__(self):
        self.set_camera_settings()

    def set_camera_settings(self, camera_width=512, camera_height=512, camera_fov=120, camera_near=0.02, camera_far=100):
        """
        This function sets the camera settings.
        camera_width: int, image width
        camera_height: int, image height
        camera_fov: int, field of view of camera
        camera_near: float, near clipping plane in meters, do not set non-zero
        camera_far: float, far clipping plane in meters
        """
        self.camera_width = camera_width    
        self.camera_height = camera_height  
        self.camera_fov = camera_fov       
        self.camera_near = camera_near
        self.camera_far = camera_far
        self.camera_focal_depth = 0.5*self.camera_height/np.tan(0.5*np.pi/180*self.camera_fov) #focal depth in pixel space
        self.camera_aspect = self.camera_width/self.camera_height                              #aspect ratio

    def get_camera_img_float(self,cameraPos, cameraOrn):
        ''' Gets the image and depth map from a camera at a position cameraPos (3) and cameraOrn (3x3) in space. '''
        __camera_view_matrix_opengl = p.computeViewMatrix(cameraEyePosition=cameraPos,
                                                    cameraTargetPosition=cameraPos+cameraOrn[:,2],
                                                    cameraUpVector=-cameraOrn[:,1])

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


class RobotwithCamera():
         
    def __init__(self):
        self.robot_id = p.loadURDF(os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf"),useFixedBase=True)
        self.initialJointPos = [0,-np.pi/4,np.pi/4,-np.pi/4,np.pi/4,np.pi/4,np.pi/4,0,0,0,0,0]
        self.numLinkJoints, self.active_joint_indices, self.numActiveJoints = self.initialize_robot()

        self.camera_offset = 0.1 #offset camera in z direction to avoid grippers
        self.camera = Camera()
        
        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        p.stepSimulation() # need to do this to initialize robot


    def initialize_robot(self):
        # Get the number of joints and joint information
        numLinkJoints = p.getNumJoints(self.robot_id) #includes passive joint
        jointInfo = [p.getJointInfo(self.robot_id, i) for i in range(numLinkJoints)]
        
        # Get the active joint indices
        active_joint_indices = []
        for i in range(numLinkJoints):
            if jointInfo[i][2]==p.JOINT_REVOLUTE:
                active_joint_indices.append(jointInfo[i][0])
        numActiveJoints = len(active_joint_indices) #exact number of active joints

        # Reset the robot to initial joint positions
        for i in range(numLinkJoints):
            p.resetJointState(self.robot_id,i,self.initialJointPos[i])
        
        return numLinkJoints, active_joint_indices, numActiveJoints

    def get_ee_position(self):
        '''
        Function to get the end effector position and orientation
        '''
        endEffectorIndex = self.numActiveJoints
        endEffectorState = p.getLinkState(self.robot_id, endEffectorIndex)
        endEffectorPos = np.array(endEffectorState[0])
        endEffectorOrn = np.array(p.getMatrixFromQuaternion(endEffectorState[1])).reshape(3,3)
        
        #add an offset to get past the forceps
        endEffectorPos += self.camera_offset*endEffectorOrn[:,2]
        return endEffectorPos, endEffectorOrn
    
    def update_camera_feed(self):
        '''
        Function to update the feed from the camera
        '''
        cameraPos, cameraOrn = self.get_ee_position()
        rgb, depth = self.camera.get_camera_img_float(cameraPos, cameraOrn)
        return rgb, depth



class Environment:
    def __init__(self,time_step = 1/240):

        self.time_step = time_step
        self.client = p.connect(p.GUI)

        # Set the path to the URDF files included with PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf") # Load the ground plane

        self.objects = []


    def AddObject(self, dimension, position, orientation, color):
        """
        This function adds a box object to the environment.
        dimension: [box_length, box_width, box_depth]
        position: [x, y, z]
        orientation: [roll, pitch, yaw]
        color: [r, g, b] # range-> 0-1
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
        
    def MoveObject(self, object_id:int, position:list, orientation:list):
        """
        This function moves the object to the given position and orientation.
        """
        p.resetBasePositionAndOrientation(object_id, position, orientation)





if __name__ == "__main__":
    env = Environment()
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.1], [0, 0, 0], [1, 0, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.2], [0, 0, 0], [0, 1, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.3], [0, 0, 0], [0, 0, 1])

    while not KeyboardInterrupt:
        p.stepSimulation()
        time.sleep(1/240)

    p.disconnect()