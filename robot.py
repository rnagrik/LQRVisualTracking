"""
This file has the robot class. The robot has a camera that gives RGBD image
"""
import os
import numpy as np
import pybullet as p
import pybullet_data
from camera import CameraModule


class RobotWithCamera():
    """
    This class is for the robot with Camera
    """

    def __init__(self):

        self.robot_id = p.loadURDF(
            os.path.join(pybullet_data.getDataPath(), "franka_panda/panda.urdf"),
            useFixedBase=True
            )

        self.initial_joint_pos = [0, -np.pi/4, np.pi/4,
                                  -np.pi/4, np.pi/4, np.pi/4,
                                  np.pi/4,0,0,
                                  0,0,0]

        self.num_link_joints, \
            self.active_joint_indices, \
                self.num_active_joints = self.initialize_robot()

        #offset camera in z direction to avoid grippers
        self.camera_offset = 0.1
        self.camera = CameraModule()

        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])
        # need to do this to initialize robot
        p.stepSimulation()

    def initialize_robot(self) -> tuple:
        """
        Get the number of joints and joint information
        :return Tuple of Number of total joints, indices of active joints and 
        number of active joints
        """

        #includes passive joint
        num_link_joints = p.getNumJoints(self.robot_id)
        joint_info = [p.getJointInfo(self.robot_id, i) for i in range(num_link_joints)]

        # Get the active joint indices
        active_joint_indices = []

        for i in range(num_link_joints):
            if joint_info[i][2]==p.JOINT_REVOLUTE:
                active_joint_indices.append(joint_info[i][0])

        #exact number of active joints
        num_active_joints = len(active_joint_indices)

        # Reset the robot to initial joint positions
        for i in range(num_link_joints):
            p.resetJointState(self.robot_id,i,self.initial_joint_pos[i])

        return num_link_joints, active_joint_indices, num_active_joints

    def get_ee_position(self) -> tuple:
        '''
        Function to get the end effector position and orientation
        :return end effector position (np.ndarray) and orientation 3x3 (np.ndarray)
        '''
        end_effector_index = self.num_active_joints
        end_effector_state = p.getLinkState(self.robot_id, end_effector_index)
        end_effector_pos = np.array(end_effector_state[0])
        end_effector_orn = np.array(p.getMatrixFromQuaternion(end_effector_state[1])).reshape(3,3)

        #add an offset to get past the forceps
        end_effector_pos += self.camera_offset*end_effector_orn[:,2]
        return end_effector_pos, end_effector_orn

    def update_camera_feed(self) -> tuple:
        '''
        Function to update the feed from the camera
        :return Tuple of rgb and depth image
        '''
        camera_pos, camera_orn = self.get_ee_position()
        rgb, depth = self.camera.get_camera_img_float(camera_pos, camera_orn)
        return rgb, depth
