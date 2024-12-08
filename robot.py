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

        p.resetBasePositionAndOrientation(self.robot_id, [0, 0, 0], [0, 0, 0, 1])

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

    def get_nearest_object_using_camera(self,object_ids: np.ndarray, points_in_3d: np.ndarray) -> tuple:
        """
        Gets the object that is nearest to the robot based on camera readings
        :param object_ids: Array of object ids
        :param points_in_3d: Correspoding points in 3D world frame. Shape is Nx3
        :return Returns the object id, object location and location in camera. If no object is visible 
        it returns empty array
        """

        # Get Camera Position
        camera_position, camera_orientation = self.get_ee_position()

        # Set the projection Matrixes
        self.camera.get_camera_view_and_projection_opencv(camera_position=camera_position, 
                                                          camera_orientation=camera_orientation)

        # Get the Pixel coordinates of the objects
        pixel_coordinates, z_coordinate = self.camera.opengl_plot_world_to_pixelspace(points_in_3d)

        # This code filters and gets the nearest point
        objects_in_image_mask = ((pixel_coordinates[:, 0] >= 0) & (pixel_coordinates[:, 0] <= 512) & 
                          (pixel_coordinates[:, 1] >= 0) & (pixel_coordinates[:, 1] <= 512) &
                          (z_coordinate < 1))

        object_ids_in_frame = object_ids[objects_in_image_mask]
        points_objects_in_frame_3d = points_in_3d[objects_in_image_mask]
        
        if object_ids_in_frame.shape[0] > 0:
            
            distance = np.linalg.norm(points_objects_in_frame_3d - camera_position, axis=-1)
            minimum_distance_index = np.argmin(distance)

            nearest_object = np.array([object_ids_in_frame[minimum_distance_index]])
            nearest_point = points_objects_in_frame_3d[minimum_distance_index]
            nearest_object_pixel = pixel_coordinates[objects_in_image_mask][minimum_distance_index]
            return nearest_object, nearest_point, nearest_object_pixel
        else:
            return np.array([]), np.array([]), np.array([])
    
    def set_back_to_initial_position(self) -> None:
        """
        This function sets the robot back to its original_postion
        """
        # Reset the robot to initial joint positions
        for i in range(self.num_link_joints):
            p.resetJointState(self.robot_id,i,self.initial_joint_pos[i])

    def get_robot_jacobian(self):
        """
        This function returns the jacobian of the robot
        """

        joint_states = p.getJointStates(self.robot_id, range(self.num_link_joints))
        joint_infos = [p.getJointInfo(self.robot_id, i) for i in range(self.num_link_joints)]
        joint_states = [j for j, i in zip(joint_states, joint_infos) if i[3] > -1]
        joint_positions = [state[0] for state in joint_states]

        zero_vec = [0.0]*len(joint_positions)
        linearJacobian, angularJacobian = p.calculateJacobian(self.robot_id, 
                                                              self.num_active_joints,
                                                              [0,0,self.camera_offset],
                                                              joint_positions, 
                                                              zero_vec,
                                                              zero_vec)
        Jacobian = np.vstack((linearJacobian,angularJacobian))
        return Jacobian[:,:self.num_active_joints]  

    def move_robot(self, control_input: np.ndarray):
        """
        This function moves the robot given the control input
        :param control_input: It is the control input to the robot shape (6,)
        (x_dot, y_dot, z_dot, omega_x, omega_y, omega_z)
        :return None. Sets the robot position and orientation in place
        """
        jacobian = self.get_robot_jacobian()
        joint_velocities = np.linalg.pinv(jacobian) @ control_input
        
        for i in range(self.num_active_joints):
            p.setJointMotorControl2(self.robot_id, i, p.VELOCITY_CONTROL, targetVelocity=joint_velocities[i])




class HangingCamera():
    """
    This class is for the Camera without a robot
    """

    def __init__(self,
                 initial_camera_position: np.ndarray, 
                 initial_camera_orientation: np.ndarray):
        
        self.step_time = 1/240
        self.camera_position = initial_camera_position
        self.camera_orientation = initial_camera_orientation
        self.camera = CameraModule()
    
    def move_robot(self, control_input: np.ndarray):
        """
        This function moves the robot given the control input
        :param control_input: It is the control input to the robot shape (6,)
        (x_dot, y_dot, z_dot, umega_x, umega_y, umega_z)
        :return None. Sets the robot position and orientation in place
        """
        
        self.camera_position = self.camera_position + control_input[:3]*self.step_time
        change_angle_camera_frame = self.step_time*self.camera_orientation.T @ control_input[3:]
        required_quaternion = p.getQuaternionFromEuler(change_angle_camera_frame)
        required_new_matrix = p.getMatrixFromQuaternion(required_quaternion)    
        change_in_orientation = np.array([[required_new_matrix[0], required_new_matrix[1], required_new_matrix[2]], 
                                        [required_new_matrix[3], required_new_matrix[4], required_new_matrix[5]], 
                                        [required_new_matrix[6], required_new_matrix[7], required_new_matrix[8]]])
        self.camera_orientation = self.camera_orientation @ change_in_orientation

    def get_ee_position(self) -> tuple:
        '''
        Function to get the end effector position and orientation
        :return end effector position (np.ndarray) and orientation 3x3 (np.ndarray)
        '''
        return self.camera_position, self.camera_orientation

    def update_camera_feed(self) -> tuple:
        '''
        Function to update the feed from the camera
        :return Tuple of rgb and depth image
        '''
        camera_pos, camera_orn = self.get_ee_position()
        rgb, depth = self.camera.get_camera_img_float(camera_pos, camera_orn) 
        return rgb, depth

    def get_nearest_object_using_camera(self,object_ids: np.ndarray, points_in_3d: np.ndarray) -> tuple:
        """
        Gets the object that is nearest to the robot based on camera readings
        :param object_ids: Array of object ids
        :param points_in_3d: Correspoding points in 3D world frame. Shape is Nx3
        :return Returns the object id, object location and location in camera. If no object is visible 
        it returns empty array
        """

        # Get Camera Position
        camera_position, camera_orientation = self.get_ee_position()

        # Set the projection Matrixes
        self.camera.get_camera_view_and_projection_opencv(camera_position=camera_position, 
                                                          camera_orientation=camera_orientation)

        # Get the Pixel coordinates of the objects
        pixel_coordinates, z_coordinate = self.camera.opengl_plot_world_to_pixelspace(points_in_3d)

        # This code filters and gets the nearest point
        objects_in_image_mask = ((pixel_coordinates[:, 0] >= 0) & (pixel_coordinates[:, 0] <= 512) & 
                          (pixel_coordinates[:, 1] >= 0) & (pixel_coordinates[:, 1] <= 512) &
                          (z_coordinate < 1))

        object_ids_in_frame = object_ids[objects_in_image_mask]
        points_objects_in_frame_3d = points_in_3d[objects_in_image_mask]
        
        if object_ids_in_frame.shape[0] > 0:
            
            distance = np.linalg.norm(points_objects_in_frame_3d - camera_position, axis=-1)
            minimum_distance_index = np.argmin(distance)

            nearest_object = np.array([object_ids_in_frame[minimum_distance_index]])
            nearest_point = points_objects_in_frame_3d[minimum_distance_index]
            nearest_object_pixel = pixel_coordinates[objects_in_image_mask][minimum_distance_index]
            return nearest_object, nearest_point, nearest_object_pixel
        else:
            return np.array([]), np.array([]), np.array([])
