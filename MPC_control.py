import numpy as np
<<<<<<< HEAD
import casadi as ca
import pybullet as p
from camera import *
from utils import *

class Control:
    def __init__(self, N, time_step, sim_time):
        self.N = N # prediction horizon
        self.time_step = time_step # simulation time step
        self.sim_time = sim_time # current simulation time step
        self.ref_state = np.array([[0],[0]])
        self.state_lower_bound = [...]
        self.state_upper_bound = [...]
        self.control_lower_bound = [...]
        self.control_upper_bound = [...]
        self.Q = np.array(...)
        self.R = np.array(...)
        self.tempCamera = CameraModule()

    def getObjectCoords(self, object_id, i):
        pass

    def motionModel(self, objectWorldCoords, U, current_ee_pose):
        # curr_ -> current state of the object (u,v) pixels
        # objectWorldCoords -> (x,y,z) in the NEXT time step of the object in the world frame
        # U -> input given to camera (xdot, ydot, zdot, rolldot, pitchdot, yawdot) (np.array)
        # current_ee_pose -> current pose of the camera (x, y, z, roll, pitch, yaw) (np.array)
        # return : (u*, v*)

        # get the position of the camera in the world frame given the control input
        new_ee_pose = current_ee_pose + self.time_step*U    

        # calculate the pixel co-ordinates of the object given the positions of the the object and the camera
        camera_position = tuple(new_ee_pose[:3])
        camera_orientation = tuple(new_ee_pose[3:])
        self.tempCamera.get_camera_view_and_projection_opencv(camera_position=camera_position, camera_orientation=camera_orientation)
        pixel_coordinates, z_coordinate = self.tempCamera.opengl_plot_world_to_pixelspace(objectWorldCoords)
        
        pixel_coordinates = np.array(pixel_coordinates)
        pixel_coordinates.reshape(2,1)
        return pixel_coordinates

    def performControl(self, current_ee_pose):
        # initialize stuff
        U = ca.MX.sym('U', 6, self.N)
        g = []     # has predicted states of the next N time steps
        lbg = []   # lower bound of the state (in pixels u, v) for the next N time steps
        ubg = []   # upper bound of the state (in pixels u, v) for the next N time steps
        ubx = []   # upper bound of the control input for the next N time steps
        lbx = []   # lower bound of the control input for the next N time steps
        obj = 0    # objective function that will calculated over the next N time steps --> to minimize
        

        # set the casadi objects in loop        
        for i in range (self.N) :

            # get the co-ordinates of the object in the world frame at time i+sim_time
            objectWorldCoords = self.getObjectCoords(i)

            # get the next state given the control and the object location in world frame
            predicted_state = self.motionModel(objectWorldCoords, np.array([U[0,i], U[1,i], U[2,i], U[3,i], U[4,i], U[5,i]]), current_ee_pose)

            # calculate the error in state and add to the objective
            state_error = self.ref_state - predicted_state
            obj += state_error.T @ self.Q @ state_error + U[:,i].T @ self.R @ U[:,i]

            # add the predicted state and the bounds to casadi objects
            g.append(predicted_state[0]); lbg.append(self.state_lower_bound[0]); ubg.append(self.state_upper_bound[0])
            g.append(predicted_state[1]); lbg.append(self.state_lower_bound[1]); ubg.append(self.state_upper_bound[1])
            lbx.append(self.control_lower_bound[0]); ubx.append(self.control_upper_bound[0])
            lbx.append(self.control_lower_bound[1]); ubx.append(self.control_upper_bound[1])
            lbx.append(self.control_lower_bound[2]); ubx.append(self.control_upper_bound[2])
            lbx.append(self.control_lower_bound[3]); ubx.append(self.control_upper_bound[3])
            lbx.append(self.control_lower_bound[4]); ubx.append(self.control_upper_bound[4])
            lbx.append(self.control_lower_bound[5]); ubx.append(self.control_upper_bound[5])
        
        opt_variables = ca.vertcat(ca.reshape(U,-1,1))

        # initial guess for inputs and states
        u0 = ...

        # define optimization solver
        nlp = {'f':obj, 'x':opt_variables, 'g':ca.vertcat(*g)}
        solver = ca.nlpsol("S", "ipopt", nlp)
        sol = solver(
            x0= u0.T.reshape(-1,1),  # initial guess
            lbx=lbx, # lower bound on optimization variables
            ubx=ubx, # upper bound on optimization variables
            lbg=lbg, # lower bound on optimization constraints
            ubg=ubg, # upper bound on optimization constraints
        )

        # get the solution
        x = sol["x"] 

        # extract the control input and return
        x = x.full()
        u = x[:6]
        return u
    
if __name__ == "__main__":
    pass



        
=======
from camera import CameraModule

class Controller:
    def __init__(self,control_step):

        self.dt = control_step

        self.num_joints = 7
        self.Q = np.eye(self.num_joints)
        self.R = np.eye(self.num_joints)
        self.K = np.zeros((self.num_joints, 3))
        self.u_prev, self.v_prev = None, None

    def calculate_camera_velocity_in_pixel_space(self, u, v)->tuple:
        """
        Calculate the camera velocity using the current and previous pixel coordinates
        :param u: the current x pixel coordinate
        :param v: the current y pixel coordinate
        :return: a tuple of the velocity in the x and y direction
        """

        if self.u_prev is None:
            du, dv = 0, 0
        else:
            du = u - self.u_prev
            dv = v - self.v_prev
            return du, dv
        
        self.u_prev = u
        self.v_prev = v

        return du/self.dt, dv/self.dt
    

    def calculate_required_camera_movement(self,u,v,depth,camera_ori,camera:CameraModule):
        """
        Calculate the control input for the camera
        :param u: the current x pixel coordinate
        :param v: the current y pixel coordinate
        :param depth: the depth of the object
        :param camera_ori: the orientation of the camera
        :return: a tuple of the position and orientation of the camera in world frame
        """

        u = int(u - camera.camera_width/2)
        v = int(-(v - camera.camera_height/2))
        z = depth        
        f = camera.camera_focal_depth
        L = np.array([[ -f/z ,   0  ,  u/z  ,   u*v/f   , -(f+v*v/f),  v ],
                      [   0  , -f/z ,  v/z  , (f+v*v/f) ,   -u*v/f  , -u ]])        
        L_inv = np.linalg.pinv(L)

        delta_pix = self.camera_params['image_center'] - np.array([u,v])
        delta_pos = L_inv @ delta_pix
        delta_X = delta_pos[:3]
        delta_Omega = delta_pos[3:]
        
        return delta_X @ camera_ori , delta_Omega @ camera_ori
        
    def calculate_joint_velocity(self, joint_angles, delta_X, delta_Omega):
        raise NotImplementedError
>>>>>>> 0f871475d774f8a954ffec27cecbd11b01eb62c5
