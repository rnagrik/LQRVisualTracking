import numpy as np
import casadi as ca
import pybullet as p
import cv2
from camera import CameraModule
from utils import trajectory, update_orientation_from_angles

class Control:
    def __init__(self, N, time_step):
        self.N = N # prediction horizon
        self.time_step = time_step # simulation time step
        self.ref_state = np.array([[256],[256]])
        self.state_lower_bound = [0, 0]
        self.state_upper_bound = [512, 512]
        self.control_lower_bound = [-1, -1, -1, -1, -1, -1]
        self.control_upper_bound = [1, 1, 1, 1, 1, 1]
        self.Q = 0.5*np.eye(2)
        self.R = np.eye(6)
        self.tempCamera = CameraModule()

    def getObjectCoords(self, trajectory_params: dict, current_iteration: int) -> np.ndarray:
        """
        This function calculates the trjaectory of the object at the given time instance
        :params trajectory_params: all the information required for trajectory information
        :params current_iteration: current iteration at which we need the object location
        :return trajectory of object at the given location shape (1x3)
        """
        object_id = trajectory_params["object_id"]
        current_time = trajectory_params["current_simulation_time"] + current_iteration*self.time_step
        initial_position = trajectory_params["initial_position"]
        initial_orientation = trajectory_params["initial_orientation"]
        object_center, _ = trajectory(object_id,
                                      current_time,
                                      initial_position,
                                      initial_orientation)
        
        object_center = np.array(object_center)[np.newaxis, :]
        
        return object_center

    def motionModel(self, 
                    objectWorldCoords: np.ndarray,
                    U: np.ndarray, 
                    current_ee_position:np.ndarray,
                    current_ee_orientation:np.ndarray) -> tuple:
        """
        This function is the motion model for the problem. Calculates the next state.
        :param objectWorldCoords: (x,y,z) in the NEXT time step of the object in the world frame
        :param U: input given to camera (xdot, ydot, zdot, rolldot, pitchdot, yawdot)
        :param current_ee_position: current position of the camera (3, )
        :param current_ee_orientation: current orientation of the camera (3x3)
        return : new state of the object in image (u*, v*), new state of the robot
        """
        # get the position of the camera in the world frame given the control input
        new_ee_position = current_ee_position + self.time_step*U[:3]    
        change_angles_local_frame = self.time_step*current_ee_orientation.T @ U[3:]
        
        # Get new orientation from change in angles in local frame
        new_ee_orientation =  update_orientation_from_angles(
            current_ee_orientation,
            change_angles_local_frame)

        # calculate the pixel co-ordinates of the object given the positions of the the object and the camera
        self.tempCamera.get_camera_view_and_projection_opencv(
            camera_position=new_ee_position,
            camera_orientation=new_ee_orientation
            )
        pixel_coordinates, _ = self.tempCamera.opengl_plot_world_to_pixelspace(objectWorldCoords)
        
        pixel_coordinates = pixel_coordinates.T
        # pixel_coordinates.reshape(2,1)
        return pixel_coordinates, new_ee_position, new_ee_orientation

    def performControl(self, 
                       current_ee_position: np.ndarray, 
                       current_ee_orientation: np.ndarray, 
                       nearest_object_trajectory_params: dict):
        """
        This calculates the MPC control. 
        :param current_ee_position: current position of camera in world frame. (3,) shape
        :param current_ee_orientation: current orientation of camera in world frame (3x3) shape
        :param simulation_time: Current simulation time
        :param nearest_object_trajectory_params: This contains all the information required for getting 
        trajectory of the object
        :return control output
        """
        # initialize stuff
        U = ca.MX.sym('U', 6, self.N)
        g = []     # has predicted states of the next N time steps
        lbg = []   # lower bound of the state (in pixels u, v) for the next N time steps
        ubg = []   # upper bound of the state (in pixels u, v) for the next N time steps
        ubx = []   # upper bound of the control input for the next N time steps
        lbx = []   # lower bound of the control input for the next N time steps
        obj = 0    # objective function that will calculated over the next N time steps --> to minimize
        

        # set the casadi objects in loop        
        for i in range (1,self.N+1) :

            # get the co-ordinates of the object in the world frame at time i+sim_time
            objectWorldCoords = self.getObjectCoords(nearest_object_trajectory_params, i)
            # get the next state given the control and the object location in world frame
            predicted_state, current_ee_position, current_ee_orientation = self.motionModel(
                objectWorldCoords,
                np.array([U[0,i-1], U[1,i-1], U[2,i-1], U[3,i-1], U[4,i-1], U[5,i-1]]),
                current_ee_position,
                current_ee_orientation
            )

            # calculate the error in state and add to the objective
            state_error = self.ref_state - predicted_state
            a = state_error.T @ self.Q @ state_error
            b  = U[:,i-1].T @ self.R @ U[:,i-1]            
            obj += a[0,0] + b[0,0]
            # obj += state_error.T @ self.Q @ state_error + U[:,i].T @ self.R @ U[:,i]

            # add the predicted state and the bounds to casadi objects
            g.append(predicted_state[0,0]); lbg.append(self.state_lower_bound[0]); ubg.append(self.state_upper_bound[0])
            g.append(predicted_state[1,0]); lbg.append(self.state_lower_bound[1]); ubg.append(self.state_upper_bound[1])
            lbx.append(self.control_lower_bound[0]); ubx.append(self.control_upper_bound[0])
            lbx.append(self.control_lower_bound[1]); ubx.append(self.control_upper_bound[1])
            lbx.append(self.control_lower_bound[2]); ubx.append(self.control_upper_bound[2])
            lbx.append(self.control_lower_bound[3]); ubx.append(self.control_upper_bound[3])
            lbx.append(self.control_lower_bound[4]); ubx.append(self.control_upper_bound[4])
            lbx.append(self.control_lower_bound[5]); ubx.append(self.control_upper_bound[5])
        
        opt_variables = ca.vertcat(ca.reshape(U,-1,1))

        # initial guess for inputs and states
        u0 = np.array([0, 0, 0, 0, 0, 0]*self.N).reshape(-1, 6).T

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
    """
    Testing the MPC controller
    """
    from environment import Environment
    from robot import HangingCamera
    import time

    sim_time = 0
    env = Environment()
    initial_camera_position = np.array([0,0.5,1])
    initial_camera_orientation = np.array([[1,0,0],
                                           [0,-1,0],
                                           [0,0,-1]])
    robot = HangingCamera(initial_camera_position, initial_camera_orientation)

    controller = Control(N=10,
                         time_step=1/240)
    
    env.add_object([0.1, 0.1, 0.01], [0, 0, 0.01], [0, 0, 0], [1, 0, 0])
    env.add_object([0.1, 0.1, 0.01], [0, 0, 0.02], [0, 0, 0], [0, 1, 0])
    env.add_object([0.1, 0.1, 0.01], [0, 0, 0.03], [0, 0, 0], [0, 0, 1])

    object_locations = np.zeros((len(env.objects), 3))
    while True:
        rgb, depth = robot.update_camera_feed()

        for object_index, object_id in enumerate(env.objects):
            object_center, object_orientation = trajectory(object_id,
                                                           sim_time,
                                                           [0, 1, 0.5],
                                                           [0, 0, 0])
            env.move_object(object_id, object_center, object_orientation)
            object_locations[object_index, :] = object_center
            
        nearest_object, nearest_point, nearest_pixel = robot.get_nearest_object_using_camera(
            object_ids=np.array(env.objects),
            points_in_3d=object_locations)

        for pixel_idx in range(nearest_pixel.shape[0]):
            pixel_location = (int(nearest_pixel[0]), int(nearest_pixel[1]))
            cv2.circle(rgb, pixel_location, 4, (0, 0, 0), -1)
            cv2.circle(rgb, (256, 256), 4, (255, 0, 255), -1)
        
        # Get current camera pose
        cam_pos, cam_orientation = robot.get_ee_position()

        # Get parameters of the nearest object to track        
        

        # Get control and move the robot
        print("Nearest Object : ", nearest_object)
        if nearest_object.shape[0] == 0:
            u = [0,0,0,0,0,0]
        else :
            nearest_obj_params = {"object_id": nearest_object[0],
                              "current_simulation_time": sim_time,
                              "initial_position": [0, 1, 0.5],
                              "initial_orientation": [0, 0, 0]}
            u = controller.performControl(current_ee_orientation=cam_orientation,
                                        current_ee_position=cam_pos,
                                        nearest_object_trajectory_params=nearest_obj_params)
        robot.move_robot(np.array(u[:, 0]))
        
        p.stepSimulation()
        time.sleep(1/240)

        sim_time += 1/240
        
        cv2.imshow("rgb", rgb)
        cv2.waitKey(1)