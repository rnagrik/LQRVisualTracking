import numpy as np
import casadi as ca
import pybullet as p

class Control:
    def __init__(self, N, time_step, sim_time):
        self.N = N # prediction horizon
        self.time_step = time_step # simulation time step
        self.sim_time = sim_time # current simulation time step
        self.ref_state = np.array([[0],[0]])

    def motionModel(self, curr_, objectWorldCoords, U, current_ee_pose):
        # curr_ -> current state of the object (u,v) pixels
        # objectWorldCoords -> (x,y,z) in the NEXT time step of the object in the world frame
        # U -> input given to camera (xdot, ydot, zdot, rolldot, pitchdot, yawdot) (np.array)
        # current_ee_pose -> current pose of the camera (x, y, z, roll, pitch, yaw) (np.array)
        # return : (u*, v*)

        # get the position of the camera in the world frame given the control input
        new_ee_pose = current_ee_pose + self.time_step*U    

        # calculate the pixel co-ordinates of the object given the positions of the the object and the camera
        



    def performControl(self, curr_state, curr_ref_state, current_ee_pose):
        # initialize stuff
        U = ca.MX.sym('U', 6, self.N)
        g = []     # has predicted states of the next N time steps
        lbg = []   # lower bound of the state (in pixels u, v) for the next N time steps
        ubg = []   # upper bound of the state (in pixels u, v) for the next N time steps
        ubx = []   # upper bound of the control input for the next N time steps
        lbx = []   # lower bound of the control input for the next N time steps
        obj = 0    # objective function that will calculated over the next N time steps --> to minimize

        curr_ = curr_state  # current state of the system

        # set the casadi objects in loop        
        for i in range (self.N) :

            # get the co-ordinates of the object in the world frame at time i+sim_time
            objectWorldCoords = self.getObjectCoords(i)

            # get the next state given the control and the object location in world frame
            predicted_state = self.motionModel(curr_, objectWorldCoords, np.array([U[0,i], U[1,i], U[2,i], U[3,i], U[4,i], U[5,i]]), current_ee_pose)

            # calculate the error in state and add to the objective
            state_error = self.ref_state - predicted_state
            obj += state_error.T @ self.Q @ state_error + U[:,i].T @ self.R @ U[:,i]

            # add the predicted state and the bounds to casadi objects
            g.append(predicted_state[0]); lbg.append(self.state_lower_bound[0]); ubg.append(self.state_upper_bound[0])
            g.append(predicted_state[1]); lbg.append(self.state_lower_bound[1]); ubg.append(self.state_upper_bound[1])

            # update the current state to the current prediction to be used for the next step
            curr_ = predicted_state


        
