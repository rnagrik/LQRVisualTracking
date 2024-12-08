"""
It is the main file to run the robot
"""
import time
import cv2
import numpy as np
import pybullet as p
from environment import Environment
from robot import RobotWithCamera
from utils import trajectory
from MPC_control import Control


if __name__ == "__main__":

    sim_time = 0
    env = Environment()
    initial_camera_position = np.array([0,0.5,1])
    initial_camera_orientation = np.array([[1,0,0],
                                           [0,-1,0],
                                           [0,0,-1]])
    robot = RobotWithCamera()

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
            u = [[0],[0],[0],[0],[0],[0]]
            u= np.array(u)
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
