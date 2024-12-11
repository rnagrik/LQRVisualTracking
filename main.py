"""
It is the main file to run the robot
"""
import time
import os
import cv2
import numpy as np
import pybullet as p
from environment import Environment
from robot import RobotWithCamera
from MPC_control import MPCControl
from baseline_control import PIDControl
from utils import trajectory
from evaluate import Recorder



# Choose controller from ["PID", "MPC1", "MPC2"]
CONTROLLER = "MPC2"
TIME_STEP = 1/240
ITERATIONS = 3000
USE_SAVED_DATA = True
SAVED_RUN_NUMBER = -1


# Initialize the recorder
recorder = Recorder(controller_type=CONTROLLER)
if USE_SAVED_DATA:
    recorder.load_data(run=SAVED_RUN_NUMBER)
    print("Loaded Data")
    recorder.evaluate_controller(show_plot=True, print_error=True)
    # save rgb video using cv2 in mp4 format
    video_writer = cv2.VideoWriter(
        f'{recorder.data_dir}/{CONTROLLER}_{recorder.iter-1}.mp4',  # Output video file name
        cv2.VideoWriter_fourcc(*'mp4v'),  # Codec for MP4 format
        1/(TIME_STEP*4),  # Frame rate
        (512, 512)  # Frame size
    )


if __name__ == "__main__":

    # Initialize the environment
    env = Environment()
    env.add_default_objects()
    object_locations = np.zeros((len(env.objects), 3))
    sim_time = 0

    # Initialize the robot and define the controller
    robot = RobotWithCamera()
    if CONTROLLER in ["MPC1", "MPC2"]:
        controller = MPCControl(N=10,
                                time_step=TIME_STEP,
                                dh_params=robot.DH_params)
    elif CONTROLLER == "PID":
        controller = PIDControl()
        controller.add_camera_information(focal_length=robot.camera.camera_focal_depth,
                                          imageWidth=robot.camera.camera_width,
                                          imageHeight=robot.camera.camera_height)

    # Run the simulation loop
    for i in range(ITERATIONS):
        # Update the camera feed and get the image
        rgb, depth = robot.update_camera_feed()

        # Move the objects in the environment
        for object_index, object_id in enumerate(env.objects):
            object_center, object_orientation = trajectory(object_id,
                                                           sim_time,
                                                           [0, 1, 0.5],
                                                           [0, 0, 0])
            env.move_object(object_id, object_center, object_orientation)
            object_locations[object_index, :] = object_center

        # Get the nearest object to the robot        
        nearest_object, nearest_point, nearest_pixel = robot.get_nearest_object_using_camera(
            object_ids=np.array(env.objects),
            points_in_3d=object_locations)
        print("Nearest Object : ", nearest_object)
        print(f"iteration : {i}")

        # Draw the nearest object on the image
        for pixel_idx in range(nearest_pixel.shape[0]):
            pixel_location = (int(nearest_pixel[0]), int(nearest_pixel[1]))
            cv2.circle(rgb, pixel_location, 4, (0, 0, 0), -1)
            cv2.circle(rgb, (256, 256), 4, (255, 0, 255), -1)
        
        # Get current camera pose and orientation
        cam_pos, cam_orientation = robot.get_ee_position_orientation()
    
        # Get control and move the robot
        if USE_SAVED_DATA:
            if CONTROLLER in ["PID","MPC1"]:
                u = recorder.control_data[i, :].reshape((6, 1))
                robot.move_robot(velocity=np.array(u[:, 0]))
            elif CONTROLLER == "MPC2":
                u = recorder.control_data[i, :].reshape((7, 1))
                robot.move_robot2(joint_velocities=np.array(u[:, 0]))
        elif nearest_object.shape[0] == 0:
            if CONTROLLER in ["PID","MPC1"]:
                u = np.zeros((6, 1))
            elif CONTROLLER == "MPC2":
                u = np.zeros((7, 1))
        elif CONTROLLER in ["MPC1", "MPC2"]:
            nearest_obj_params = {"object_id": nearest_object[0],
                              "current_simulation_time": sim_time,
                              "initial_position": [0, 1, 0.5],
                              "initial_orientation": [0, 0, 0]}
            if CONTROLLER == "MPC1":
                u = controller.getControl(current_ee_orientation=cam_orientation,
                                current_ee_position=cam_pos,
                                nearest_object_trajectory_params=nearest_obj_params)
                robot.move_robot(velocity=np.array(u[:, 0]))
            elif CONTROLLER == "MPC2":
                u = controller.getControl2(current_joint_position=robot.get_active_joint_states(),
                                           nearest_object_trajectory_params=nearest_obj_params)
                robot.move_robot2(joint_velocities=np.array(u[:, 0]))
        elif CONTROLLER == "PID":
            u = controller.getControl(object_loc=nearest_pixel,depth=depth,time_step=TIME_STEP,camera_orientation=cam_orientation)
            robot.move_robot(velocity=np.array(u[:, 0]))
        else:
            raise ValueError("Invalid Controller")
            
        p.stepSimulation()
        time.sleep(TIME_STEP)
        sim_time += TIME_STEP

        cv2.imshow("rgb", rgb)
        cv2.waitKey(1)

        if USE_SAVED_DATA:
            video_writer.write(rgb)

        if not USE_SAVED_DATA:
            if nearest_object.shape[0] != 0:
                recorder.record_pixel(nearest_pixel)
            recorder.record_control(u)
            if i % 10 == 0:
                recorder.evaluate_controller(show_plot=False, print_error=True)
                recorder.save_data()
    
    if not USE_SAVED_DATA:
        recorder.save_data()

    if USE_SAVED_DATA:
        video_writer.release()
        recorder.evaluate_controller(show_plot=True, print_error=True)
        