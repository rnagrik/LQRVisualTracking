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


if __name__ == "__main__":
    env = Environment()
    robot = RobotWithCamera()

    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.1], [0, 0, 0], [1, 0, 0])
    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.2], [0, 0, 0], [0, 1, 0])
    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.3], [0, 0, 0], [0, 0, 1])

    object_locations = np.zeros((len(env.objects), 3))
    while True:
        robot.set_back_to_initial_position()
        p.stepSimulation()
        time.sleep(1/240)
        rgb, depth = robot.update_camera_feed()

        for object_index, object_id in enumerate(env.objects):
            object_center, object_orientation = trajectory(object_id,
                                                           time.time(),
                                                           [0.1*object_id, 0.1*object_id, 0.1],
                                                           [0, 0, 0])
            env.move_object(object_id, object_center, object_orientation)
            object_locations[object_index, :] = object_center
            
        nearest_object, nearest_point, nearest_pixel = robot.get_nearest_object_using_camera(
            object_ids=np.array(env.objects),
            points_in_3d=object_locations)

        for pixel_idx in range(nearest_pixel.shape[0]):
            pixel_location = (nearest_pixel[0], nearest_pixel[1])
            cv2.circle(rgb, pixel_location, 4, (0, 0, 0), -1)
        cv2.imshow("rgb", rgb)
        cv2.waitKey(1)

