"""
It is the main file to run the robot
"""
import time
import cv2
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

    while True:
        p.stepSimulation()
        time.sleep(1/240)
        rgb, depth = robot.update_camera_feed()
        cv2.imshow("rgb", rgb)

        for object_id in env.objects:
            object_center, object_orientation = trajectory(object_id,
                                                           time.time(),
                                                           [0.1*object_id, 0.1*object_id, 0.1],
                                                           [0, 0, 0])
            env.move_object(object_id, object_center, object_orientation)
