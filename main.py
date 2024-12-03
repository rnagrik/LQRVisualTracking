import numpy as np
from EnvCam import Environment
from utils import Trajectory
import pybullet as p
import time
import  cv2



if __name__ == "__main__":
    env = Environment()

    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.1], [0, 0, 0], [1, 0, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.2], [0, 0, 0], [0, 1, 0])
    env.AddObject([0.1, 0.1, 0.1], [0, 0, 0.3], [0, 0, 0], [0, 0, 1])

    while True:
        p.stepSimulation()
        time.sleep(1/240)


        rgb, depth = env.camera.get_camera_img_float()
        cv2.imshow("rgb", rgb)

        for object_id in env.objects:
            object_center, object_orientation = Trajectory(object_id, 
                                                           time.time(), 
                                                           [0.1*object_id, 0.1*object_id, 0.1], 
                                                           [0, 0, 0])
            env.MoveObject(object_id, object_center, object_orientation)