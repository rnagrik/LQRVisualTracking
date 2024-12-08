"""
This file contains utility functions for the project
"""
import numpy as np
import pybullet as p


def trajectory(object_id: int,
               time: int,
               initial_position: list,
               initial_orientation: list) -> tuple:
    """
    This function returns the position and orientation of the object at a given time.
    :param object_id: the id of the object
    :param time: the current time
    :param initial_position: list, the initial position of the object
    :param initial_orientation: list, the initial orientation of the object
    :return A tuple of object position in 3D coordinates (list) and orientation in quarternion 
    """
    [x, y, z] = initial_position
    object_center=[x + 0.2*object_id*np.sin((object_id*np.pi/2)+time/6),
                   y + 0.2*object_id*np.sin(time/6),
                   z + 0.1*np.cos((object_id*np.pi)/6 + time/6)]

    object_orientation=initial_orientation

    # convert the euler angles to quaternion
    object_orientation = p.getQuaternionFromEuler(object_orientation)

    return object_center, object_orientation
