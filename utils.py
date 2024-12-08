"""
This file contains utility functions for the project
"""
import numpy as np
import pybullet as p


def trajectory(object_id: int,
               time: float,
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

def update_orientation_from_angles(current_orientation: np.ndarray,
                                   change_in_angles:np.ndarray) -> np.ndarray:
    """
    This function changes the orientation by the amount change in angles.
    :param current_orientation: This is the current orientation of the 
    object in world frame. Shape (3x3)
    :param change_in_angles: This is the change in angles in the frame 
    of the object. Shape (3,)
    :return new orientation of the object in the world frame
    """
    theta_x, theta_y, theta_z = \
        change_in_angles[0], change_in_angles[1], change_in_angles[2]
    
    rotation_about_x = np.array([
        [1, 0, 0],
        [0, np.cos(theta_x), -np.sin(theta_x)],
        [0, np.sin(theta_x), np.cos(theta_x)]])

    rotation_about_Y = np.array([
        [np.cos(theta_y), 0, np.sin(theta_y)],
        [0, 1, 0],
        [-np.sin(theta_y), 0, np.cos(theta_y)]
        ])

    rotation_about_z = np.array([
        [np.cos(theta_z), -np.sin(theta_z), 0],
        [np.sin(theta_z), np.cos(theta_z), 0],
        [0, 0, 1]
    ])

    # chnage in orientation in the world frame
    r_delta = rotation_about_z @ rotation_about_Y @ rotation_about_x 
    
    # New orientation matrix
    new_orientation = current_orientation @ r_delta
    return new_orientation
