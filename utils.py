import numpy as np
import pybullet as p

def Trajectory(object_id,time:int,initial_position,initial_orientation):
    """
    This function returns the position and orientation of the object at a given time.
    object_id: int, the id of the object
    time: int, the current time
    initial_position: list, the initial position of the object
    initial_orientation: list, the initial orientation of the object
    """
    [x, y, z] = initial_position
    object_center=[x + object_id*np.sin(np.pi/2+time/10),
                   y + object_id*np.sin(time/20), 
                   object_id + z*0.1*np.sin(time/10)]

    object_orientation=initial_orientation

    # convert the euler angles to quaternion
    object_orientation = p.getQuaternionFromEuler(object_orientation)

    return object_center, object_orientation