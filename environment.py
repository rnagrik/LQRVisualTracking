"""
This file contains the environment for the project. 
"""
import time
import numpy as np
import pybullet as p
import pybullet_data


class Environment:
    """
    This class creates the environment
    We can add objects and move objects in the environment
    """
    def __init__(self,time_step = 1/240):

        self.time_step = time_step
        self.client = p.connect(p.GUI)

        # Set the path to the URDF files included with PyBullet
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.resetSimulation()
        p.setTimeStep(self.time_step)
        p.setGravity(0, 0, -9.81)
        p.loadURDF("plane.urdf") # Load the ground plane

        self.objects = []


    def add_object(self,
                   dimension: list,
                   position: list,
                   orientation: list,
                   color: list) -> None:
        """
        This function adds a box object to the environment.
        :param dimension: [box_length, box_width, box_depth]
        :param position: [x, y, z]
        :param orientation: [roll, pitch, yaw]
        :param color: [r, g, b]  range-> 0-1
        :return Does not return anything. Add object in place
        """
        [box_length, box_width, box_depth] = dimension
        object_center = position
        object_orientation = orientation
        object_color = [*color,1]

        geom_box = p.createCollisionShape(p.GEOM_BOX,
                                         halfExtents=[box_length/2,
                                                      box_width/2,
                                                      box_depth/2])

        visual_box = p.createVisualShape(p.GEOM_BOX,
                                        halfExtents=[box_length/2,
                                                     box_width/2,
                                                     box_depth/2],
                                        rgbaColor=object_color)
        box_id = p.createMultiBody(
            baseMass=0,
            baseCollisionShapeIndex=geom_box,
            baseVisualShapeIndex=visual_box,
            basePosition=np.array(object_center),
            baseOrientation=p.getQuaternionFromEuler(object_orientation)
        )

        self.objects.append(box_id)

    def remove_object(self, object_id: int) -> None:
        """
        This function is used to remove the objects from the environment
        :param object_id: Id of the object to return
        :return Returns nothing. Remove objects in place
        """
        p.removeBody(object_id)
        self.objects.remove(object_id)

    def move_object(self, object_id:int, position:list, orientation:list) -> None:
        """
        This function moves the object to the given position and orientation.
        :param object_id: ID of the object to move
        :param position: New 3D position of the object
        :param orientation: New Orientation of the object in quarternions
        :return Returns nothing. Moves object in place
        """
        p.resetBasePositionAndOrientation(object_id, position, orientation)


if __name__ == "__main__":
    env = Environment()
    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.1], [0, 0, 0], [1, 0, 0])
    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.2], [0, 0, 0], [0, 1, 0])
    env.add_object([0.1, 0.1, 0.1], [0, 0, 0.3], [0, 0, 0], [0, 0, 1])

    while not KeyboardInterrupt:
        p.stepSimulation()
        time.sleep(1/240)

    p.disconnect()
