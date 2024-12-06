"""
This file contains the camera class which 
contains the functions related to computer vision and perception
"""
import numpy as np
import pybullet as p


class CameraModule:
    """
    Camera class with the vision functions
    """
    def __init__(self):
        self.set_camera_settings()

    def set_camera_settings(self,
                            camera_width: int = 512,
                            camera_height: int = 512,
                            camera_fov: int = 120,
                            camera_near: float = 0.02,
                            camera_far: float = 100) -> None:
        """
        This function sets the camera settings.
        :param camera_width: image width
        :param camera_height: image height
        :param camera_fov: field of view of camera
        :param camera_near: near clipping plane in meters, do not set non-zero
        :param camera_far: far clipping plane in meters
        :return None
        """
        self.camera_width = camera_width
        self.camera_height = camera_height
        self.camera_fov = camera_fov
        self.camera_near = camera_near
        self.camera_far = camera_far
        # focal depth in pixel space
        self.camera_focal_depth = 0.5*self.camera_height/np.tan(0.5*np.pi/180*self.camera_fov)
        #aspect ratio
        self.camera_aspect = self.camera_width/self.camera_height

    def get_camera_img_float(self,
                             camera_pos: np.ndarray,
                             camera_orn: np.ndarray) -> tuple:
        ''' 
        Gets the image and depth map 
        from a camera at a position cameraPos (3) and cameraOrn (3x3) in space.
        
        :param cameraPos: Position of camera
        :param cameraOrn: Orientation of Camera
        :return RGB image and Depth Image
        '''
        __camera_view_matrix_opengl = p.computeViewMatrix(
            cameraEyePosition=camera_pos,
            cameraTargetPosition=camera_pos+camera_orn[:,2],
            cameraUpVector=-camera_orn[:,1]
            )

        __camera_projection_matrix_opengl = p.computeProjectionMatrixFOV(self.camera_fov,
                                                                         self.camera_aspect,
                                                                         self.camera_near,
                                                                         self.camera_far)

        _, _, rgb_img, non_lin_depth_img, _ = p.getCameraImage(
            self.camera_width,
            self.camera_height,
            __camera_view_matrix_opengl,
            __camera_projection_matrix_opengl,
            renderer=p.ER_BULLET_HARDWARE_OPENGL
            )

        # adjust for clipping and nonlinear distance i.e.,
        # 1/d (0 is closest, i.e., near, 1 is furthest away, i.e., far

        denominator_term = self.camera_far + \
                           self.camera_near - \
                           (self.camera_far-self.camera_near)*non_lin_depth_img

        depth_img_linearized = self.camera_far*self.camera_near/denominator_term

        rgb_image = np.array(rgb_img[:,:,:3], dtype=np.uint8)
        depth_image = np.array(depth_img_linearized, dtype=np.float32)

        return rgb_image, depth_image
