from camera import CameraModule

class Controller:
    def __init__(self,control_step):

        self.dt = control_step

        self.num_joints = 7
        self.Q = np.eye(self.num_joints)
        self.R = np.eye(self.num_joints)
        self.K = np.zeros((self.num_joints, 3))
        self.u_prev, self.v_prev = None, None

    def calculate_camera_velocity_in_pixel_space(self, u, v)->tuple:
        """
        Calculate the camera velocity using the current and previous pixel coordinates
        :param u: the current x pixel coordinate
        :param v: the current y pixel coordinate
        :return: a tuple of the velocity in the x and y direction
        """

        if self.u_prev is None:
            du, dv = 0, 0
        else:
            du = u - self.u_prev
            dv = v - self.v_prev
            return du, dv
        
        self.u_prev = u
        self.v_prev = v

        return du/self.dt, dv/self.dt
    

    def calculate_required_camera_movement(self,u,v,depth,camera_ori,camera:CameraModule):
        """
        Calculate the control input for the camera
        :param u: the current x pixel coordinate
        :param v: the current y pixel coordinate
        :param depth: the depth of the object
        :param camera_ori: the orientation of the camera
        :return: a tuple of the position and orientation of the camera in world frame
        """

        u = int(u - camera.camera_width/2)
        v = int(-(v - camera.camera_height/2))
        z = depth        
        f = camera.camera_focal_depth
        L = np.array([[ -f/z ,   0  ,  u/z  ,   u*v/f   , -(f+v*v/f),  v ],
                      [   0  , -f/z ,  v/z  , (f+v*v/f) ,   -u*v/f  , -u ]])        
        L_inv = np.linalg.pinv(L)

        delta_pix = self.camera_params['image_center'] - np.array([u,v])
        delta_pos = L_inv @ delta_pix
        delta_X = delta_pos[:3]
        delta_Omega = delta_pos[3:]
        
        return delta_X @ camera_ori , delta_Omega @ camera_ori
        
    def calculate_joint_velocity(self, joint_angles, delta_X, delta_Omega):
        raise NotImplementedError