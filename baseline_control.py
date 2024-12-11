import numpy as np

class PIDControl:
    def __init__(self,k_p_linear=0.007,k_p_omega=0.08):
        self.K_p_linear = k_p_linear
        self.K_p_omega = k_p_omega
        self.ref_state = np.array([[256],[256]])

    def add_camera_information(self, focal_length, imageWidth, imageHeight):
        self.focal_length = focal_length
        self.imgWidth = imageWidth
        self.imgHeight = imageHeight


    def getImageJacobian(self,u_px,v_px,depthImg):
        """
        Get the image jacobian for the camera
        """
        
        u = int(u_px - self.imgWidth/2)
        v = int(-(v_px - self.imgHeight/2)) # Pinhole Camera Y flipped
        z = depthImg[u_px,v_px]
        f = self.focal_length

        L = [[ -f/z ,   0  ,  u/z  ,   u*v/f   , -(f+v*v/f),  v ],
            [   0  , -f/z ,  v/z  , (f+v*v/f) ,   -u*v/f  , -u ]]
        
        image_jacobian = np.array(L)
        
        return image_jacobian
    
    def getControl(self, object_loc, depth, time_step,camera_orientation):
        """
        Get the control for the camera to follow the object using PID control
        """
        image_jacobian = self.getImageJacobian(int(object_loc[0]),int(object_loc[1]),depth)

        L_inv = np.linalg.pinv(image_jacobian)
        delta_pix = (self.ref_state - np.array(object_loc).reshape(2,1))
        print("Delta Pix : ",delta_pix)
        delta_pos = L_inv @ delta_pix

        delta_pos[:3] = camera_orientation @ delta_pos[:3] * self.K_p_linear
        delta_pos[3:] = camera_orientation @ delta_pos[3:] * self.K_p_omega
        print("Delta Pos : ",delta_pos)
        
        return delta_pos/time_step
