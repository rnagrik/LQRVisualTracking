import numpy as np
from matplotlib import pyplot as plt
import os


class Recorder:
    """
    This class is used to record data and evaluate the controller performance
    """
    def __init__(self,controller_type=None):
        self.tracking_data = None
        self.control_data = None
        self.camera_size = 512
        self.controller_type = controller_type
        if controller_type in ['MPC1','MPC2','PID']:
            self.data_dir = f'data/{controller_type}'
        else:
            print("Invalid Controller Type")

        self.iter = 1
        while os.path.exists(f'{self.data_dir}/tracking_data_{self.iter}.npy') or os.path.exists(f'{self.data_dir}/control_data_{self.iter}.npy'):
            print(f"Run {self.iter} already exists")
            self.iter += 1

    def record_pixel(self, pixels):
        pixels = np.array(pixels).reshape((1,2))
        if self.tracking_data is None:
            self.tracking_data = pixels
        else:
            self.tracking_data = np.vstack((self.tracking_data, pixels))

    def record_control(self, control):
        if self.controller_type in ['MPC1','PID']:
            control = np.array(control).reshape((1,6))
        elif self.controller_type == 'MPC2':
            control = np.array(control).reshape((1,7))
        if self.control_data is None:
            self.control_data = control
        else:
            self.control_data = np.vstack((self.control_data, control))

    def evaluate_controller(self,show_plot=True,print_error=True):
        """
        Evaluate the controller performance and plot the tracking error over time
        """
        if self.tracking_data is None:
            print("No Tracking Data Found")
            return

        tracking_error = self.tracking_data - self.camera_size/2
        tracking_error = np.linalg.norm(tracking_error, axis=1)
        mean_tracking_error = np.mean(tracking_error)
        std_dev = np.std(tracking_error)
        if print_error:
            print(f"Mean Tracking Error : {mean_tracking_error}")
            print(f"Standard Deviation : {std_dev}")
        if show_plot:
            plt.plot(tracking_error,label='Tracking Error ')
            plt.xlabel('Time')
            plt.ylabel('Tracking Error')
            plt.legend()
            plt.show()

        return mean_tracking_error

    def save_data(self):
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        if self.tracking_data is not None:
            np.save(f'{self.data_dir}/tracking_data_{self.iter}.npy',self.tracking_data)
        if self.control_data is not None:
            np.save(f'{self.data_dir}/control_data_{self.iter}.npy',self.control_data)

    def load_data(self,run=-1,tracking_data=True,control_data=True): 
        assert type(run) == int, "Run should be a positive integer"
        if run == -1:
            run = self.iter-1
        else:
            self.iter = run+1
        run = str(run)
        if tracking_data:
            self.tracking_data = np.load(f'{self.data_dir}/tracking_data_{run}.npy')
            print(f"Loaded Tracking Data {self.data_dir}/tracking_data_{run}.npy")
        if control_data:
            self.control_data = np.load(f'{self.data_dir}/control_data_{run}.npy')
            print(f"Loaded Control Data {self.data_dir}/control_data_{run}.npy")