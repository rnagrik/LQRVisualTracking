# Visual Servoing with Model-Predictive Control for Nearest Object Tracking

This project deals with **Nearest-Object Tracking** using **Model-Predictive Control (MPC)** on a **Franka-Emika-Panda 7 DOF robot arm**.<br>
The problem was solved in 2 stages - (a) Identifying the nearest object to track (b) Designing a controller to track the nearest object where "tracking" is defined as maintaining the object at the center of the image. 

<p align="center">
  <img src="https://github.com/rnagrik/MPC-VisualTracking/blob/master/Results/MPC2.gif" alt="Alt Text" width="50%">
</p>

## Identifying the Nearest Object <br>
We use the camera-calibration matrix and the camera position, orientation and the object location in the world to calculate the pixel-coordinates by taking projections in 2D. 

## MPC Controller
The state of the object is defined as the pixel co-ordinates of the object in the camera frame (2 dimensional). The control space is the joint angle velocity of the manipulator (7-dimensional). The motion model basically relates the joint velocities (U) given to the bot at the current time step to the pixel co-ordinates that we "predict" will observe for the object in the next time-step. We use the D-H parameters of the bot to identify the configuration of the robot (and hence the position and orientation of the camera) at the next time-step. Based on the new position and the object location in the world, we take the projection (similar to what's done in the above part) to calculate the pixel co-ordinates of the object at the next time-step. <br>
The cost function for optimization is Mean-Square error between the pixel co-ordinates + penalty on the control + penalty for moving at extreme angles and we use the prediction horizon as N=10.

## Steps to Run the Simulation

To get started with the simulation, follow these steps:

1. **Install PyBullet and CasADi**<br>
   `pip install casadi`<br>
   `pip install pybullet` <br>
3. **Clone the repository and navigate to project directory:**
   ```bash
   git clone https://github.com/rnagrik/MPC-VisualTracking.git
   cd MPC-VisualTracking
   
4. Run the Simulation
   ```bash
   python3 main.py --controller MPC2
Available controllers are `PID`, `MPC1`, `MPC2`
