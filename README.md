## Visual Servoing with LQR for Nearest Object Tracking <br>

This project deals with Nearest-Object Tracking using LQR on an Franka-Emika-Panda 7 DOF robot arm.

<p align="center">
  <img src="https://github.com/rnagrik/MPC-VisualTracking/blob/robot_mpc/results/others/MPC2_redefine.gif" alt="Alt Text" width="70%">
  <br>
  <b>Figure</b>: MPC2, optimised for robot to stay upright
</p>

## Steps to Run the Simulation

To get started with the simulation, follow these steps:

1. **Clone the repository and navigate to project directory:**
   ```bash
   git clone https://github.com/rnagrik/MPC-VisualTracking.git
   cd MPC-VisualTracking
   
3. Run the Simulation
   ```bash
   python3 main.py --controller MPC2
Available controllers are `PID`, `MPC1`, `MPC2`
