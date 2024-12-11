# Visual Servoing with Model-Predictive Control for Nearest Object Tracking

This project deals with **Nearest-Object Tracking** using **Model-Predictive Control (MPC)** on a **Franka-Emika-Panda 7 DOF robot arm**.

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
