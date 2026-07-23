# Two-Wheel Balancing Robot (Webots Simulation)

A Webots simulation project featuring a **Two-Wheel Self-Balancing Robot (SBR)**. This project implements advanced control strategies, including $H_\infty$ control, along with supervisor-based optimization and mathematical modeling.

---
## 📸 Simulation Preview

<p align="center">
  <img src="./simulation_preview.png" alt="Two-Wheel Robot Simulation Preview" width="600"/>
</p>

## 📌 Project Overview

This repository contains the full Webots simulation setup for a self-balancing two-wheeled robot. It includes:
* **Robot Controllers:** Basic balance control and robust $H_\infty$ controller implementation (`controller_hinf.py`).
* **Supervisor & Optimizer:** A Webots supervisor controller used to optimize control parameters automatically (`supervisor_optimizer.py`).
* **Theoretical Documentation:** Detailed PDFs covering the mathematical model and $H_\infty$ control theory.

---

## 📁 Repository Structure

```text
.
├── controllers/
│   ├── robot_controller/
│   │   ├── controller_hinf.py      # H-infinity robust control implementation
│   │   └── robot_controller.py     # Main robot control script
│   └── supervisor_optimizer/
│       ├── prop.py                 # Optimization parameters/properties
│       └── supervisor_optimizer.py # Supervisor script for parameter optimization
├── theorie/
│   ├── hinf_theory.pdf             # H-infinity control theoretical background
│   └── mathematic_model.pdf        # Mathematical derivation & dynamic model
└── worlds/
    └── SBR_world.wbt               # Webots simulation world
🚀 How to RunPrerequisitesWebots Robot Simulator (R2023b or newer recommended)Python 3.x with required libraries installed (numpy, etc.)StepsClone the repository:Bashgit clone https://github.com/mohammedibra225-stack/two_wheels_balecing_robot.git
cd YOUR_REPOSITORY
Open the simulation:Launch Webots.Open the world file: worlds/SBR_world.wbt.Run the simulation:Press the Play button in Webots.To switch controllers or run the supervisor optimization, select the corresponding controller in the robot / supervisor node settings inside Webots.📚 Theory & ModelingFor an in-depth understanding of the physics and control algorithms used in this project, refer to the files in the theorie/ folder:mathematic_model.pdf: Equations of motion and state-space representation.hinf_theory.pdf: Controller formulation, weighting functions, and $H_\infty$ robustness analysis.
