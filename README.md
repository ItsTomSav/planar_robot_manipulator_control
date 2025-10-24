# planar_robot_manipulator_control
Control strategies for a planar robotic manipulator in MATLAB and Simulink.

# Control of a 2DOF Planar Robotic Manipulator

## 🎯 Overview
This project focuses on analyzing and implementing different control strategies for a **2-DOF planar robotic manipulator** using MATLAB and Simulink.
The goal is to design, simulate, and validate **decentralized** and **centralized** control approaches for trajectory tracking in both joint and operational space.

## 🧰 Technologies
- ![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-green?style=flat-square&logo=mathworks)
- ![Simulink](https://img.shields.io/badge/Simulink-Modeling-blue?style=flat-square&logo=simulink)
- ![Simscape](https://img.shields.io/badge/Simscape-Multibody-lightgrey?style=flat-square&logo=simulink)


## ⚙️ Description
- Implemented decentralized and centralized control schemes in joint and operational space.
- Analyzed system dynamics and verified controller performance through simulation.
- Developed comparative plots of trajectory tracking accuracy.


## 🧪 Results
The designed controllers achieved stable trajectory tracking. It should be noted that controlling a manipulator in operational space is generally more complex than controlling it in joint space, since designing the controller in operational space still requires calculating the Jacobian of the manipulator.


## 📁 Repository Structure
- src/ → MATLAB and Simulink source files
- docs/ → report and documentation
- README.md   → Project overview  

## 👤 Authors
Developed by [Tommaso Savino](https://github.com/ItsTomSav) and [Francesco Savino](https://github.com/FrankSav80).
Master’s Degree in Automation Engineering, Politecnico di Bari
