# 🤖 Dobot Magician Kinematic Model in MATLAB

## 📌 Overview

This repository contains a high-fidelity, professional MATLAB script for modeling the kinematics of the **Dobot Magician** robotic arm represented in Figura 1. 

<p align="center">
  <img src="imagens/DobotMagician.png" alt="Dobot Magician 3D Model" width="500">
   <br>
  <small><i>Figure 1: <a href="https://www.dobot-robots.com/products/education/magician.html" target="_blank">Dobot Magician</a> manipulator.</i></small>  
</p>

The Dobot Magician is a popular 4-DOF (Degree of Freedom) robotic manipulator widely used in industry and education, featuring a base, shoulder, elbow, and wrist. With a maximum payload of **500 grams** and a reach of **320 mm**, accurate modeling is crucial for simulation, trajectory planning, and control.

> 💡 **Digital Twin Precision:** This project leverages the MATLAB `rigidBodyTree` representation to create an accurate virtual model. It specifically addresses common dimensional discrepancies found in general literature by implementing corrected physical measurements, ensuring the resulting "digital twin" accurately reflects the real-world hardware.

---

## 🚀 Key Features

*   **Accurate Denavit-Hartenberg (DH) Parameters**
    *   Implements refined DH parameters that align perfectly with the physical robot's geometry, providing an absolute requirement for accurate Forward and Inverse Kinematics calculations.
*   **Corrected Link Proportions**
    *   Replaces generic or simplified dimensions with precise physical measurements (e.g., Base $L_1$ = 138 mm, Shoulder $L_2$ = 135 mm, Elbow $L_3$ = 147 mm) to prevent real-world collision errors.
*   **End-Effector Integration**
    *   Dynamically includes the standard 8 mm connection flange and an optional 74 mm tool extension (for pneumatic grippers or suction cups) to accurately track the true Tool Center Point (TCP).
*   **Optimized Data Structures**
    *   Utilizes the `'column'` `DataFormat` in the `rigidBodyTree` object. This is highly recommended by MATLAB for computational efficiency during Jacobian matrix iterations and dynamic Lagrangian simulations.
*   **Parallelogram Compensation**
    *   The physical Dobot utilizes a closed-loop four-bar linkage mechanism to keep the end-effector parallel to the ground surface. This script maps this behavior into a serial open-chain model by mathematically mimicking the mechanical coupling (linking the wrist pitch counter-rotations to the shoulder and elbow states).
*   **Advanced 3D Visualization**
    *   Renders the robot with aesthetic corrections (using supplementary geometric transformations) so that the 3D cylinders and boxes align flawlessly with the theoretical DH mathematical frames.

---

## 📊 Technical Specifications Modeled

| Link / Segment | Dimension | Description |
| :--- | :---: | :--- |
| **L1 (Base Pedestal)** | 138.0 mm | Orthogonal distance from the physical resting plane to the J2 transversal axis. |
| **L2 (Rear Arm)** | 135.0 mm | The pivoting shoulder link length. |
| **L3 (Forearm)** | 147.0 mm | The intermediate elbow link length. |
| **L4 (Wrist Offset)** | 59.7 mm | Linear distance from the final rotational joint to the mounting face. |
| **Flange** | 8.0 mm | Standard metallic connection flange (acts as a fixed Z-offset). |
| **Extension** | 74.0 mm | Physical length of the external tool (gripper/suction cup stem). |

---

## 🛠️ Prerequisites

To run this simulation framework, you will need:
*   **MATLAB** (R2020a or newer recommended)
*   **Robotics System Toolbox**

---

## 💻 How to Use

1. **Clone** this repository to your local machine:
   ```bash
   git clone https://github.com
   ```
2. Open the script in the **MATLAB editor**.
3. **Configure Angles:** Modify the *User Inputs* section to set your desired joint angles (`theta1` to `theta5` in degrees).
4. **Toggle Attachments:** Change the `has_gripper` boolean flag to `true` or `false` to attach or remove the tool extension.
5. **Run:** Execute the script. 

*A 3D simulation figure will open, displaying the adjusted proportions, the active coordinate frames (X, Y, Z vectors), and a live text readout of the TCP's final Cartesian coordinates.*

---

## 🧠 Motivation & Architecture

Using the `setFixedTransform` function with the `'dh'` argument simplifies the assembly of any manipulator in MATLAB. However, relying on generalized dimensions often leads to a severe **"sim-to-real" gap**, resulting in collisions or singularity errors during Inverse Kinematics (IK) solvers. 

This script bridges that gap by providing a mathematically sound and physically verified foundation, paving the way for advanced robotics research, obstacle avoidance algorithms, and ROS integrations.
