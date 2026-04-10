# A330 Pitch angle control during landing gear deployment

![Python](https://img.shields.io/badge/Python-3.11-3776AB.svg?style=flat&logo=python&logoColor=white)
![Control Systems](https://img.shields.io/badge/Control_Systems-LQI_Optimal_Control-8A2BE2.svg)
![Status](https://img.shields.io/badge/Status-Educational_Project-green.svg)

> **Robust A330 pitch angle control using an LQI controller designed to counteract the pitch-down effect of landing gear deployment, featuring noisy sensor simulation. Built on Python.**

---

### Visual Demo: Pitch-Down Effect and LQI Correction

<table width="100%">
  <tr>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/340bd9c0-e7fc-4a86-8b12-bee5ce76eb8a" width="350" alt="A330 Landing Gear Deployment GIF"/>
      <br><br>
      <b>Fig 1.</b> Simulation of the A330 landing phase.
    </td>
    <td width="50%" align="center">
      <img src="https://github.com/user-attachments/assets/483b18c9-996e-43d7-b308-8bc861646036" width="350" alt="Aircraft body axes illustration"/>
      <br><br>
      <b>Fig 2.</b> Aircraft body axes and flight dynamics convention.
    </td>
  </tr>
</table>

**Simulation Stages:**
1. **Landing Approach:** The aircraft approaches the tarmac with a stable initial pitch angle.
2. **Gear Deployment:** Landing gears are extended, creating a sudden aerodynamic pitch-down moment.
3. **Pitch Deviation:** The aircraft experiences an immediate and undesired drop in its pitch attitude.
4. **LQI Correction:** The controller rapidly adjusts the elevator to reject the disturbance and restore the target pitch angle.

---

### Context & Project Overview

During final approach, deploying the landing gear is a routine but critical step. However, dropping the heavy gear into the airflow introduces a sudden aerodynamic challenge: a strong pitch-down effect.

#### The Aerodynamic Problem
When the landing gear is lowered, it creates a massive amount of drag. Because this extra drag happens well below the aircraft's Center of Gravity (CG), it acts like a lever, naturally pulling the nose of the plane down. 

<p align="center">
  <img width="500" alt="Pitch-down effect illustration" src="https://github.com/user-attachments/assets/60b6eab6-1044-4670-9aef-0d8a7731dcce" />
  <br>
  <i>Fig 2. Aerodynamic pitch-down moment.</i>
</p>

#### Why It Matters
At low altitudes and slower approach speeds, an unexpected nose drop is a serious safety risk. If the flight control system doesn't correct it immediately, the aircraft can dip below its target glide path, potentially leading to a hard landing. Fast, precise elevator adjustments are needed to keep the plane steady.

#### The Control Systems Solution
To automatically counteract this pitch-down moment, this project implements an **LQI (Linear Quadratic Integral) controller**. 

* **Disturbance Rejection:** The Integral (I) action completely eliminates the steady-state error caused by the gear deployment, returning the pitch exactly to its 3° target.
* **Optimal Actuation:** The Linear Quadratic (LQ) algorithm quickly and smoothly corrects the pitch angle without over-stressing the elevator actuators.
* **Realistic Simulation:** Gaussian noise is injected into the signals to mimic imperfect, real-world IMU sensor readings.

---

### Technical Approach

### 1. Mathematical Modeling

#### 1.1. Non-Linear Longitudinal Flight Dynamics (3-DOF)
The project begins with the fundamental non-linear equations of motion governing the aircraft's longitudinal dynamics (3 Degrees of Freedom). These equations describe the aircraft's behavior in the symmetry plane (ignoring roll and yaw):

$$
\begin{cases}
\dot{u} = \dfrac{X(u, w, q, \delta_e, T)}{m} - g \sin\theta + q w \\
\dot{w} = \dfrac{Z(u, w, q, \delta_e, T)}{m} + g \cos\theta - q u \\
\dot{q} = \dfrac{M(u, w, q, \delta_e, T)}{I_y} \\
\dot{\theta} = q
\end{cases}
$$

**Where:**
* $u, w$: Longitudinal and vertical velocities.
* $q$: Pitch rate.
* $\theta$: Pitch angle.
* $X, Z, M$: Aerodynamic forces and pitching moment.
* $\delta_e, T$: Elevator deflection (control input) and Engine Thrust.
* $m, I_y$: Aircraft mass and pitch moment of inertia.

#### 1.2. Linearized State-Space Representation

