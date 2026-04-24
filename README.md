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
      <img src="https://github.com/user-attachments/assets/483b18c9-996e-43d7-b308-8bc861646036" width="300" alt="Aircraft body axes illustration"/>
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

To automatically counteract this pitch-down moment, this project implements an **LQI (Linear Quadratic Integral) controller**. 

---

### Technical Approach

### 1. Mathematical Modeling

#### 1.1. The 6-DOF Flight Dynamics Model
To describe how an aircraft moves in 3D space, we use the 6 Degrees of Freedom (6-DOF) equations. We assume the aircraft is symmetrical. 
These equations link the translation speeds $(u, v, w)$ and the rotation speeds $(p, q, r)$:

**Translational Dynamics:**

$$
\begin{cases}
\dot{u} = \dfrac{X}{m} - g \sin\theta - (q w - r v) \\
\dot{v} = \dfrac{Y}{m} + g \cos\theta \sin\phi - (r u - p w) \\
\dot{w} = \dfrac{Z}{m} + g \cos\theta \cos\phi - (p v - q u)
\end{cases}
$$

**Rotational Dynamics:**

$$
\begin{cases}
\dot{p} = \dfrac{L + (I_y - I_z)q r + I_{xz}(\dot{r} + p q)}{I_x} \\
\dot{q} = \dfrac{M + (I_z - I_x)p r + I_{xz}(r^2 - p^2)}{I_y} \\
\dot{r} = \dfrac{N + (I_x - I_y)p q + I_{xz}(\dot{p} - q r)}{I_z}
\end{cases}
$$

**Where:**

* $u, v, w$: Roll, pitch, and yaw body velocities.
* $p, q, r$: Roll, pitch, and yaw angular rates.
* $\phi, \theta$: Roll and pitch angles.
* $X, Y, Z$: Aerodynamic and thrust forces.
* $L, M, N$: Aerodynamic and thrust moments.
* $m$: Aircraft mass.
* $I_x, I_y, I_z, I_{xz}$: Moments of inertia.

---

#### 1.2. Simplification: From 6-DOF to 3-DOF

The 6-DOF model is complete but complex. Since this project focuses only on pitch control during landing, we can simplify it. 

We use these standard assumptions:
* **Symmetric flight** *(No roll, yaw, or side movement)* ($v = p = r = \phi = 0$).
* **Flat Earth** *(A good approximation for a short landing)*.
* **Constant mass** *(We ignore fuel burn during this short phase)*.

Based on these assumptions, we reduce the complex system into a simpler **3-DOF longitudinal model**.

The new model can be written as :

$$
\begin{cases}
\dot{u} = \dfrac{X(u, w, q, \delta_e, T)}{m} - g \sin\theta + q w \\
\dot{w} = \dfrac{Z(u, w, q, \delta_e, T)}{m} + g \cos\theta - q u \\
\dot{q} = \dfrac{M(u, w, q, \delta_e, T)}{I_y} \\
\dot{\theta} = q
\end{cases}
$$

---

#### 2. Linearization and Error Model

To design our controller, we need a linear model. We do this by studying the small variations (perturbations) around our target flight condition.

First, we define our steady equilibrium point (the "trim condition") for the landing approach:

$$
\begin{cases}
U_0 = 70 \text{ m/s} & \text{(Steady approach speed)} \\
\Theta_0 = 3^\circ & \text{(Trim pitch angle)} \\
W_0 = 0 & \text{(Zero vertical speed relative to the steady path)} \\
Q_0 = 0 & \text{(Zero pitch rate, meaning the pitch angle is constant)}
\end{cases}
$$

Next, we define our state variables as the sum of this steady equilibrium value (subscript 0) and a small variation ($\Delta$):

* $u = U_0 + \Delta u$
* $w = W_0 + \Delta w$
* $q = Q_0 + \Delta q$
* $\theta = \Theta_0 + \Delta \theta$

By substituting these into our 3-DOF equations and removing negligible terms, we get the linearized error model:

$$
\begin{cases}
\Delta\dot{u} = \dfrac{\Delta X}{m} - g \cos(\Theta_0) \Delta\theta \\
\Delta\dot{w} = U_0 \Delta q + \dfrac{\Delta Z}{m} - g \sin(\Theta_0) \Delta\theta \\
\Delta\dot{q} = \dfrac{\Delta M}{I_{yy}} \\
\Delta\dot{\theta} = \Delta q
\end{cases}
$$

This linear model is now perfectly suited to be converted into a State-Space representation ($A, B, C, D$ matrices).


---

#### 3. Linear State Space Model

To implement the controller in Python, we rewrite the linear equations into a standard **State Space** form.

The general system is defined by:

$$
\begin{cases}
\dot{\mathbf{x}} = A\mathbf{x} + B\mathbf{u} + E\mathbf{d}_{gear} \\
\mathbf{y} = C\mathbf{x}
\end{cases},
$$

**Where:**
* **State vector** $x = [\Delta u, \Delta w, q, \Delta \theta]^T$.
* **Control input** $u = \delta_e$ *(Elevator deflection)*.
* **Disturbance** $d_{gear}$ *(Represented as a step input representing the landing gear deployment)*.





























---
**2. Developed Matrix Form:**
Based on the aerodynamic derivatives and the linearization, the matrices $A$, $B$, and $E$ are structured as follows:

$$
\begin{bmatrix} 
\Delta\dot{u} \\ 
\Delta\dot{w} \\ 
\dot{q} \\ 
\Delta\dot{\theta} 
\end{bmatrix} = 
\begin{bmatrix} 
X_u/m & X_w/m & X_q/m & -g \cos\Theta_0 \\ 
Z_u/m & Z_w/m & (U_0 + Z_q/m) & -g \sin\Theta_0 \\ 
M_u/I_{yy} & M_w/I_{yy} & M_q/I_{yy} & 0 \\ 
0 & 0 & 1 & 0 
\end{bmatrix}
\begin{bmatrix} 
\Delta u \\ 
\Delta w \\ 
q \\ 
\Delta \theta 
\end{bmatrix} +
\begin{bmatrix} 
X_{\delta_e}/m \\ 
Z_{\delta_e}/m \\ 
M_{\delta_e}/I_{yy} \\ 
0 
\end{bmatrix} \delta_e +
\begin{bmatrix} 
X_{gear}/m \\ 
Z_{gear}/m \\ 
M_{gear}/I_{yy} \\ 
0 
\end{bmatrix} d_{gear}
$$



**3. Output Matrix:**
Since we focus on the pitch angle $\Delta \theta$, the $C$ matrix selects the 4th state:

$$
C = \begin{bmatrix} 0 & 0 & 0 & 1 \end{bmatrix}
$$

---





















#### 1.3. LQI (Linear Quadratic Integral) Formulation
A standard LQR controller provides optimal regulation but cannot guarantee zero steady-state error in the presence of a constant disturbance (like the persistent drag from the landing gear). To solve this, we introduce an **Integral action**.

We augment the state vector with the integral of the tracking error, $e(t) = y(t) - r(t)$ (where $r(t)$ is the target pitch angle):

$$
x_i(t) = \int_{0}^{t} (C x(\tau) - r(\tau)) d\tau
$$

The augmented state vector becomes $x_a = [x^T, x_i]^T$, resulting in the augmented system:

$$
\dot{x}_a = A_a x_a + B_a u + E_a d_{gear}
$$

$$
A_a = \begin{bmatrix} A & 0_{4 \times 1} \\ -C & 0 \end{bmatrix}, \quad B_a = \begin{bmatrix} B \\ 0 \end{bmatrix}
$$

#### 1.4. Optimal Gain Calculation
The optimal control law $u = -K x_a$ is found by minimizing the infinite-horizon quadratic cost function:

$$
J = \int_{0}^{\infty} (x_a^T Q x_a + u^T R u) dt
$$

By carefully tuning the weighting matrices $Q$ (state penalty) and $R$ (control effort penalty), we balance aggressive disturbance rejection against actuator limits. In this project, a massive penalty (7000.0) is applied to the integral state within the $Q$ matrix to force a rapid elimination of the steady-state error caused by the gear deployment, ensuring the aircraft strictly maintains its 3° approach angle.

