# Inverted Pendulum Stabilization — State-Space Control & LQR

A MATLAB/Simulink implementation of a linearized cart-pendulum system stabilized using Linear Quadratic Regulator (LQR) control. This project demonstrates nonlinear modeling, state-space linearization, controllability analysis, and optimal state-feedback control design.

---

## Project Overview

The inverted pendulum on a cart is a classic benchmark in control systems engineering. The system is inherently unstable in open loop — the goal is to design a controller that keeps the pendulum upright while allowing the cart to track a desired position.

This project covers:
- Derivation of the nonlinear equations of motion
- Linearization around the upright equilibrium
- State-space representation and controllability verification
- LQR controller design
- Closed-loop simulation in MATLAB and Simulink
- Comparison of open-loop vs closed-loop behavior
- Disturbance rejection analysis

---

## System Description

The system consists of a cart of mass `M` free to slide along a horizontal track, with a pendulum of mass `m` and length `l` attached at a pivot.

**State vector:**

```
x = [cart position; cart velocity; pendulum angle; angular velocity]
  = [x; x_dot; theta; theta_dot]
```

**Linearized around:** `theta = 0` (upright equilibrium)

**System parameters (defined in `parameters.m`):**

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Cart mass | M | 1.0 kg |
| Pendulum mass | m | 0.2 kg |
| Pendulum length | l | 0.5 m |
| Gravitational acceleration | g | 9.81 m/s² |
| Friction coefficient | b | 0.1 N·s/m |

---

## Repository Structure

```
inverted-pendulum-control/
│
├── parameters.m          # System parameters and state-space matrices
├── lqr_design.m          # LQR gain computation and controllability check
├── open_loop_sim.m       # Open-loop simulation (unstable response)
├── closed_loop_sim.m     # Closed-loop LQR simulation with plots
│
├── simulink/
│   └── pendulum_model.slx    # Simulink block diagram (Weekend 2)
│
├── results/
│   ├── open_loop_response.png
│   ├── closed_loop_response.png
│   └── disturbance_response.png
│
└── README.md
```

---

## How to Run

### Prerequisites
- MATLAB R2021a or later
- Control System Toolbox

### Steps

1. Clone the repository:
   ```
   git clone https://github.com/aapaul29/inverted-pendulum-control.git
   ```

2. Open MATLAB and navigate to the project folder:
   ```matlab
   cd 'C:\Users\aarya\OneDrive\Desktop\inverted-pendulum-control'
   ```

3. Run scripts in order:
   ```matlab
   run('parameters.m')       % Load system parameters
   run('lqr_design.m')       % Design LQR controller
   run('open_loop_sim.m')    % Simulate unstable open-loop
   run('closed_loop_sim.m')  % Simulate stabilized closed-loop
   ```

---

## Results

### Open-Loop Response
Without control, the pendulum angle diverges exponentially from any non-zero initial condition, confirming the system is unstable in open loop.

### Closed-Loop Response (LQR)
With LQR state feedback, the pendulum stabilizes to the upright position from an initial angle of ~5°. The cart position converges to zero with no steady-state error.

**Performance summary:**

| Metric | Value |
|--------|-------|
| Settling time (angle) | ~2.5 s |
| Settling time (cart) | ~3.5 s |
| Overshoot | < 5% |
| Steady-state error | 0 |

### Disturbance Rejection
An impulse disturbance (force applied to cart) is rejected within ~3 seconds, with the pendulum returning to vertical.

---

## Controller Design

### Controllability
The controllability matrix `C = [B, AB, A²B, A³B]` is computed and verified to have full rank (rank = 4), confirming the system is fully controllable.

### LQR Formulation
The LQR minimizes the cost function:

```
J = ∫ (x' Q x + u' R u) dt
```

**Weighting matrices used:**

```matlab
Q = diag([1, 1, 10, 10]);   % Penalize angle deviation most heavily
R = 0.01;                    % Allow aggressive control input
```

The optimal gain matrix `K` is computed via MATLAB's `lqr()` function, and control input is `u = -K * x`.

---

## Key Concepts Demonstrated

- **Linearization** of a nonlinear dynamical system
- **State-space modeling** (A, B, C, D matrices)
- **Controllability analysis** using the controllability matrix
- **LQR design** and cost function interpretation
- **Closed-loop stability** via eigenvalue placement
- **Disturbance rejection** in feedback systems

---

## Skills & Tools

`MATLAB` · `Simulink` · `Control System Toolbox` · `State-Space Control` · `LQR` · `Linearization` · `GitHub`

---

## Resume Bullet

> Modeled and stabilized an inverted pendulum using state-space methods and LQR control in MATLAB/Simulink, achieving closed-loop stability for an inherently unstable system with disturbance rejection within 3 seconds.

---

## References

- Ogata, K. — *Modern Control Engineering*, 5th Ed.
- Franklin, Powell, Emami-Naeini — *Feedback Control of Dynamic Systems*
- MATLAB Documentation: [`lqr`](https://www.mathworks.com/help/control/ref/lqr.html), [`ss`](https://www.mathworks.com/help/control/ref/ss.html)
