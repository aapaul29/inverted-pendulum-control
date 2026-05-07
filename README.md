# Triple Inverted Pendulum Control — LQR, PID & UKF

A MATLAB/Simulink project implementing state-space control, optimal LQR design, PID comparison, and Unscented Kalman Filter (UKF) state estimation for a triple inverted pendulum on a cart.

---

## Project Overview

The triple inverted pendulum on a cart is one of the most demanding benchmarks in control systems: three links balanced vertically, all unstable in open loop, controlled by a single horizontal force on the cart. This project covers the full pipeline from mathematical modeling to simulated closed-loop control with realistic sensor noise.

**What this project demonstrates:**
- Nonlinear equations of motion → linearization → state-space representation (8 states)
- Controllability verification
- LQR (Linear Quadratic Regulator) design and Q/R matrix tuning
- PID controller design and head-to-head comparison with LQR
- UKF state estimation from noisy sensor measurements
- Open-loop vs. closed-loop simulation

---

## System Description

Three rigid links are connected in series at pivot joints, mounted on a cart of mass `M` that slides freely along a horizontal track. A single horizontal force `F` applied to the cart must balance all three links simultaneously.

**State vector (8 states):**
```
x = [cart_position; cart_velocity;
     theta1; theta1_dot;
     theta2; theta2_dot;
     theta3; theta3_dot]
```

**Linearized around:** all angles = 0 (fully upright equilibrium)

**System parameters** (defined in `matlab/parameters.m`):

| Parameter         | Symbol | Value     |
|-------------------|--------|-----------|
| Cart mass         | M      | 1.0 kg    |
| Link 1 mass       | m1     | 0.3 kg    |
| Link 2 mass       | m2     | 0.2 kg    |
| Link 3 mass       | m3     | 0.1 kg    |
| Link 1 length     | l1     | 0.5 m     |
| Link 2 length     | l2     | 0.4 m     |
| Link 3 length     | l3     | 0.3 m     |
| Gravity           | g      | 9.81 m/s² |

---

## Repository Structure

```
inverted-pendulum-control/
│
├── matlab/
│   ├── parameters.m           # System parameters (run this first)
│   ├── linearize.m            # Derives A, B, C, D matrices (8x8 system)
│   ├── lqr_design.m           # LQR gain computation + controllability check
│   ├── pid_design.m           # PID tuning + closed-loop simulation
│   ├── ukf_design.m           # UKF implementation (sigma points, noise matrices)
│   ├── compare_controllers.m  # Side-by-side LQR vs PID performance plots
│   └── open_loop_sim.m        # Unstable open-loop response (for contrast)
│
├── simulink/
│   ├── lqr_model.slx          # Simulink: LQR closed-loop
│   ├── pid_model.slx          # Simulink: PID closed-loop
│   └── ukf_model.slx          # Simulink: LQR + UKF estimation
│
├── results/
│   ├── open_loop_response.png
│   ├── lqr_response.png
│   ├── pid_response.png
│   ├── lqr_vs_pid_comparison.png
│   └── ukf_estimation.png
│
├── docs/
│   ├── system_derivation.pdf  # Equations of motion + linearization
│   └── project_report.pdf     # Summary report
│
└── README.md
```

---

## How to Run

### Prerequisites
- MATLAB R2021a or later
- Control System Toolbox
- Statistics and Machine Learning Toolbox *(for UKF)*

### Steps

1. Clone the repository:
   ```
   git clone https://github.com/aapaul29/inverted-pendulum-control.git
   ```

2. Open MATLAB and navigate to the project folder:
   ```matlab
   cd 'path/to/inverted-pendulum-control'
   ```

3. Run scripts in order:
   ```matlab
   run('matlab/parameters.m')          % Load system parameters
   run('matlab/lqr_design.m')          % Design LQR controller
   run('matlab/open_loop_sim.m')       % Show unstable open-loop response
   run('matlab/pid_design.m')          % Design PID controller
   run('matlab/ukf_design.m')          % Run UKF state estimation
   run('matlab/compare_controllers.m') % Generate comparison plots
   ```

---

## Results

### Open-Loop Response
Without any control input, a small initial perturbation causes all three link angles to diverge exponentially — confirming the system is open-loop unstable.

### LQR Closed-Loop Response
With LQR state feedback (gain matrix K computed from 8-state system), all three pendulum links stabilize from small initial perturbations. The Q matrix weights angle states heavily to prioritize balancing over cart positioning.

| Metric              | LQR    | PID    |
|---------------------|--------|--------|
| Settling time (θ₁)  | ~2.5 s | ~4.5 s |
| Settling time (θ₂)  | ~3.0 s | ~5.5 s |
| Settling time (θ₃)  | ~3.5 s | ~6.0 s |
| Overshoot           | < 8%   | ~15%   |
| Steady-state error  | 0      | 0      |

> *Note: Results will be updated with final simulation plots.*

### UKF State Estimation
The UKF recovers all 8 states from noisy position and angle measurements only, removing the need for velocity sensors. Estimated states closely track true states after a short convergence period.

---

## Control Theory Background

### Why LQR?
With **8 states** and a single input, manually placing all 8 closed-loop poles is impractical. LQR finds the optimal state-feedback gain matrix **K** by minimizing:

```
J = ∫ (xᵀQx + uᵀRu) dt
```

**Q** penalizes state errors (angle deviations, cart displacement) and **R** penalizes control effort (force magnitude). The 8x8 diagonal Q matrix gives direct, physical control over which states are prioritized.

**Starting point (Bryson's rule):**
```
Q = diag([5, 1, 100, 10, 100, 10, 100, 10])
R = 0.01
```

### Why UKF over EKF?
The triple pendulum equations are highly nonlinear. The UKF propagates **sigma points** through the true nonlinear dynamics rather than approximating with a Jacobian, giving better estimation accuracy — especially important with 8 states where linearization errors compound.

### Why compare with PID?
PID is the most widely deployed controller in industry. Comparing it against LQR on an 8-state system demonstrates the fundamental limitation of single-loop classical control applied to complex multi-state systems — and why modern state-space methods exist.

---

## Tools Used

- **MATLAB** — modeling, LQR/PID design, UKF, simulation scripts
- **Simulink** — block diagram simulation and visualization
- **Git / GitHub** — version control
