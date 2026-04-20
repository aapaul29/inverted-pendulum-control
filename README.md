# Triple Inverted Pendulum on a Cart — LQR Control

## System Description

A triple inverted pendulum on a cart is one of the most challenging benchmark control problems. Three rigid links are connected in series and balanced upright above a horizontally moving cart. The system is highly unstable with 4 degrees of freedom and 8 states.

### State Vector
```
x = [x, θ₁, θ₂, θ₃, ẋ, θ̇₁, θ̇₂, θ̇₃]
```
| Symbol | Meaning | Unit |
|--------|---------|------|
| x | cart horizontal position | m |
| θ₁, θ₂, θ₃ | link angles from vertical (upright = 0) | rad |
| ẋ, θ̇₁, θ̇₂, θ̇₃ | corresponding velocities | m/s, rad/s |

### Mechanical Schematic
```
          o  ← link 3 tip
          |  (m₃, l₃)
          o  ← joint 2
          |  (m₂, l₂)
          o  ← joint 1
          |  (m₁, l₁)
    [===cart===]  →  u (force)
    ___________
```

---

## Files

| File | Description |
|------|-------------|
| `parameters.m` | All physical constants (masses, lengths, inertias, gravity) |
| `lqr_design.m` | Linearised state-space model, controllability check, LQR gain |
| `open_loop_sim.m` | Open-loop simulation — shows exponential divergence |
| `closed_loop_sim.m` | Closed-loop LQR simulation + disturbance rejection test |
| `simulink/pendulum_model.slx` | Simulink model (nonlinear) |
| `docs/system_derivation.pdf` | Full Lagrangian derivation |

---

## How to Run

1. Open MATLAB and set the working directory to the repo root.
2. Make sure the `plots/` folder exists (or create it: `mkdir plots`).
3. Run in this order:

```matlab
parameters        % load system constants
lqr_design        % build model and compute LQR gain K
open_loop_sim     % visualise unstable open-loop behaviour
closed_loop_sim   % visualise stabilised closed-loop + disturbance
```

Plots are saved automatically to `plots/`.

---

## Linearised Model

Around the upright equilibrium the equations of motion become:

```
M(q) q̈ = Bᵤ·u  −  C·q̇  −  K·q
```

where **M** is the 4×4 inertia matrix (cart + 3 links), **K** is a diagonal
"negative-stiffness" matrix (gravity destabilises the upright position), and
**C** is the damping matrix (cart friction only).

Rewritten as an 8-state linear system:
```
ẋ = A·x + B·u,   A ∈ ℝ⁸ˣ⁸,  B ∈ ℝ⁸
```

---

## Controller Design — LQR

Minimise the infinite-horizon cost:
```
J = ∫ (xᵀQx + uᵀRu) dt
```

Tuned weights:
```
Q = diag([10, 100, 100, 100, 1, 10, 10, 10])
R = 0.01
```
Higher penalty on pendulum angles θ₁–θ₃ than on cart position.
Low R allows aggressive actuator force.

---

## Results

### Open-Loop: Instability
Even a 1–2° perturbation causes all links to fall within ~2 s.
The cart position diverges simultaneously.

### Closed-Loop: Stabilisation
From initial angles of ~3–5° the LQR controller returns all states
to the upright equilibrium. Typical settling time: 3–5 s.

### Disturbance Rejection
A velocity impulse applied to the cart at t = 5 s is rejected;
all states recover to equilibrium within ~2 s.

---

## Physical Parameters

| Parameter | Symbol | Value |
|-----------|--------|-------|
| Cart mass | M | 1.0 kg |
| Link 1 mass / length | m₁, l₁ | 0.5 kg, 0.6 m |
| Link 2 mass / length | m₂, l₂ | 0.4 kg, 0.5 m |
| Link 3 mass / length | m₃, l₃ | 0.3 kg, 0.4 m |
| Cart friction | b | 0.1 N·s/m |
| Gravity | g | 9.81 m/s² |

Each link is modelled as a uniform rod: CoM at l/2, I = m·l²/3.
