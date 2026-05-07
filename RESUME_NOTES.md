# Resume & Interview Notes

> This file is for personal reference — not part of the project documentation.

---

## Resume Bullet

> Modeled and stabilized a triple inverted pendulum (8-state system) using LQR optimal control in MATLAB/Simulink; implemented an Unscented Kalman Filter for state estimation from noisy measurements and benchmarked performance against PID control.

---

## Key Interview Points

- *"The triple pendulum has 8 states and is open-loop unstable — all three links diverge from any perturbation without control."*
- *"I linearized the nonlinear equations of motion around the fully upright equilibrium."*
- *"Controllability must be verified first — the rank of the 8x8 controllability matrix must equal 8."*
- *"LQR is the right tool here: placing 8 poles manually is impractical; LQR gives a systematic, optimal solution."*
- *"The UKF is chosen over EKF because the nonlinearities are significant — sigma points handle this more accurately."*

---

## Why This Project Stands Out

- 8-state system is significantly more complex than the standard single-pendulum benchmark
- UKF signals awareness that real systems don't give perfect state measurements
- LQR vs PID comparison shows you can evaluate and justify controller choices — not just implement them
- Fully documented with derivations, simulation plots, and Simulink models
