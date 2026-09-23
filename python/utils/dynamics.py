import os
import sys
import numpy as np
from numpy.linalg import solve

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils import parameters as p


# ---------------------------------------------------------------------------
# Linearized system  (exact port of linearize.m)
# ---------------------------------------------------------------------------

def get_linear_system():
    """
    Linearized (A, B, C, D) around the fully upright equilibrium.

    State  (8,): [x, xd, θ1, θ1d, θ2, θ2d, θ3, θ3d]
    Input  (1,): F — horizontal cart force [N]

    θ1 is absolute; θ2 relative to link 1; θ3 relative to link 2.
    Matches linearize.m output exactly.
    """
    # First moments of mass (coupling terms)
    a1 = p.m1*p.lc1 + p.m2*(p.l1 + p.lc2) + p.m3*(p.l1 + p.l2 + p.lc3)
    a2 = p.m2*p.lc2 + p.m3*(p.l2 + p.lc3)
    a3 = p.m3*p.lc3

    # Linearised mass matrix entries (evaluated at equilibrium: cos→1)
    M11 = p.M + p.m1 + p.m2 + p.m3
    M22 = (p.I1 + p.m1*p.lc1**2
         + p.I2 + p.m2*(p.l1 + p.lc2)**2
         + p.I3 + p.m3*(p.l1 + p.l2 + p.lc3)**2)
    M23 = (p.I2 + p.m2*(p.l1 + p.lc2)*p.lc2
         + p.I3 + p.m3*(p.l1 + p.l2 + p.lc3)*(p.l2 + p.lc3))
    M24 =  p.I3 + p.m3*(p.l1 + p.l2 + p.lc3)*p.lc3
    M33 = (p.I2 + p.m2*p.lc2**2
         + p.I3 + p.m3*(p.l2 + p.lc3)**2)
    M34 =  p.I3 + p.m3*(p.l2 + p.lc3)*p.lc3
    M44 =  p.I3 + p.m3*p.lc3**2

    M_mat = np.array([
        [M11,  a1,  a2,  a3 ],
        [ a1, M22, M23, M24 ],
        [ a2, M23, M33, M34 ],
        [ a3, M24, M34, M44 ],
    ])

    # Gravity stiffness (linearised -dV/dq at equilibrium)
    g1 = p.g * (p.m1*p.lc1 + p.m2*(p.l1 + p.lc2) + p.m3*(p.l1 + p.l2 + p.lc3))
    g2 = p.g * (p.m2*p.lc2 + p.m3*(p.l2 + p.lc3))
    g3 = p.g *  p.m3*p.lc3

    K_grav = np.array([
        [0,  0,  0,  0 ],
        [0, g1, g2, g3 ],
        [0, g2, g2, g3 ],
        [0, g3, g3, g3 ],
    ])

    # Rayleigh damping (cart friction + joint dampings)
    C_damp = np.diag([p.b, p.b1, p.b2, p.b3])

    # Generalised input (force on cart)
    B_input = np.array([[1.0], [0.0], [0.0], [0.0]])

    # Block-form state matrices
    MK = solve(M_mat, K_grav)
    MC = solve(M_mat, C_damp)
    MB = solve(M_mat, B_input)

    A_block = np.block([
        [np.zeros((4, 4)),  np.eye(4)],
        [MK,               -MC       ],
    ])
    B_block = np.vstack([np.zeros((4, 1)), MB])

    # Permute from [x,θ1,θ2,θ3, xd,θ1d,θ2d,θ3d]
    #           to [x,xd, θ1,θ1d, θ2,θ2d, θ3,θ3d]
    perm = [0, 4, 1, 5, 2, 6, 3, 7]
    A = A_block[np.ix_(perm, perm)]
    B = B_block[perm]
    C = np.eye(8)
    D = np.zeros((8, 1))

    return A, B, C, D


def print_stability_report(A):
    """Print eigenvalues and stability status (mirrors linearize.m console output)."""
    ev = np.linalg.eigvals(A)
    n_unstable = int(np.sum(ev.real > 1e-9))
    print("\nOpen-loop eigenvalues:")
    for e in sorted(ev, key=lambda z: z.real):
        sign = '+' if e.imag >= 0 else '-'
        print(f"  {e.real:+.6f} {sign} {abs(e.imag):.6f}j")
    if n_unstable:
        print(f"{n_unstable} unstable mode(s) — open-loop system is UNSTABLE.")
    else:
        print("All eigenvalues stable — open-loop system is stable.")


# ---------------------------------------------------------------------------
# Nonlinear dynamics
# ---------------------------------------------------------------------------

def _mass_matrix_nl(q):
    """
    Full nonlinear 4×4 mass matrix for q = [x, θ1, θ2, θ3].
    Built from translational + rotational Jacobians of each body.
    """
    _, th1, th2, th3 = q
    ph1 = th1
    ph2 = th1 + th2
    ph3 = th1 + th2 + th3

    c1, s1 = np.cos(ph1), np.sin(ph1)
    c2, s2 = np.cos(ph2), np.sin(ph2)
    c3, s3 = np.cos(ph3), np.sin(ph3)

    # Translational Jacobians  shape (2, 4):  rows=[∂x_CoM, ∂y_CoM], cols=[x,θ1,θ2,θ3]
    J1 = np.array([
        [1,  p.lc1*c1,                         0,          0      ],
        [0, -p.lc1*s1,                         0,          0      ],
    ])
    J2 = np.array([
        [1,  p.l1*c1 + p.lc2*c2,   p.lc2*c2,              0      ],
        [0, -p.l1*s1 - p.lc2*s2,  -p.lc2*s2,              0      ],
    ])
    J3 = np.array([
        [1,  p.l1*c1 + p.l2*c2 + p.lc3*c3,  p.l2*c2 + p.lc3*c3,  p.lc3*c3 ],
        [0, -p.l1*s1 - p.l2*s2 - p.lc3*s3, -p.l2*s2 - p.lc3*s3, -p.lc3*s3 ],
    ])

    # Rotational Jacobians  shape (1, 4):  ∂(absolute angular velocity)/∂q_dot
    Jr1 = np.array([[0, 1, 0, 0]])
    Jr2 = np.array([[0, 1, 1, 0]])
    Jr3 = np.array([[0, 1, 1, 1]])

    M_cart       = np.zeros((4, 4)); M_cart[0, 0] = p.M
    return (M_cart
            + p.m1 * J1.T @ J1 + p.I1 * Jr1.T @ Jr1
            + p.m2 * J2.T @ J2 + p.I2 * Jr2.T @ Jr2
            + p.m3 * J3.T @ J3 + p.I3 * Jr3.T @ Jr3)


def _gravity_forces_nl(q):
    """
    Generalised gravity forces τ = -∂V/∂q  (positive → destabilising at upright).
    """
    _, th1, th2, th3 = q
    s1 = np.sin(th1)
    s2 = np.sin(th1 + th2)
    s3 = np.sin(th1 + th2 + th3)

    return np.array([
        0.0,
        p.g * (p.m1*p.lc1*s1
               + p.m2*(p.l1*s1 + p.lc2*s2)
               + p.m3*(p.l1*s1 + p.l2*s2 + p.lc3*s3)),
        p.g * (p.m2*p.lc2*s2 + p.m3*(p.l2*s2 + p.lc3*s3)),
        p.g *  p.m3*p.lc3*s3,
    ])


def _coriolis_centrifugal(q, qdot):
    """
    Coriolis + centrifugal forces via Christoffel symbols, with M(q) differentiated
    numerically (central differences).  Returns h(q,qdot) where  M*qddot + h = rhs.
    """
    eps = 1e-6
    n   = 4
    dMdq = np.zeros((n, n, n))
    for k in range(n):
        eq = np.zeros(n); eq[k] = eps
        dMdq[k] = (_mass_matrix_nl(q + eq) - _mass_matrix_nl(q - eq)) / (2.0 * eps)

    h = np.zeros(n)
    for i in range(n):
        for j in range(n):
            for k in range(n):
                gamma = 0.5 * (dMdq[k, i, j] + dMdq[j, i, k] - dMdq[i, j, k])
                h[i] += gamma * qdot[j] * qdot[k]
    return h


def nonlinear_dynamics(state, u):
    """
    Full nonlinear equations of motion.

    Args:
        state : (8,) array  [x, xd, θ1, θ1d, θ2, θ2d, θ3, θ3d]
        u     : scalar — cart force [N]

    Returns:
        state_dot : (8,) array
    """
    x, xd, th1, th1d, th2, th2d, th3, th3d = state
    q    = np.array([x,   th1,  th2,  th3 ])
    qdot = np.array([xd, th1d, th2d, th3d])

    M_nl    = _mass_matrix_nl(q)
    tau_g   = _gravity_forces_nl(q)
    h       = _coriolis_centrifugal(q, qdot)
    tau_d   = np.diag([p.b, p.b1, p.b2, p.b3]) @ qdot
    B       = np.array([1.0, 0.0, 0.0, 0.0])

    qddot = solve(M_nl, tau_g - h - tau_d + B * u)

    return np.array([
        qdot[0], qddot[0],
        qdot[1], qddot[1],
        qdot[2], qddot[2],
        qdot[3], qddot[3],
    ])
