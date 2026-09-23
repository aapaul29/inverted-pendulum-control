import os
import sys
import numpy as np
from scipy.linalg import solve_continuous_are

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.base import Controller
from utils.dynamics import get_linear_system


class LQRController(Controller):
    """
    Infinite-horizon continuous-time LQR for the linearised triple pendulum.

    Solves the algebraic Riccati equation:
        A'P + PA - PBR⁻¹B'P + Q = 0
    and applies  u = -K @ state  where  K = R⁻¹ B' P.

    Default Q / R reproduce the Bryson's-rule weights from lqr_design.m.
    """

    def __init__(self, Q=None, R=None, F_max=50.0):
        self.F_max = F_max
        A, B, _, _ = get_linear_system()
        self.A = A
        self.B = B.flatten()          # (8,)

        if Q is None:
            Q = self._default_Q()
        if R is None:
            R = self._default_R()

        self.Q = np.asarray(Q, dtype=float)
        self.R = float(R)

        self.K, self.P = self._solve_lqr(A, B.flatten(), self.Q, self.R)

    # ------------------------------------------------------------------
    # Default cost weights (Bryson's rule, matching lqr_design.m)
    # ------------------------------------------------------------------

    @staticmethod
    def _default_Q():
        q_x   = 1.0 / 0.5**2
        q_xd  = 1.0 / 2.0**2
        q_th  = 1.0 / np.deg2rad(5)**2   # tight angle penalty
        q_thd = 1.0 / 0.5**2
        return np.diag([q_x, q_xd, q_th, q_thd, q_th, q_thd, q_th, q_thd])

    @staticmethod
    def _default_R():
        return 1.0 / 50.0**2             # max acceptable force ~50 N

    # ------------------------------------------------------------------
    # Riccati solve
    # ------------------------------------------------------------------

    @staticmethod
    def _solve_lqr(A, b, Q, R):
        """
        Solve continuous ARE and return (K, P).
        scipy.linalg.solve_continuous_are expects:
            A'P + PA - P B R⁻¹ B' P + Q = 0
        """
        B2d = b.reshape(-1, 1)
        P   = solve_continuous_are(A, B2d, Q, np.array([[R]]))
        K   = (1.0 / R) * B2d.T @ P    # (1, 8)
        return K.flatten(), P           # K is (8,)

    # ------------------------------------------------------------------
    # Controller interface
    # ------------------------------------------------------------------

    def compute(self, state: np.ndarray) -> float:
        u = -float(self.K @ state)
        return float(np.clip(u, -self.F_max, self.F_max))

    def reset(self):
        pass   # LQR is stateless

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def closed_loop_eigenvalues(self):
        """Eigenvalues of A - b K  (all should be in LHP)."""
        return np.linalg.eigvals(self.A - np.outer(self.B, self.K))

    def print_report(self):
        print("\nLQR gain K:")
        print("  ", np.array2string(self.K, precision=4, suppress_small=True))
        ev = self.closed_loop_eigenvalues()
        print("\nClosed-loop eigenvalues (A - bK):")
        for e in sorted(ev, key=lambda z: z.real):
            sign = '+' if e.imag >= 0 else '-'
            print(f"  {e.real:+.4f} {sign} {abs(e.imag):.4f}j")
        n_unstable = int(np.sum(ev.real > 1e-9))
        if n_unstable == 0:
            print("All closed-loop poles in LHP — controller is stabilising.")
        else:
            print(f"WARNING: {n_unstable} unstable closed-loop pole(s).")
