import os
import sys
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.base import Controller
from controllers.lqr import LQRController


class PIDController(Controller):
    """
    PID controller for the triple inverted pendulum.

    Two operating modes:

    angle_only=True  (default, matches pid_design.m)
        Takes the full LQR gain K and zeros out the cart position / velocity
        terms.  Angles converge but the cart drifts — this is intentional;
        it shows the cost of ignoring the cart DoF vs. full-state LQR.
        u = -K_pd @ state   where K_pd = [0, 0, K[2..7]]

    angle_only=False
        Full-state PD + optional integral action on cart position to
        correct the steady-state drift that the angle-only mode exhibits.
        u = -K_pd @ state - ki_x * integral(x dt)

    Gains can be supplied manually or derived from an LQRController instance
    via the class method  PIDController.from_lqr().
    """

    def __init__(
        self,
        K_pd: np.ndarray,
        angle_only: bool = True,
        ki_x: float = 0.0,
        dt: float = 0.02,
        F_max: float = 50.0,
    ):
        """
        Args:
            K_pd      : (8,) gain vector applied as u = -K_pd @ state.
                        If angle_only=True the first two entries are forced to 0.
            angle_only: zero cart (x, xd) gains — mirrors pid_design.m behaviour.
            ki_x      : integral gain on cart position error.  Use a small value
                        (e.g. 5–20) to correct the steady-state cart drift when
                        angle_only=True.  Set to 0 for the pure MATLAB equivalent.
            dt        : time step used to accumulate the cart-position integral [s].
            F_max     : saturation limit on output force [N].
        """
        K_pd = np.asarray(K_pd, dtype=float).copy()
        if angle_only:
            K_pd[0] = 0.0   # zero x   feedback
            K_pd[1] = 0.0   # zero xd  feedback

        self.K_pd       = K_pd
        self.angle_only = angle_only
        self.ki_x       = float(ki_x)
        self.dt         = float(dt)
        self.F_max      = float(F_max)

        self._integral_x = 0.0   # accumulated cart-position error

    # ------------------------------------------------------------------
    # Factory
    # ------------------------------------------------------------------

    @classmethod
    def from_lqr(
        cls,
        lqr: LQRController = None,
        angle_only: bool = True,
        ki_x: float = 0.0,
        dt: float = 0.02,
        F_max: float = 50.0,
    ):
        """
        Derive PID gains from a solved LQRController (or create one automatically).
        This is the direct Python equivalent of pid_design.m.
        """
        if lqr is None:
            lqr = LQRController(F_max=F_max)
        return cls(lqr.K.copy(), angle_only=angle_only, ki_x=ki_x, dt=dt, F_max=F_max)

    # ------------------------------------------------------------------
    # Controller interface
    # ------------------------------------------------------------------

    def compute(self, state: np.ndarray) -> float:
        state = np.asarray(state, dtype=float)
        x     = state[0]

        u = -float(self.K_pd @ state)

        # Optional integral correction on cart position
        if self.ki_x != 0.0:
            self._integral_x += x * self.dt
            u -= self.ki_x * self._integral_x

        return float(np.clip(u, -self.F_max, self.F_max))

    def reset(self):
        self._integral_x = 0.0

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def closed_loop_eigenvalues(self, A, B):
        """
        Eigenvalues of A - B @ K_pd.
        Pass the linearised A (8×8) and B (8,) from dynamics.get_linear_system().
        """
        b = np.asarray(B).flatten()
        return np.linalg.eigvals(A - np.outer(b, self.K_pd))

    def print_report(self, A=None, B=None):
        print("\nPD (angle-only) gain K_pd:")
        labels = ["x", "xd", "th1", "th1d", "th2", "th2d", "th3", "th3d"]
        for label, k in zip(labels, self.K_pd):
            print(f"  K_{label:<4s} = {k:+.4f}")
        if self.ki_x:
            print(f"\n  ki_x (cart integral) = {self.ki_x}")
        if A is not None and B is not None:
            ev = self.closed_loop_eigenvalues(A, B)
            print("\nClosed-loop eigenvalues (A - b K_pd):")
            for e in sorted(ev, key=lambda z: z.real):
                sign = '+' if e.imag >= 0 else '-'
                print(f"  {e.real:+.4f} {sign} {abs(e.imag):.4f}j")
            n_unstable = int(np.sum(ev.real > 1e-9))
            if n_unstable == 0:
                print("All closed-loop poles stable.")
            else:
                print(f"WARNING: {n_unstable} UNSTABLE pole(s) — gains need tuning.")
