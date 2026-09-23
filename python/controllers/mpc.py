import os
import sys
import numpy as np
import cvxpy as cp
from scipy.signal import StateSpace
from scipy.linalg import solve_discrete_are

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from controllers.base import Controller
from utils.dynamics import get_linear_system


class MPCController(Controller):
    """
    Linear Model Predictive Control for the triple inverted pendulum.

    Solves a constrained finite-horizon QP at every timestep:

        min   sum_{k=0}^{N-1} [x_k' Q x_k + R u_k²]  +  x_N' P x_N
        s.t.  x_{k+1} = Ad x_k + Bd u_k          (ZOH discrete dynamics)
              |u_k|    <= F_max                    (force constraint)
              |x_k[0]| <= x_limit                  (cart position constraint)

    P is the DARE solution (LQR cost-to-go), so the unconstrained MPC
    solution exactly matches LQR — constraints only activate when needed.

    Uses CLARABEL (default CVXPY solver) which handles the ill-conditioned
    P matrix reliably. Problem is built once; state is a Parameter so
    only parameter values are updated between solves (warm-started).

    Key advantage over LQR: hard constraints are guaranteed inside the
    optimisation, not just clipped after the fact.
    """

    def __init__(
        self,
        N:       int   = 20,
        dt:      float = 0.02,
        F_max:   float = 50.0,
        x_limit: float = 2.0,
        Q=None,
        R=None,
    ):
        self.N       = N
        self.dt      = dt
        self.F_max   = float(F_max)
        self.x_limit = float(x_limit)

        A, B, _, _ = get_linear_system()
        sys_d    = StateSpace(A, B, np.eye(8), np.zeros((8, 1))).to_discrete(dt, method="zoh")
        self.Ad  = sys_d.A
        self.Bd  = sys_d.B.flatten()

        if Q is None:
            q_x   = 1.0 / 0.5**2
            q_xd  = 1.0 / 2.0**2
            q_th  = 1.0 / np.deg2rad(5)**2
            q_thd = 1.0 / 0.5**2
            Q = np.diag([q_x, q_xd, q_th, q_thd, q_th, q_thd, q_th, q_thd])
        if R is None:
            R = 1.0 / 50.0**2

        self.Q = np.asarray(Q, dtype=float)
        self.R = float(R)

        # Terminal cost: DARE solution — same cost-to-go as LQR
        self.P = solve_discrete_are(
            self.Ad, self.Bd.reshape(-1, 1),
            self.Q, np.array([[self.R]]),
        )

        # Scale cost matrices so CLARABEL sees a well-conditioned problem.
        # Divide by the Frobenius norm of Q — does not change the optimal
        # solution, only the numerical conditioning.
        self._scale = float(np.linalg.norm(self.Q, "fro"))
        self._Qs = self.Q / self._scale
        self._Rs = self.R / self._scale
        self._Ps = self.P / self._scale

        self._build_problem()

    # ------------------------------------------------------------------
    # Problem setup
    # ------------------------------------------------------------------

    def _build_problem(self):
        n, N = 8, self.N
        Ad, Bd = self.Ad, self.Bd

        self._X  = cp.Variable((N + 1, n))
        self._U  = cp.Variable(N)
        self._x0 = cp.Parameter(n)

        # Vectorised objective using scaled matrices
        cost = 0.0
        for k in range(N):
            cost += cp.quad_form(self._X[k], self._Qs) + self._Rs * cp.square(self._U[k])
        cost += cp.quad_form(self._X[N], self._Ps)

        cons = [self._X[0] == self._x0]
        for k in range(N):
            cons += [
                self._X[k + 1] == Ad @ self._X[k] + Bd * self._U[k],
                self._U[k]      <=  self.F_max,
                self._U[k]      >= -self.F_max,
                self._X[k + 1, 0] <=  self.x_limit,
                self._X[k + 1, 0] >= -self.x_limit,
            ]

        self._prob = cp.Problem(cp.Minimize(cost), cons)

    # ------------------------------------------------------------------
    # Controller interface
    # ------------------------------------------------------------------

    def compute(self, state: np.ndarray) -> float:
        self._x0.value = np.asarray(state, dtype=float)

        try:
            self._prob.solve(
                solver=cp.CLARABEL,
                verbose=False,
            )
            if (self._prob.status in (cp.OPTIMAL, cp.OPTIMAL_INACCURATE)
                    and self._U.value is not None):
                return float(np.clip(self._U.value[0], -self.F_max, self.F_max))
        except Exception:
            pass

        return 0.0

    def reset(self):
        pass

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------

    def print_report(self):
        n_vars = (self.N + 1) * 8 + self.N
        print(f"\nMPC Controller")
        print(f"  Horizon     : N={self.N} steps  ({self.N * self.dt:.2f} s lookahead)")
        print(f"  Sample time : {self.dt} s")
        print(f"  Force limit : ±{self.F_max} N  (hard constraint)")
        print(f"  Cart limit  : ±{self.x_limit} m  (hard constraint)")
        print(f"  Solver      : CLARABEL  ({n_vars} decision variables)")
        print(f"  Terminal P  : DARE solution  (same as LQR cost-to-go)")
