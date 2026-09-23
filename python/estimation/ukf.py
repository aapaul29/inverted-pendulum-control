import os
import sys
import numpy as np
from scipy.linalg import cholesky
from scipy.signal import StateSpace

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.dynamics import get_linear_system


class UKF:
    """
    Generic Unscented Kalman Filter.

    Mirrors the sigma-point algorithm in ukf_design.m exactly:
      - Merwe-scaled sigma points  (alpha, beta, kappa)
      - Separate predict / update steps fused into step()
      - Supports any process_fn and measure_fn (linear or nonlinear)

    Usage:
        ukf = UKF(n_x, n_y, process_fn, measure_fn, Q, R)
        ukf.reset(x0, P0)
        for u, y in data:
            x_hat = ukf.step(u, y)
    """

    def __init__(
        self,
        n_x: int,
        n_y: int,
        process_fn,
        measure_fn,
        Q: np.ndarray,
        R: np.ndarray,
        alpha: float = 1e-3,
        beta:  float = 2.0,
        kappa: float = 0.0,
    ):
        self.n_x        = n_x
        self.n_y        = n_y
        self.process_fn = process_fn
        self.measure_fn = measure_fn
        self.Q          = np.asarray(Q, dtype=float)
        self.R          = np.asarray(R, dtype=float)

        # Sigma-point scaling  (matches ukf_design.m lines 41-53)
        lam        = alpha**2 * (n_x + kappa) - n_x
        self.gamma = np.sqrt(n_x + lam)

        n_sig      = 2 * n_x + 1
        self.Wm    = np.full(n_sig, 1.0 / (2.0 * (n_x + lam)))
        self.Wc    = np.full(n_sig, 1.0 / (2.0 * (n_x + lam)))
        self.Wm[0] = lam / (n_x + lam)
        self.Wc[0] = lam / (n_x + lam) + (1.0 - alpha**2 + beta)

        self.x_hat: np.ndarray | None = None
        self.P:     np.ndarray | None = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def reset(self, x0: np.ndarray, P0: np.ndarray):
        """Set initial estimate and covariance."""
        self.x_hat = np.asarray(x0, dtype=float).copy()
        self.P     = np.asarray(P0, dtype=float).copy()

    def step(self, u, y: np.ndarray) -> np.ndarray:
        """
        One predict + update cycle.

        Args:
            u : control input forwarded to process_fn
            y : (n_y,) noisy measurement vector

        Returns:
            x_hat : (n_x,) updated state estimate
        """
        y = np.asarray(y, dtype=float)

        # ---- Predict ---------------------------------------------------
        Xsig      = self._sigma_points()
        Xsig_pred = np.column_stack(
            [self.process_fn(Xsig[:, i], u) for i in range(2 * self.n_x + 1)]
        )

        x_pred = Xsig_pred @ self.Wm
        P_pred = self.Q.copy()
        for j in range(2 * self.n_x + 1):
            d       = Xsig_pred[:, j] - x_pred
            P_pred += self.Wc[j] * np.outer(d, d)

        # ---- Update ----------------------------------------------------
        Ysig   = np.column_stack(
            [self.measure_fn(Xsig_pred[:, i]) for i in range(2 * self.n_x + 1)]
        )

        y_pred = Ysig @ self.Wm
        S      = self.R.copy()
        T      = np.zeros((self.n_x, self.n_y))
        for j in range(2 * self.n_x + 1):
            dy  = Ysig[:, j]      - y_pred
            dx  = Xsig_pred[:, j] - x_pred
            S  += self.Wc[j] * np.outer(dy, dy)
            T  += self.Wc[j] * np.outer(dx, dy)

        K_gain    = T @ np.linalg.inv(S)
        self.x_hat = x_pred + K_gain @ (y - y_pred)
        self.P     = P_pred - K_gain @ S @ K_gain.T

        return self.x_hat.copy()

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _sigma_points(self) -> np.ndarray:
        """
        Generate (n_x, 2*n_x+1) sigma-point matrix.
        Columns: [x_hat,  x_hat ± gamma*chol(P)]
        """
        sqrt_P = self.gamma * cholesky(self.P, lower=True)
        Xsig   = np.empty((self.n_x, 2 * self.n_x + 1))
        Xsig[:, 0] = self.x_hat
        for i in range(self.n_x):
            Xsig[:, i + 1]             = self.x_hat + sqrt_P[:, i]
            Xsig[:, i + 1 + self.n_x] = self.x_hat - sqrt_P[:, i]
        return Xsig


# ---------------------------------------------------------------------------
# Factory — matches ukf_design.m configuration
# ---------------------------------------------------------------------------

#: Measurement matrix: observe [x, θ1, θ2, θ3] — no velocities
C_MEAS = np.zeros((4, 8))
C_MEAS[0, 0] = 1   # x
C_MEAS[1, 2] = 1   # th1
C_MEAS[2, 4] = 1   # th2
C_MEAS[3, 6] = 1   # th3

#: Default process-noise covariance  (ukf_design.m line 37)
Q_DEFAULT = np.diag([1e-6, 1e-4, 1e-6, 1e-4, 1e-6, 1e-4, 1e-6, 1e-4])

#: Default sensor-noise covariance  (ukf_design.m line 38)
R_DEFAULT = np.diag([1e-4, 5e-5, 5e-5, 5e-5])

#: Default initial state covariance  (ukf_design.m lines 59-61)
P0_DEFAULT = np.diag([
    0.01**2,           0.05**2,
    np.deg2rad(1)**2,  0.1**2,
    np.deg2rad(1)**2,  0.1**2,
    np.deg2rad(1)**2,  0.1**2,
])


def build_pendulum_ukf(Ts: float = 0.01, Q=None, R=None):
    """
    Build and return a (UKF, C_meas) pair configured for the triple pendulum.

    Discretises the linearised plant with ZOH at sample rate Ts (default 0.01 s
    = 100 Hz, matching ukf_design.m).  The process model is the resulting Ad, Bd.

    Args:
        Ts : sample period [s]
        Q  : process noise covariance (8×8). Defaults to Q_DEFAULT.
        R  : sensor noise covariance  (4×4). Defaults to R_DEFAULT.

    Returns:
        ukf    : UKF instance (call ukf.reset(x0, P0) before stepping)
        C_meas : (4×8) measurement matrix
    """
    A, B, _, _ = get_linear_system()
    B_col       = B.reshape(-1, 1)

    # ZOH discretisation  (mirrors c2d(ss(A,B,...), Ts) in MATLAB)
    sys_d = StateSpace(A, B_col, np.eye(8), np.zeros((8, 1))).to_discrete(Ts, method="zoh")
    Ad    = sys_d.A
    Bd    = sys_d.B.flatten()

    def process_fn(x, u):
        return Ad @ x + Bd * float(u)

    def measure_fn(x):
        return C_MEAS @ x

    ukf = UKF(
        n_x        = 8,
        n_y        = 4,
        process_fn = process_fn,
        measure_fn = measure_fn,
        Q          = Q if Q is not None else Q_DEFAULT,
        R          = R if R is not None else R_DEFAULT,
    )
    return ukf, C_MEAS
