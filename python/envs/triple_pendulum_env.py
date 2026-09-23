import os
import sys
import numpy as np
from scipy.integrate import solve_ivp
import gymnasium as gym
from gymnasium import spaces

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from utils.dynamics import nonlinear_dynamics


class TriplePendulumEnv(gym.Env):
    """
    Gymnasium environment for a triple inverted pendulum on a cart.

    Observation (8,):  [x, xd, θ1, θ1d, θ2, θ2d, θ3, θ3d]
        θ1  — absolute angle of link 1 from vertical   [rad]
        θ2  — angle of link 2 relative to link 1       [rad]
        θ3  — angle of link 3 relative to link 2       [rad]

    Action (1,):  F — horizontal cart force [N], clipped to ±F_max

    Episode ends when any absolute link angle exceeds theta_limit (~30°),
    the cart leaves ±x_limit, or max_episode_steps is reached.
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}

    def __init__(
        self,
        render_mode=None,
        dt=0.02,
        max_episode_steps=500,
        F_max=50.0,
        x_limit=2.0,
        theta_limit=np.pi / 6,
    ):
        super().__init__()

        self.dt               = dt
        self.max_episode_steps = max_episode_steps
        self.F_max            = float(F_max)
        self.x_limit          = float(x_limit)
        self.theta_limit      = float(theta_limit)
        self.render_mode      = render_mode

        self._state      = None
        self._step_count = 0
        self._fig        = None   # for "human" render

        # ------------------------------------------------------------------
        # Spaces
        # ------------------------------------------------------------------
        obs_high = np.array(
            [x_limit * 2, np.inf, np.pi, np.inf, np.pi, np.inf, np.pi, np.inf],
            dtype=np.float64,
        )
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float64)
        self.action_space      = spaces.Box(
            low=-self.F_max, high=self.F_max, shape=(1,), dtype=np.float64
        )

    # ------------------------------------------------------------------
    # Core API
    # ------------------------------------------------------------------

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)

        # Small random perturbation around the upright equilibrium.
        # Angles nudged up to ±3° (0.05 rad); velocities up to ±0.05 rad/s.
        noise_scale = np.array([0.0, 0.0, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05])
        self._state      = self.np_random.uniform(-noise_scale, noise_scale)
        self._step_count = 0

        if self.render_mode == "human":
            self._render_frame()

        return self._state.copy(), {}

    def step(self, action):
        u = float(np.clip(np.asarray(action).flat[0], -self.F_max, self.F_max))

        # Integrate one control step with the full nonlinear dynamics.
        sol = solve_ivp(
            fun=lambda t, s: nonlinear_dynamics(s, u),
            t_span=(0.0, self.dt),
            y0=self._state,
            method="RK45",
            max_step=self.dt / 4,
        )
        self._state      = sol.y[:, -1]
        self._step_count += 1

        # Absolute link angles
        x, _, th1, _, th2, _, th3, _ = self._state
        phi1 = th1
        phi2 = th1 + th2
        phi3 = th1 + th2 + th3

        terminated = bool(
            abs(x)    > self.x_limit
            or abs(phi1) > self.theta_limit
            or abs(phi2) > self.theta_limit
            or abs(phi3) > self.theta_limit
        )
        truncated = self._step_count >= self.max_episode_steps

        reward = self._reward(x, phi1, phi2, phi3, u, terminated)

        if self.render_mode == "human":
            self._render_frame()

        return self._state.copy(), reward, terminated, truncated, {}

    # ------------------------------------------------------------------
    # Reward
    # ------------------------------------------------------------------

    def _reward(self, x, phi1, phi2, phi3, u, terminated):
        if terminated:
            return -10.0

        angle_cost  = phi1**2 + phi2**2 + phi3**2
        x_cost      = 0.1  * x**2
        action_cost = 0.001 * u**2

        # +1 alive bonus, minus smooth costs that grow as system deviates
        return 1.0 - angle_cost - x_cost - action_cost

    # ------------------------------------------------------------------
    # Rendering  (matplotlib, only when render_mode="human")
    # ------------------------------------------------------------------

    def render(self):
        if self.render_mode == "rgb_array":
            return self._render_frame()

    def _render_frame(self):
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            import matplotlib.patches as patches
            from utils.parameters import l1, l2, l3
        except ImportError:
            return

        if self._fig is None:
            plt.ion()
            self._fig, self._ax = plt.subplots(figsize=(8, 5))

        ax = self._ax
        ax.cla()

        x, _, th1, _, th2, _, th3, _ = self._state
        phi1 = th1
        phi2 = th1 + th2
        phi3 = th1 + th2 + th3

        # Joint positions
        x0, y0 = x, 0.0
        x1 = x0 + l1 * np.sin(phi1);  y1 = y0 + l1 * np.cos(phi1)
        x2 = x1 + l2 * np.sin(phi2);  y2 = y1 + l2 * np.cos(phi2)
        x3 = x2 + l3 * np.sin(phi3);  y3 = y2 + l3 * np.cos(phi3)

        # Track
        ax.axhline(0, color="gray", linewidth=1)
        ax.add_patch(patches.FancyBboxPatch(
            (x - 0.15, -0.07), 0.30, 0.07,
            boxstyle="round,pad=0.01", color="steelblue"
        ))

        # Links
        for (xa, ya, xb, yb) in [(x0,y0,x1,y1),(x1,y1,x2,y2),(x2,y2,x3,y3)]:
            ax.plot([xa, xb], [ya, yb], "o-", color="coral", linewidth=3, markersize=6)

        total_len = l1 + l2 + l3
        ax.set_xlim(x - total_len - 0.3, x + total_len + 0.3)
        ax.set_ylim(-0.3, total_len + 0.2)
        ax.set_aspect("equal")
        ax.set_title(
            f"step={self._step_count}   "
            f"φ1={np.degrees(phi1):.1f}°  "
            f"φ2={np.degrees(phi2):.1f}°  "
            f"φ3={np.degrees(phi3):.1f}°"
        )

        self._fig.canvas.draw()
        self._fig.canvas.flush_events()

    def close(self):
        if self._fig is not None:
            import matplotlib.pyplot as plt
            plt.close(self._fig)
            self._fig = None
