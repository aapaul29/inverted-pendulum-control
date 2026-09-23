"""
rl/train.py
-----------
Train a PPO agent on the TriplePendulumEnv using Stable-Baselines3.

Run from the python/ directory:
    python rl/train.py                        # default 1M steps
    python rl/train.py --timesteps 2000000    # longer run
    python rl/train.py --algo sac             # use SAC instead

Progress is logged to TensorBoard:
    tensorboard --logdir rl/logs
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import VecNormalize
from stable_baselines3.common.callbacks import (
    CheckpointCallback,
    EvalCallback,
    CallbackList,
)
from stable_baselines3.common.monitor import Monitor

from envs.triple_pendulum_env import TriplePendulumEnv

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR   = os.path.join(SCRIPT_DIR, "saved_models")
LOGS_DIR     = os.path.join(SCRIPT_DIR, "logs")
VECNORM_PATH = os.path.join(MODELS_DIR, "vec_normalize.pkl")

os.makedirs(MODELS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR,   exist_ok=True)

try:
    import tensorboard  # noqa: F401
    _TB_AVAILABLE = True
except ImportError:
    _TB_AVAILABLE = False
    print("Note: tensorboard not installed — training will run without TB logging.")

# ---------------------------------------------------------------------------
# Hyperparameters
# ---------------------------------------------------------------------------

PPO_KWARGS = dict(
    policy        = "MlpPolicy",
    n_steps       = 2048,
    batch_size    = 64,
    n_epochs      = 10,
    gamma         = 0.99,
    gae_lambda    = 0.95,
    clip_range    = 0.2,
    ent_coef      = 0.005,      # small entropy bonus encourages exploration
    learning_rate = 3e-4,
    policy_kwargs = dict(net_arch=[64, 64]),
    verbose       = 1,
    tensorboard_log = LOGS_DIR if _TB_AVAILABLE else None,
)

SAC_KWARGS = dict(
    policy        = "MlpPolicy",
    buffer_size   = 200_000,
    batch_size    = 256,
    gamma         = 0.99,
    tau           = 0.005,
    learning_rate = 3e-4,
    ent_coef      = "auto",     # SAC auto-tunes entropy coefficient
    policy_kwargs = dict(net_arch=[64, 64]),
    verbose       = 1,
    tensorboard_log = LOGS_DIR if _TB_AVAILABLE else None,
)

# ---------------------------------------------------------------------------
# Training
# ---------------------------------------------------------------------------

def make_env(rank: int = 0, seed: int = 0):
    def _init():
        env = TriplePendulumEnv()
        env = Monitor(env)
        env.reset(seed=seed + rank)
        return env
    return _init


def train(algo: str = "ppo", timesteps: int = 1_000_000, n_envs: int = 4):
    print(f"\n=== Triple Pendulum RL Training ===")
    print(f"  Algorithm : {algo.upper()}")
    print(f"  Timesteps : {timesteps:,}")
    print(f"  Envs      : {n_envs}")
    print(f"  Models    -> {MODELS_DIR}")
    print(f"  Logs      -> {LOGS_DIR}\n")

    # ------------------------------------------------------------------
    # Vectorised + normalised training environment
    # ------------------------------------------------------------------
    train_env = make_vec_env(
        make_env(seed=0),
        n_envs=n_envs,
    )
    # VecNormalize: normalise observations and rewards for stabler training
    train_env = VecNormalize(
        train_env,
        norm_obs    = True,
        norm_reward = True,
        clip_obs    = 10.0,
    )

    # Separate eval env (no reward normalisation so episode returns are readable)
    eval_env = VecNormalize(
        make_vec_env(make_env(seed=999), n_envs=1),
        norm_obs    = True,
        norm_reward = False,
        clip_obs    = 10.0,
        training    = False,
    )

    # ------------------------------------------------------------------
    # Model
    # ------------------------------------------------------------------
    algo = algo.lower()
    if algo == "ppo":
        model = PPO(env=train_env, **PPO_KWARGS)
    elif algo == "sac":
        if n_envs > 1:
            print("SAC works best with n_envs=1 — adjusting.")
            train_env.close(); eval_env.close()
            train_env = VecNormalize(
                make_vec_env(make_env(seed=0), n_envs=1),
                norm_obs=True, norm_reward=True, clip_obs=10.0,
            )
            eval_env = VecNormalize(
                make_vec_env(make_env(seed=999), n_envs=1),
                norm_obs=True, norm_reward=False, clip_obs=10.0, training=False,
            )
        model = SAC(env=train_env, **SAC_KWARGS)
    else:
        raise ValueError(f"Unknown algorithm '{algo}'. Choose 'ppo' or 'sac'.")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    checkpoint_cb = CheckpointCallback(
        save_freq     = max(50_000 // n_envs, 1),
        save_path     = MODELS_DIR,
        name_prefix   = f"{algo}_pendulum",
        save_replay_buffer = (algo == "sac"),
        save_vecnormalize  = True,
        verbose       = 1,
    )

    eval_cb = EvalCallback(
        eval_env,
        best_model_save_path = MODELS_DIR,
        log_path             = LOGS_DIR,
        eval_freq            = max(100_000 // n_envs, 1),
        n_eval_episodes      = 10,
        deterministic        = True,
        verbose              = 1,
    )

    callbacks = CallbackList([checkpoint_cb, eval_cb])

    # ------------------------------------------------------------------
    # Train
    # ------------------------------------------------------------------
    model.learn(total_timesteps=timesteps, callback=callbacks, progress_bar=True)

    # ------------------------------------------------------------------
    # Save final model + normalisation stats
    # ------------------------------------------------------------------
    final_path = os.path.join(MODELS_DIR, f"{algo}_pendulum_final")
    model.save(final_path)
    train_env.save(VECNORM_PATH)
    print(f"\nModel saved  -> {final_path}.zip")
    print(f"VecNormalize -> {VECNORM_PATH}")

    train_env.close()
    eval_env.close()
    return model


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Train RL agent on triple pendulum.")
    parser.add_argument("--algo",       default="ppo",       choices=["ppo", "sac"])
    parser.add_argument("--timesteps",  default=1_000_000,   type=int)
    parser.add_argument("--n_envs",     default=4,           type=int,
                        help="Parallel envs (PPO only; SAC auto-sets to 1)")
    args = parser.parse_args()

    train(algo=args.algo, timesteps=args.timesteps, n_envs=args.n_envs)
