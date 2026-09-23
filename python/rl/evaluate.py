"""
rl/evaluate.py
--------------
Load a trained RL model and evaluate it on the triple pendulum.

Run from the python/ directory:
    python rl/evaluate.py                             # loads best_model.zip
    python rl/evaluate.py --model rl/saved_models/ppo_pendulum_final.zip
    python rl/evaluate.py --episodes 20 --render
"""

import os
import sys
import argparse

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

from envs.triple_pendulum_env import TriplePendulumEnv

SCRIPT_DIR   = os.path.dirname(os.path.abspath(__file__))
MODELS_DIR   = os.path.join(SCRIPT_DIR, "saved_models")
VECNORM_PATH = os.path.join(MODELS_DIR, "vec_normalize.pkl")


def load_model(model_path: str):
    """Auto-detect algorithm from filename and load the model."""
    name = os.path.basename(model_path).lower()
    cls  = SAC if "sac" in name else PPO
    return cls.load(model_path)


def evaluate(
    model_path:  str  = None,
    n_episodes:  int  = 10,
    render:      bool = False,
    seed:        int  = 0,
):
    # ------------------------------------------------------------------
    # Resolve model path
    # ------------------------------------------------------------------
    if model_path is None:
        best = os.path.join(MODELS_DIR, "best_model.zip")
        final_ppo = os.path.join(MODELS_DIR, "ppo_pendulum_final.zip")
        if os.path.exists(best):
            model_path = best
        elif os.path.exists(final_ppo):
            model_path = final_ppo
        else:
            raise FileNotFoundError(
                f"No model found in {MODELS_DIR}. Run train.py first."
            )

    print(f"\nLoading model : {model_path}")
    model = load_model(model_path)

    # ------------------------------------------------------------------
    # Wrap env with the same VecNormalize stats used during training
    # ------------------------------------------------------------------
    render_mode = "human" if render else None
    raw_env     = DummyVecEnv([lambda: TriplePendulumEnv(render_mode=render_mode)])

    if os.path.exists(VECNORM_PATH):
        env = VecNormalize.load(VECNORM_PATH, raw_env)
        env.training  = False   # freeze normalisation stats
        env.norm_reward = False
        print(f"VecNormalize  : {VECNORM_PATH}\n")
    else:
        env = raw_env
        print("VecNormalize  : not found — running without normalisation\n")

    # ------------------------------------------------------------------
    # Evaluation loop
    # ------------------------------------------------------------------
    episode_rewards = []
    episode_lengths = []

    for ep in range(n_episodes):
        obs   = env.reset()
        done  = False
        total = 0.0
        steps = 0

        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = env.step(action)
            total += float(np.asarray(reward).flat[0])
            done = bool(np.asarray(done).flat[0])
            steps += 1

        episode_rewards.append(total)
        episode_lengths.append(steps)
        survived_s = steps * 0.02   # dt = 0.02 s
        print(f"  Episode {ep+1:>2d}  |  steps: {steps:>4d}  "
              f"({survived_s:.1f} s)  |  reward: {total:+.1f}")

    env.close()

    # ------------------------------------------------------------------
    # Summary
    # ------------------------------------------------------------------
    print(f"\n{'─'*50}")
    print(f"  Episodes       : {n_episodes}")
    print(f"  Mean reward    : {np.mean(episode_rewards):+.2f} ± {np.std(episode_rewards):.2f}")
    print(f"  Mean survival  : {np.mean(episode_lengths)*0.02:.2f} s  "
          f"(max {np.max(episode_lengths)*0.02:.2f} s)")
    print(f"  Success rate   : "
          f"{100*np.mean(np.array(episode_lengths) >= 500):.0f}%  "
          f"(full 10 s episode = success)")


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Evaluate a trained RL agent.")
    parser.add_argument("--model",    default=None,  help="Path to .zip model file")
    parser.add_argument("--episodes", default=10,    type=int)
    parser.add_argument("--render",   action="store_true", help="Show live animation")
    parser.add_argument("--seed",     default=0,     type=int)
    args = parser.parse_args()

    evaluate(
        model_path  = args.model,
        n_episodes  = args.episodes,
        render      = args.render,
        seed        = args.seed,
    )
