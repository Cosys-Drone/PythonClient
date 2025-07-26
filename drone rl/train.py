from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from drone_env import DroneEnv

from stable_baselines3.common.callbacks import BaseCallback

import numpy as np

env = DroneEnv()
check_env(env)  # Check if the environment follows the Gymnasium API

from stable_baselines3.common.callbacks import BaseCallback

class EpisodeLoggingCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)
        self.episode_rewards = []
        self.current_reward = 0.0
        self.episode_lengths = []
        self.current_length = 0

    def _on_step(self) -> bool:
        self.current_reward += self.locals["rewards"][0]
        self.current_length += 1

        done = self.locals["dones"][0]
        if done:
            self.episode_rewards.append(self.current_reward)
            self.episode_lengths.append(self.current_length)

            # Log to TensorBoard
            self.logger.record("custom/episode_reward", self.current_reward)
            self.logger.record("custom/episode_length", self.current_length)
            self.logger.dump(self.num_timesteps)

            if self.verbose > 0:
                print(f"[Step {self.num_timesteps}] Reward: {self.current_reward:.2f} | Length: {self.current_length}")

            # Reset for next episode
            self.current_reward = 0.0
            self.current_length = 0

        return True
    
from stable_baselines3.common.callbacks import CheckpointCallback
checkpoint_callback = CheckpointCallback(
    save_freq=10_000,
    save_path="./checkpoints11c/",  # Folder to save models
    name_prefix="drone_model",   # File name prefix
    save_replay_buffer=True,     # Optional: save replay buffer for off-policy algorithms
    save_vecnormalize=True       # Optional: save normalization stats
)

from stable_baselines3.common.callbacks import CallbackList

combined_callback = CallbackList([
    checkpoint_callback,
    EpisodeLoggingCallback()
])

# model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_airsim_drone_tensorboard/", n_steps=1024, batch_size=64)
model = PPO.load("./checkpoints11c/drone_model_350000_steps", env=env, tensorboard_log="./ppo_airsim_drone_tensorboard/", n_steps=1024, batch_size=64)

model.learn(
    total_timesteps=2_000_000,
    callback=combined_callback,
    reset_num_timesteps=False  # For resuming training
)