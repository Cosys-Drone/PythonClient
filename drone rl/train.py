from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from drone_env_step1a_3 import DroneEnv1a_3

from stable_baselines3.common.callbacks import BaseCallback

import numpy as np

env = DroneEnv1a_3()
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
    save_path="./checkpoints1a_3_audrey/",  # Folder to save models
    name_prefix="drone_model",   # File name prefix
    save_replay_buffer=True,     # Optional: save replay buffer for off-policy algorithms
    save_vecnormalize=True       # Optional: save normalization stats
)

from stable_baselines3.common.callbacks import CallbackList

combined_callback = CallbackList([
    checkpoint_callback,
    EpisodeLoggingCallback()
])

model = PPO("MlpPolicy", env, verbose=1, tensorboard_log=".EDD_tensorboard/1a_3", n_steps=4096, batch_size=128)
#model = PPO.load("./checkpoints1a_c/drone_model_50000_steps", env=env, tensorboard_log="./EDD_tensorboard/1a", n_steps=1024, batch_size=64)

model.learn(
    total_timesteps=5_000_000,
    callback=combined_callback,
    reset_num_timesteps=True  # False for resuming training
)