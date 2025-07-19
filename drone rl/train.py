from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
from drone_env import DroneEnv

from stable_baselines3.common.callbacks import BaseCallback

import numpy as np

env = DroneEnv()
check_env(env)  # Check if the environment follows the Gymnasium API

class TensorboardCallback(BaseCallback):
    def __init__(self, verbose=0):
        super().__init__(verbose)

    def _on_step(self) -> bool:
        reward = self.locals.get("rewards", [0])[-1]
        self.logger.record("custom/reward", reward, exclude="stdout")
        self.logger.dump(self.num_timesteps)  # Force write to disk
        return True


model = PPO("MlpPolicy", env, verbose=1, tensorboard_log="./ppo_airsim_drone_tensorboard/")

model.learn(
    total_timesteps=100_000,
    n_steps=128,
    callback=TensorboardCallback(),
)

model.save("ppo_airsim_drone")