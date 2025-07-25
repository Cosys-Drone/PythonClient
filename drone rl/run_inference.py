from stable_baselines3 import PPO
from drone_env import DroneEnv
import time

env = DroneEnv()
n_episodes = 5

# Load the trained model
model = PPO.load("./checkpoints11c/drone_model_200000_steps.zip", env=env)  # or whatever your file is named

for ep in range(n_episodes):
    obs, _ = env.reset()
    done = False
    truncated = False
    total_reward = 0

    while not (done or truncated):  
        action, _ = model.predict(obs, deterministic=False)
        obs, reward, done, truncated, info = env.step(action)
        total_reward += reward

    print(f"Episode {ep + 1} finished with reward: {total_reward}")