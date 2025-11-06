from stable_baselines3 import PPO
from drone_env_step1a import DroneEnv1a
import time

env = DroneEnv1a()
n_episodes = 50

# Load the trained model
model = PPO.load("./checkpoints1a_c/drone_model_140000_steps.zip", env=env)  # or whatever your file is named

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