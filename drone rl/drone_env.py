import airsim
import numpy as np
import gymnasium as gym  # ✅ Use gymnasium
from gymnasium import spaces
import time
import math

class DroneEnv(gym.Env):  # ✅ Inherit from gymnasium.Env
    def __init__(self):
        super(DroneEnv, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.action_space = spaces.MultiDiscrete([3, 3, 3]) # 1, 0, -1 for each axis
        
        obs_high = np.array([10, 10, 10, 1, 1, 1, 1000])
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.max_episode_steps = 500
        self.step_count = 0

    def reset(self, seed=None, options=None):  # ✅ Updated signature
        super().reset(seed=seed)
        self.step_count = 0
        self.client.reset()
        time.sleep(0.5)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        obs = self._get_obs()
        print("----------- Resetting Environment -----------")
        return obs, {}  # ✅ Gymnasium requires tuple return

    def _get_obs(self):
        state = self.client.getMultirotorState()
        vel = state.kinematics_estimated.linear_velocity
        
        direction, distance = self.get_direction_and_distance(state.kinematics_estimated.position, airsim.Vector3r(261.7, -319.6, -15))
        
        '''
        public static void SetToAirSim(Vector3 src, ref AirSimVector dst) {
            dst.Set(src.z, src.x, -src.y);
        }
        '''
        
        obs = np.array([
            vel.x_val, vel.y_val, vel.z_val,
            direction.x_val, direction.y_val, direction.z_val,
            distance,
        ], dtype=np.float32)
        
        return obs

    def step(self, action):
      
        direction_map = {0: -1, 1: 0, 2: 1}
        dx = direction_map[action[0]]
        dy = direction_map[action[1]]
        dz = direction_map[action[2]]
        
        speed = 5  # m/s
        duration = 0.25  # seconds
        
        self.client.moveByVelocityAsync(
            vx=dx * speed,
            vy=dy * speed,
            vz=dz * speed,
            duration=duration,
        ).join()

        obs = self._get_obs()
        distance = obs[6] # Distance to the landing pad
        reward = 10/distance
        # Print self positon and landing pad position
        
        self.step_count += 1
        terminated = self.step_count >= self.max_episode_steps
        truncated = False  # optional early stopping condition
        
        
        collision_info = self.client.simGetCollisionInfo()

        # Check if collision has occurred
        if collision_info.has_collided:
            print(f"Object name: {collision_info.object_name}")
            if ("Blocker" in collision_info.object_name or ("Cube" in collision_info.object_name)):
              print("Collision with blocker detected. Resetting environment.")
              reward -= 100
              self.reset()
                
        
        return obs, float(reward), terminated, truncated, {}

    def render(self):
        pass

    def close(self):
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def get_direction_and_distance(self, drone_pos, pad_pos):
        # Calculate direction vector
        direction = airsim.Vector3r(
            pad_pos.x_val - drone_pos.x_val,
            pad_pos.y_val - drone_pos.y_val,
            pad_pos.z_val - drone_pos.z_val
        )

        # Calculate magnitude (Euclidean distance)
        distance = math.sqrt(
            direction.x_val**2 +
            direction.y_val**2 +
            direction.z_val**2
        )

        return direction/distance, distance