import airsim
import numpy as np
import gymnasium as gym  # ✅ Use gymnasium
from gymnasium import spaces
import time
import math
import random

# Define a constant for the clock speed
clockspeed = 3  # Adjust this value as needed

class DroneEnv(gym.Env):  # ✅ Inherit from gymnasium.Env
    def __init__(self):
        super(DroneEnv, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3]) # 1, 0, -1 for each rotor
        
        obs_high = np.array([10, 10, 10, 4, 4, 4, 1, 1, 1, 1, 1, 1, 1, 1000, 1000])
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.max_episode_steps = 500
        self.step_count = 0
        self.rotor_speeds = [0.5] * 4  # Initialize rotor speeds

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.rotor_speeds = [random.randint(30, 100) / 100] * 4  # Initialize rotor speeds
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
        
        # direction, distance = self.get_direction_and_distance(state.kinematics_estimated.position, airsim.Vector3r(261.7, -319.6, -15))
        direction = 0
        distance = 0
        # Prior 
        
        orientation = state.kinematics_estimated.orientation

        pitch, yaw, roll = self.get_pitch_yaw_roll(orientation)
        
        # Set observations
        obs = np.array([
            vel.x_val, vel.y_val, vel.z_val,
            pitch, yaw, roll,
            self.rotor_speeds[0], self.rotor_speeds[1], self.rotor_speeds[2], self.rotor_speeds[3],
            direction.x_val, direction.y_val, direction.z_val,
            distance,
            state.kinematics_estimated.position.z_val # Altitude
        ], dtype=np.float32)
        
        return obs

    def step(self, action):
      
        direction_map = {0: -1, 1: 0, 2: 1}
        top_left = direction_map[action[0]]
        top_right = direction_map[action[1]]
        bottom_left = direction_map[action[2]]
        bottom_right = direction_map[action[3]]
        
        self.rotor_speeds[0] = min(max(0, self.rotor_speeds[0] + 0.01 * top_left), 1)
        self.rotor_speeds[1] = min(max(0, self.rotor_speeds[1] + 0.01 * top_right), 1)
        self.rotor_speeds[2] = min(max(0, self.rotor_speeds[2] + 0.01 * bottom_left), 1)
        self.rotor_speeds[3] = min(max(0, self.rotor_speeds[3] + 0.01 * bottom_right), 1)

        obs = self._get_obs()
        reward = 0
        truncated = False
        terminated = False
        
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
            (direction.x_val**2) +
            (direction.y_val**2) +
            (direction.z_val**2)
        )
        
        '''
        public static void SetToAirSim(Vector3 src, ref AirSimVector dst) {
            dst.Set(src.z, src.x, -src.y);
        }
        '''

        return direction/distance, distance
      
    def get_pitch_yaw_roll(self, quaternion):
        # Get the drone's pose (position and orientation)
        pose = self.client.simGetVehiclePose()

        # Access the orientation from the pose
        orientation = pose.orientation

        # You can then convert the quaternion to Euler angles (roll, pitch, yaw)
        # For example, using the AirSim utilities:
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)
        
        return pitch, yaw, roll