import airsim
import numpy as np
import gymnasium as gym  # ✅ Use gymnasium
from gymnasium import spaces
import time
import math
import random

class DroneEnv(gym.Env):  # ✅ Inherit from gymnasium.Env
    def __init__(self):
        super(DroneEnv, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3]) # 1, 0, -1 for each rotor
        
        obs_high = np.array([10, 10, 10, 4, 4, 4, 1, 1, 1, 1, 1, 1, 1, 1000])
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.max_episode_steps = 1000
        self.step_count = 0
        self.rotor_speeds = [0.5] * 4  # Initialize rotor speeds

    def reset(self, seed=None, options=None):  # ✅ Updated signature
        super().reset(seed=seed)
        self.step_count = 0
        self.rotor_speeds = [0.5] * 4  # Initialize rotor speeds
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
        orientation = state.kinematics_estimated.orientation

        pitch, yaw, roll = self.get_pitch_yaw_roll(orientation)
        
        obs = np.array([
            vel.x_val, vel.y_val, vel.z_val,
            pitch, yaw, roll,
            self.rotor_speeds[0], self.rotor_speeds[1], self.rotor_speeds[2], self.rotor_speeds[3],
            direction.x_val, direction.y_val, direction.z_val,
            distance,
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
        
        if (random.random() < 0.005):
            print(f"Rotor Speeds: {self.rotor_speeds}")
        
        
        self.client.moveByMotorPWMsAsync(
            front_left_pwm=self.rotor_speeds[0],
            front_right_pwm=self.rotor_speeds[1],
            rear_left_pwm=self.rotor_speeds[2],
            rear_right_pwm=self.rotor_speeds[3],
            duration=0.1 / 3 # Duration for the motor command
        ).join()

        obs = self._get_obs()
        distance = obs[12] # Distance to the landing pad
        # reward = 10/distance
        reward = 0
        
        self.step_count += 1
        reward += self.step_count * 0.1
        
        """# Get difference between all rotor speeds
        max_speed = max(self.rotor_speeds)
        min_speed = min(self.rotor_speeds)
        # add to reward for small differences
        reward += (0.5 - (max_speed - min_speed)) * 0.08"""
        # punish for if angles exceed certain thresholds
        if (abs(obs[3]) > 0.5):
            reward -= abs(obs[3]) * 0.1  # Pitch
        if (abs(obs[5]) > 0.5):
            reward -= abs(obs[5]) * 0.1  # roll
        if (abs(obs[4]) > 1.57):
            reward -= abs(obs[4]) * 0.1

        # reward for higher rotor speeds
        reward += (sum(self.rotor_speeds) - 3) * 0.5
        
        terminated = self.step_count >= self.max_episode_steps
        
        truncated = False
        
        collision_info = self.client.simGetCollisionInfo()
        
        state = self.client.getMultirotorState()
        z_value = state.kinematics_estimated.position.z_val
        
        angular_velocity_x = min(abs(state.kinematics_estimated.angular_velocity.x_val), 10)
        angular_velocity_y = min(abs(state.kinematics_estimated.angular_velocity.y_val), 10)
        angular_velocity_z = min(abs(state.kinematics_estimated.angular_velocity.z_val), 10)
        
        # Penalize for high angular velocities (spinning)
        if angular_velocity_x > 0.5 or angular_velocity_y > 0.5 or angular_velocity_z > 0.5:
            reward -= 0.3 * (abs(angular_velocity_x) + abs(angular_velocity_y) + abs(angular_velocity_z))
        if (reward < -1000):
            reward = -1000


        
        #reward -= 0.005 * abs(z_value + 70) ** 2 # Maintain height
        if (z_value > 0):
            reward -= 100
            terminated = True

        # sparse reward (if drone is close to landing pad and above it)
        sparseRadius = 50
        if (distance < sparseRadius and z_value < -15):
            reward += ((sparseRadius - distance)/3)**2
            if (random.random() < 0.01):
                print(f"Reward for being close to landing pad: {reward}")

        # Check if collision has occurred
        if collision_info.has_collided:
            print(f"Object name: {collision_info.object_name}")
            if ("Blocker" in collision_info.object_name or ("Cube" in collision_info.object_name)):
              print("Collision with blocker detected. Resetting environment.")
              reward -= 100
              terminated = True
            if (("Cylinder" in collision_info.object_name)):
                if ((distance < 10) and (z_value < -15)):
                    print("Collision with landing pad detected. Resetting environment.")
                    reward += (1000 - self.step_count) * 3
                    terminated = True
                    self.reset()
                else:
                    print("Collision with landing pad detected but not at the right position. Resetting environment.")
                    reward -= 100
                    terminated = True
                    
        
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