import airsim
import numpy as np
import gymnasium as gym  # ✅ Use gymnasium
from gymnasium import spaces
import time
import math
import random

# Define a constant for the clock speed
clockspeed = 3  # Adjust this value as needed

class DroneEnv1b(gym.Env):
    def __init__(self):
        super(DroneEnv1b, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3]) # 1, 0, -1 for each rotor
        
        # initialize the observation space array
        obs_high = np.array([10, 10, 10, 1.6, 3.2, 3.2, 1, 1, 1, 1, 1, 1, 1, 200, 200, 3.2, 3.2, 3.2])
        # Order: xyz velocities (3), pitch, yaw, roll, rotor speeds (4), normalized direction to pad (3), distance, altitude, angular velocities (3)
        
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.max_episode_steps = 500
        self.step_count = 0
        self.rotor_speeds = [0.5] * 4  # Initialize rotor speeds
        self.target_altitude = random.randint(-90, -50)  # Random target altitude between -90 and -50

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
        
        direction, distance = self.get_direction_and_distance(state.kinematics_estimated.position, airsim.Vector3r(261.7, -319.6, -15))
        
        orientation = state.kinematics_estimated.orientation

        # angles (in radians)
        pitch, yaw, roll = self.get_pitch_yaw_roll(orientation)
        
        # angular velocity (in radians)
        angular_velocity_x = abs(state.kinematics_estimated.angular_velocity.x_val)
        angular_velocity_y = abs(state.kinematics_estimated.angular_velocity.y_val)
        angular_velocity_z = abs(state.kinematics_estimated.angular_velocity.z_val)
        
        # Set observations
        obs = np.array([
            vel.x_val, vel.y_val, vel.z_val,
            pitch, yaw, roll,
            self.rotor_speeds[0], self.rotor_speeds[1], self.rotor_speeds[2], self.rotor_speeds[3],
            direction.x_val, direction.y_val, direction.z_val,
            distance,
            state.kinematics_estimated.position.z_val, # Altitude
            angular_velocity_x, angular_velocity_y, angular_velocity_z
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
        
        self.client.moveByMotorPWMsAsync(
            front_left_pwm=self.rotor_speeds[0],
            front_right_pwm=self.rotor_speeds[1],
            rear_left_pwm=self.rotor_speeds[2],
            rear_right_pwm=self.rotor_speeds[3],
            duration=0.1 / 3 # Duration for the motor command
        ).join()

        obs = self._get_obs()
        # Order: xyz velocities (3), pitch, yaw, roll, rotor speeds (4), normalized direction to pad (3), distance, altitude, angular velocities (3)
        state = self.client.getMultirotorState()
        z_value = state.kinematics_estimated.position.z_val
        angular_velocity_x = abs(state.kinematics_estimated.angular_velocity.x_val)
        angular_velocity_y = abs(state.kinematics_estimated.angular_velocity.y_val)
        angular_velocity_z = abs(state.kinematics_estimated.angular_velocity.z_val)
        
        reward = 0
        truncated = False
        terminated = False
        
        terminated = self.step_count >= self.max_episode_steps
        self.step_count += 1

        
        # REWARDS + PUNISHIES :3

        # Rotor Speeds (+ Offset)
        rangeOfOffset = max(self.rotor_speeds[0],self.rotor_speeds[1],self.rotor_speeds[2],self.rotor_speeds[3]) - min(self.rotor_speeds[0],self.rotor_speeds[1],self.rotor_speeds[2],self.rotor_speeds[3])
        if (rangeOfOffset > 0.05): 
            reward -= rangeOfOffset*0.1
        if (rangeOfOffset < 0.05): 
            reward += 0.08


        # Drone Angle of Rotation
        pitch = obs[3]
        roll = obs[5]
        angle = max(pitch, roll) / math.pi * 180  # Convert to degrees
        if (angle > 30):  # 30 degrees
            reward -= min(angle * 0.001, 0.075)
        if (angle < 15):  # 15 degrees
            reward += 0.06


        # Drone Angular Velocity
        # Reset episode if exceeds pi radians per second
        if (max(angular_velocity_x, angular_velocity_y, angular_velocity_z) > math.pi):
            reward -= 30
            print("Angular velocity exceeded limit!")
            terminated = True


        # Drone Position
        if (z_value > 0):
            reward -= 10
            terminated = True
            
        # Altitude
        altitude_diff = abs(z_value - self.target_altitude)
        if (altitude_diff < 20):
            reward += (10 - (altitude_diff ** 2) / 20) / 100
        if (altitude_diff > 20):
            reward += (-(altitude_diff) / 5 - 6) / 100

        # Living
        reward += 0.1

        
        return obs, float(reward), terminated, truncated, {}

    def render(self):
        pass

    def close(self):
        self.client.armDisarm(False)
        self.client.enableApiControl(False)

    def get_direction_and_distance(self, drone_pos, pad_pos):
        # Calculate direction vector (height difference only)
        direction = airsim.Vector3r(
            0,
            0,
            pad_pos.z_val - drone_pos.z_val
        )

        # Calculate magnitude (Euclidean distance)
        distance = math.sqrt(
            (0) +
            (0) +
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

        # You can then convert the quaternion to Euler angles (pitch, roll, yaw)
        # For example, using the AirSim utilities:
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)
        
        return pitch, yaw, roll