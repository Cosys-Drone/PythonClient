import airsim
import numpy as np
import gymnasium as gym  # ✅ Use gymnasium
from gymnasium import spaces
import time
import math
import random

# Define a constant for the clock speed
clockspeed = 10  # Adjust this value as needed

class DroneEnv1ab3(gym.Env):  # ✅ Inherit from gymnasium.Env
    def __init__(self):
        super(DroneEnv1ab3, self).__init__()
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.client.enableApiControl(True)
        self.client.armDisarm(True)

        self.action_space = spaces.MultiDiscrete([3, 3, 3, 3]) # 1, 0, -1 for each rotor
        
        # initialize the observation space array
        # does NOT include the speed of the disabled rotor
        obs_high = np.array([10, 10, 10, 1.6, 3.2, 3.2, 1, 1, 1, 1, 1, 1, 200, 200, 3.2, 3.2, 3.2])
        # Order: xyz velocities (3), pitch, yaw, roll, rotor speeds (4), normalized direction to pad (3), distance, altitude, angular velocities (3)
        
        self.observation_space = spaces.Box(-obs_high, obs_high, dtype=np.float32)

        self.max_episode_steps = 500
        self.step_count = 0
        self.rotor_speeds = [0] * 4  # Initialize rotor speeds
        self.rotor_speeds[0] = 0  # Disabled rotor
        self.target_altitude = -70

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.step_count = 0
        self.rotor_speeds = [random.randint(0, 10) / 100] * 4  # Initialize rotor speeds
        print("Initial rotor speeds (0 disabled): ", self.rotor_speeds)
        
        self.rotor_speeds[0] = 0  # Disabled rotor
        self.target_altitude = random.randint(-90, -50)
        # (removed random initial rotor speeds for simplicity)
        self.client.reset()
        
        time.sleep(0.5 / clockspeed)
        self.client.enableApiControl(True)
        self.client.armDisarm(True)
        self.client.takeoffAsync().join()
        
        obs = self._get_obs()
        print("----------- Resetting Environment -----------")
        return obs, {}  # ✅ Gymnasium requires tuple return

    def _get_obs(self):
        state = self.client.getMultirotorState()
        vel = state.kinematics_estimated.linear_velocity
        
        direction, distance = self.get_direction_and_distance(state.kinematics_estimated.position, airsim.Vector3r(state.kinematics_estimated.position.x_val, state.kinematics_estimated.position.y_val, self.target_altitude))
        
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
            self.rotor_speeds[1], self.rotor_speeds[2], self.rotor_speeds[3],
            direction.x_val, direction.y_val, direction.z_val,
            distance,
            state.kinematics_estimated.position.z_val, # Altitude
            angular_velocity_x, angular_velocity_y, angular_velocity_z
        ], dtype=np.float32)
        
        return obs

    def step(self, action):

        self.step_count += 1
      
        direction_map = {0: -1, 1: 0, 2: 1}
        #top_left = direction_map[action[0]]
        top_right = direction_map[action[1]]
        bottom_left = direction_map[action[2]]
        bottom_right = direction_map[action[3]]
        
        #self.rotor_speeds[0] = min(max(0, self.rotor_speeds[0] + 0.01 * top_left), 1)
        self.rotor_speeds[1] = min(max(0, self.rotor_speeds[1] + 0.01 * top_right), 1)
        self.rotor_speeds[2] = min(max(0, self.rotor_speeds[2] + 0.01 * bottom_left), 1)
        self.rotor_speeds[3] = min(max(0, self.rotor_speeds[3] + 0.01 * bottom_right), 1)
        
        self.client.moveByMotorPWMsAsync(
            front_left_pwm=0, # Disabled rotor
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

        if (z_value > 0):
          # Log values
          print("Crashed into the ground!")
          print('Angular Speeds (x, y, z): ', angular_velocity_x, angular_velocity_y, angular_velocity_z)
          print('Velocity (x, y, z): ', obs[0], obs[1], obs[2])

        
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

        # You can then convert the quaternion to Euler angles (pitch, roll, yaw)
        # For example, using the AirSim utilities:
        pitch, roll, yaw = airsim.to_eularian_angles(orientation)
        
        return pitch, yaw, roll