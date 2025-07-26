import airsim
from pynput import keyboard
import threading
import time


for _ in range(3):
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        break
    except Exception as e:
        print("Waiting for AirSim server...")
        time.sleep(2)

client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

velocity = 5  # Increase for higher speed
duration = 0.25  # Duration to maintain velocity per command

pressed_keys = set()

rotor_speeds = [0] * 4

def send_movement():
    global rotor_speeds

    #increase rotor speeds
    if 'w' in pressed_keys:
        rotor_speeds[0] = min(rotor_speeds[0] + 0.01, 1)
    if 's' in pressed_keys:
        rotor_speeds[1] = min(rotor_speeds[1] + 0.01, 1)
    if 'a' in pressed_keys:
        rotor_speeds[2] = min(rotor_speeds[2] + 0.01, 1)
    if 'd' in pressed_keys:
        rotor_speeds[3] = min(rotor_speeds[3] + 0.01, 1)
    #lower rotor speeds
    if 'i' in pressed_keys:
        rotor_speeds[0] = max(0,min(rotor_speeds[0] - 0.01, 1))
    if 'k' in pressed_keys:
        rotor_speeds[1] = max(0,min(rotor_speeds[1] - 0.01, 1))
    if 'j' in pressed_keys:
        rotor_speeds[2] = max(0,min(rotor_speeds[2] - 0.01, 1))
    if 'l' in pressed_keys:
        rotor_speeds[3] = max(0,min(rotor_speeds[3] - 0.01, 1))
    if 'r' in pressed_keys:
        client.reset()
        rotor_speeds = [0] * 4
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()

    client.moveByMotorPWMsAsync(
            front_left_pwm=rotor_speeds[0],
            front_right_pwm=rotor_speeds[1],
            rear_left_pwm=rotor_speeds[2],
            rear_right_pwm=rotor_speeds[3],
            duration=0.1 / 3 # Duration for the motor command
        ).join()
    
    # Get the drone's pose (position and orientation)
    pose = client.simGetVehiclePose()

    # Access the orientation from the pose
    orientation = pose.orientation

    # You can then convert the quaternion to Euler angles (roll, pitch, yaw)
    # For example, using the AirSim utilities:
    pitch, roll, yaw = airsim.to_eularian_angles(orientation)

    # Print the angles
    print(f"Roll: {roll} degrees")
    print(f"Pitch: {pitch} degrees")
    print(f"Yaw: {yaw} degrees")

def on_press(key):
    try:
        k = key.char.lower()
    except:
        k = key.name.lower()

    pressed_keys.add(k)

def on_release(key):
    try:
        k = key.char.lower()
    except:
        k = key.name.lower()

    if k in pressed_keys:
        pressed_keys.remove(k)

    if key == keyboard.Key.esc:
        return False  # Stop listener

listener = keyboard.Listener(on_press=on_press, on_release=on_release)
listener.start()

import math
def get_direction_and_distance(drone_pos, pad_pos):
        # Calculate direction vector
        direction = airsim.Vector3r(
            pad_pos.x_val - drone_pos.x_val,
            pad_pos.y_val - drone_pos.y_val,
            pad_pos.z_val - drone_pos.z_val
        )

        # Calculate magnitude (Euclidean distance)
        distance = math.sqrt(
            (direction.x_val**2) * 4 +
            (direction.y_val**2) * 1 +
            (direction.z_val**2) * 0.5
        )
        
        '''
        public static void SetToAirSim(Vector3 src, ref AirSimVector dst) {
            dst.Set(src.z, src.x, -src.y);
        }
        '''

        return direction/distance, distance

try:
    while listener.running:
        send_movement()
        time.sleep(0.05)
        
        collision_info = client.simGetCollisionInfo()

        # Check if collision has occurred
        if collision_info.has_collided:
            print(f"Object name: {collision_info.object_name}")
            if ("Blocker" in collision_info.object_name):
                print("Collision detected with the blocker!")
                
        state = client.getMultirotorState()
                
        direction, distance = get_direction_and_distance(
            state.kinematics_estimated.position, airsim.Vector3r(261.7, -319.6, -15))
        
except KeyboardInterrupt:
    pass

listener.stop()
client.armDisarm(False)
client.enableApiControl(False)