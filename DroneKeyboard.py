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
duration = 0.1  # Duration to maintain velocity per command

pressed_keys = set()

def send_movement():
    vx, vy, vz = 0, 0, 0

    if 'w' in pressed_keys:
        vx += velocity
    if 's' in pressed_keys:
        vx -= velocity
    if 'a' in pressed_keys:
        vy -= velocity
    if 'd' in pressed_keys:
        vy += velocity
    if 'up' in pressed_keys:
        vz -= velocity  # Move up
    if 'down' in pressed_keys:
        vz += velocity  # Move down
    if 'r' in pressed_keys:
        client.reset()
        client.enableApiControl(True)
        client.armDisarm(True)
        client.takeoffAsync().join()

    client.moveByVelocityAsync(vx, vy, vz, duration, drivetrain=airsim.DrivetrainType.MaxDegreeOfFreedom,
                               yaw_mode=airsim.YawMode(False, 0))

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
                
                
        # print(client.simListSceneObjects())
        
except KeyboardInterrupt:
    pass

listener.stop()
client.armDisarm(False)
client.enableApiControl(False)