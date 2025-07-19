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

while True:
  state = client.getMultirotorState()
  velocity = state.kinematics_estimated.linear_velocity
  print("Velocity:", velocity)
  time.sleep(2)