import airsim
import time

for _ in range(3):
    try:
        client = airsim.MultirotorClient()
        client.confirmConnection()
        break
    except Exception as e:
        print("Waiting for AirSim server...")
        time.sleep(2)

client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Fly forward
# client.moveByVelocityAsync(5, 0, 0, duration=3).join()

client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)