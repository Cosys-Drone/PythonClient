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

Drone1_name = "SimpleFlight"
print(client.listVehicles())  # Should show: ['SimpleFlight', 'SimpleFlight2']
Drone2_name = "SimpleFlight2"

# Enable API control for both drones
client.enableApiControl(True, vehicle_name=Drone1_name)
client.enableApiControl(True, vehicle_name=Drone2_name)

# Arm both drones
client.armDisarm(True, vehicle_name=Drone1_name)
client.armDisarm(True, vehicle_name=Drone2_name)

# Take off both
client.takeoffAsync(vehicle_name=Drone1_name).join()
client.takeoffAsync(vehicle_name=Drone2_name).join()

# Move Drone1 forward
client.moveByVelocityAsync(2, 0, 0, 3, vehicle_name=Drone1_name)

# Move Drone2 up
client.moveByVelocityAsync(0, 0, -2, 3, vehicle_name=Drone2_name)