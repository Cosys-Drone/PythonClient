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
        
        
client = airsim.MultirotorClient()
client.confirmConnection()
client.reset()
client.enableApiControl(True)
client.armDisarm(True)
client.takeoffAsync().join()

# Fly forward
# client.moveByVelocityAsync(5, 0, 0, duration=3).join()

rotor_states = client.getRotorStates(vehicle_name="SimpleFlight")

print("Rotor States:")

# Each rotor state typically contains:
for i, rotor in enumerate(rotor_states.rotors):
    print(f"Rotor {i}:")
    print(f"  Thrust: {rotor.thrust}")
    print(f"  Torque Scaler: {rotor.torque_scaler}")
    print(f"  Speed: {rotor.speed}")

client.landAsync().join()
client.armDisarm(False)
client.enableApiControl(False)