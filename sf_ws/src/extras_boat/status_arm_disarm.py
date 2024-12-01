from pymavlink import mavutil
import time

# Create the connection to the vehicle
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')

master.wait_heartbeat()

print("Connection established!")

# Function to check if the vehicle is armed
def is_armed(heartbeat):
    return heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED != 0

# Continuously check and print armed status
try:
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True)
        if heartbeat:
            armed_status = "Armed" if is_armed(heartbeat) else "Disarmed"
            print(f"Vehicle is {armed_status}")
        time.sleep(1)  # Delay to limit the rate of messages
except KeyboardInterrupt:
    print("Stopped checking vehicle status.")
