

from pymavlink import mavutil
import time
import random
import numpy as np

# Create a MAVLink connection (adjust connection string as necessary)
master = mavutil.mavlink_connection('udpin:192.168.2.1:14550')
master.wait_heartbeat()  # Wait for a heartbeat to ensure connectivity


def set_default_home_position():
    """
    Sets the default home position for the UAV.
    
    Parameters:
    conn - MAVLink connection instance
    home_lat - Latitude of the home position
    home_lon - Longitude of the home position
    home_alt - Altitude of the home position
    """
    x, y, z = 0, 0, 0  # Local frame position offsets
    q = [1, 0, 0, 0]   # Quaternion representing no rotation
    home_lat = int(0.1234 * 1e7)  # Convert to MAVLink GPS format
    home_lon = int(-0.1234 * 1e7)
    home_alt = 10000  # Altitude in millimeters

    approach_x, approach_y, approach_z = 0, 0, 1  # Approach vector, usually pointing upwards

    master.mav.set_home_position_send(
        master.target_system,  # target_system: System ID
        home_lat, 
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )


def send_global_vision_position_estimate(x, y, z, roll, pitch, yaw, covariance):
    current_time_us = int(time.time() * 1e6)  # Unix time in microseconds

    print (" : ", x, " : ", y," : ", z, " : ",roll," : ", pitch," : ", yaw)
    master.mav.vision_position_estimate_send(
        current_time_us,  # Timestamp (microseconds since UNIX epoch)
        x,                # Global X position
        y,                # Global Y position
        z,                # Global Z position
        roll,             # Roll angle in radians
        pitch,            # Pitch angle in radians
        yaw,              # Yaw angle in radians
        covariance        # Covariance matrix upper right triangular (first six rows of 6x6 matrix)
    )

def continuously_send():
    # Initial position and attitude
    x, y, z = 0.0, 0.0, -5.0
    roll, pitch, yaw = 0.0, 0.0, 1.57
    set_default_home_position()
    set_default_home_position()

    while True:
        # Update position and orientation with random fluctuations
        x += random.uniform(-1.05, 1.05)  # Increment/decrement within +/- 5 cm
        y += random.uniform(-1.05, 1.05)
        z += random.uniform(-1.05, 1.05)
        roll += random.uniform(-1.001, 1.001)  # Small radian fluctuation
        pitch += random.uniform(-1.001, 1.001)
        yaw += random.uniform(-1.001, 1.001)

        # Covariance (simulated as a simple scaled identity matrix for demonstration)
        tracker_confidence = 3  # Simulate a tracking confidence level (1-3)
        cov_scale = pow(10, 3 - tracker_confidence)
        covariance = [0.01 * cov_scale] * 21  # Simplified diagonal covariance

        send_global_vision_position_estimate(x, y, z, roll, pitch, yaw, covariance)
        time.sleep(3.1)  # Send every 100 ms

if __name__ == '__main__':
    continuously_send()

