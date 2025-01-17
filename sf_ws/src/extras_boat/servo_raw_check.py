#!/usr/bin/env python3

"""
Example of how to connect pymavlink to an autopilot via a UDP connection
and print only SERVO_OUTPUT_RAW messages (i.e., thruster/servo outputs).
"""

import time
from pymavlink import mavutil

# Create the connection (listening for packets on UDP port 14550)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for the first heartbeat so we know the system ID
master.wait_heartbeat()
print("Heartbeat received from system (system %u component %u)" %
      (master.target_system, master.target_component))

# Continuously read and filter only SERVO_OUTPUT_RAW messages
while True:
    try:
        msg = master.recv_match()
        if msg is not None:
            if msg.get_type() == 'SERVO_OUTPUT_RAW':
                # Print the entire dictionary; includes fields like servo1_raw, servo2_raw, ...
                print(msg.to_dict())
    except Exception as e:
        # Uncomment for debugging
        # print(f"Error receiving message: {e}")
        pass

    time.sleep(0.1)
