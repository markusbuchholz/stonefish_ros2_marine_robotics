import time
from pymavlink import mavutil

master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

master.wait_heartbeat()
print("Heartbeat received from system")

while True:
    try:
        msg = master.recv_match(blocking=True)
        if msg is None:
            continue

        msg_type = msg.get_type()

        print("Message Type:", msg_type)

        msg_dict = msg.to_dict()

        msg_dict.pop('mavpackettype', None)

        field_topics = sorted(msg_dict.keys())
        print("Field Topics:", field_topics)

    except Exception as e:
        print("Error:", e)
    time.sleep(0.1)
