#!/usr/bin/env python
import rospy
from mavros_msgs.msg import OverrideRCIn
from time import sleep

# Initialize the ROS node
rospy.init_node('rc_override_control', anonymous=True)
pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

# Function to publish RC commands
def publish_rc_command(channels, duration):
    msg = OverrideRCIn()
    msg.channels = channels
    pub.publish(msg)
    sleep(duration)
    # Stop the movement by setting all channels to 0
    msg.channels = [0] * 18
    pub.publish(msg)

# Movement functions
def move_forward():
    publish_rc_command([1550, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 1.0)
    publish_rc_command([1540, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 4.0)

def move_right():
    publish_rc_command([1540, 0, 1540, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 5.0)

def move_left():
    publish_rc_command([1540, 0, 1450, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 5.0)

def move_back():
    publish_rc_command([1450, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 4.0)

def stop_movement():
    publish_rc_command([1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], 8.0)

if __name__ == '__main__':
    try:
        while not rospy.is_shutdown():
            cmd = input("Enter command (1: Forward, 2: Left, 3: Right, 4: Back, 0: Stop, q: Quit): ")
            if cmd == 'q':
                print("Exiting...")
                break
            elif cmd == '1':
                move_forward()
            elif cmd == '2':
                move_left()
            elif cmd == '3':
                move_right()
            elif cmd == '4':
                move_back()
            elif cmd == '0':
                stop_movement()
            else:
                print("Invalid command:", cmd)
    except rospy.ROSInterruptException:
        pass





















# catkin_create_pkg boat_control rospy mavros_msgs