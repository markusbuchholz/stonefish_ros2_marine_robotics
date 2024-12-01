#!/bin/bash

# Function to process and display the data
process_data() {
    echo "Timestamp: ${secs} secs, ${nsecs} nsecs"
    echo "Orientation - x: ${orient_x}, y: ${orient_y}, z: ${orient_z}, w: ${orient_w}"
    echo "Angular Velocity - x: ${ang_vel_x}, y: ${ang_vel_y}, z: ${ang_vel_z}"
    echo "Linear Acceleration - x: ${lin_acc_x}, y: ${lin_acc_y}, z: ${lin_acc_z}"
    echo "--------------------------------"
}

# Initialize variables
secs=0
nsecs=0
orient_x=0
orient_y=0
orient_z=0
orient_w=0
ang_vel_x=0
ang_vel_y=0
ang_vel_z=0
lin_acc_x=0
lin_acc_y=0
lin_acc_z=0

# Subscribe to the /bluerov2/imu topic and process the data
rostopic echo /bluerov2/imu | while read line; do
    case "$line" in
        *"stamp:"*)
            read line
            secs=$(echo $line | awk '{print $2}')
            read line
            nsecs=$(echo $line | awk '{print $2}')
            ;;
        *"orientation:"*)
            read line
            orient_x=$(echo $line | awk '{print $2}')
            read line
            orient_y=$(echo $line | awk '{print $2}')
            read line
            orient_z=$(echo $line | awk '{print $2}')
            read line
            orient_w=$(echo $line | awk '{print $2}')
            ;;
        *"angular_velocity:"*)
            read line
            ang_vel_x=$(echo $line | awk '{print $2}')
            read line
            ang_vel_y=$(echo $line | awk '{print $2}')
            read line
            ang_vel_z=$(echo $line | awk '{print $2}')
            ;;
        *"linear_acceleration:"*)
            read line
            lin_acc_x=$(echo $line | awk '{print $2}')
            read line
            lin_acc_y=$(echo $line | awk '{print $2}')
            read line
            lin_acc_z=$(echo $line | awk '{print $2}')
            process_data
            ;;
    esac
done
