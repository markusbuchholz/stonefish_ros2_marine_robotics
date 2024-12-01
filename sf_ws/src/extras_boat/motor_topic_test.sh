#!/bin/bash

# function to publish PWM values
publish_pwm() {
    local port=$1
    local stbd=$2
    ros2 topic pub /blueboat/motors_pwm/cmd_pwm std_msgs/String "data: 'port:$port,stbd:$stbd'" -1
}


for pwm in $(seq 0 10 200); do
    publish_pwm $pwm $pwm
    sleep 3
done


for pwm in $(seq 200 -10 0); do
    publish_pwm $pwm $pwm
    sleep 3
done


for pwm in $(seq 0 10 200); do
    publish_pwm $pwm 0
    sleep 3
done


for pwm in $(seq 200 -10 0); do
    publish_pwm $pwm 0
    sleep 3
done


for pwm in $(seq 0 10 200); do
    publish_pwm 0 $pwm
    sleep 3
done


for pwm in $(seq 200 -10 0); do
    publish_pwm 0 $pwm
    sleep 3
done
