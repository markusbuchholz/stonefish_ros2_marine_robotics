#!/bin/bash
##
# Function for each movement to avoid code repetition
move_forward() {
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1550, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 1.0
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1540, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 4
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
}

move_right() {
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1540, 0, 1540, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 5
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
}

move_left() {
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1540, 0, 1450, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 5
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
}

move_back() {
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1450, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 4
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
}

stop_movement() {
  rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn "channels: [1500, 0, 1500, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]" -r 10 &
  sleep 8
  pkill -f "rostopic pub /mavros/rc/override mavros_msgs/OverrideRCIn"
}



while true; do
  echo "Enter command (1: Forward, 2: Left, 3: Right, 4: Back, 0: Stop, q: Quit):"
  read -r cmd

  # Quit option
  if [ "$cmd" = "q" ]; then
    echo "Exiting..."
    exit 0
  fi

  # Process command
  case $cmd in
    1)
      move_forward
      ;;
    2)
      move_left
      ;;
    3)
      move_right
      ;;
    4)
      move_back
      ;;
    0)
      stop_movement
      ;;
    *)
      echo "Invalid command: $cmd"
      ;;
  esac
done


#./control_robot.sh 1 2 1 3 4 0
