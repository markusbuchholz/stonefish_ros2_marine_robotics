# StoneFish ROS 2 Marine Robotics Simulator

![image](https://github.com/user-attachments/assets/5654ddf9-6532-4610-aca9-8ed265ed5d94)


## Overview

The repository provides a Docker container with a bash scripts that can seamlessly build and run the container. <br>

Docker container offers ROS 2 environment whare you can launch vehicles in [StoneFish](https://stonefish.readthedocs.io/en/latest/).<br>

The repository provides all necessary commands and references that can boost your software's research and development process, which utilizes the ROS 2 stack.<br>

SITL allows you to simulate the vehicle hardware and firmware [ArduSub](https://www.ardusub.com/) on your host directly.<br>

There are several scripts in: ```/extras_rov```, ```/extras_boat``` to communicate with vehicle using [Pymavlink](https://www.ardusub.com/developers/pymavlink.html).<br>

There is a simple ROS 2 interface to ArduPilot (only for BlueROV2). Check ```/extras_interface```. <br>


## Prerequisites

- Install [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html) to support Docker to access GPU (required).
- Repository has been tested on: Ubuntu 22.04, Ubuntu 24.04, ArchLinux (Kernel 6.8).


## Notes

Adjust paths in ```run.sh```.

```bash
local_workspace="/home/markus/underwater/stonefish_ros2_marine_robotics/src"
local_stonefish="/home/markus/underwater/stonefish_ros2_marine_robotics/stonefish"
local_sf_ws="/home/markus/underwater/stonefish_ros2_marine_robotics/sf_ws"
local_gz_ws="/home/markus/underwater/stonefish_ros2_marine_robotics/gz_ws"
local_SITL_Models="/home/markus//underwater/stonefish_ros2_marine_robotics/SITL_Models"
```



## Build Docker

```bash

git clone https://github.com/markusbuchholz/stonefish_ros2_marine_robotics.git

cd /stonefish_ros2_marine_robotics/docker

sudo ./build.sh
```

## Run Docker


```bash

sudo ./run.sh

cd ../stonefish

# IMPORTANT. You build only once since the stonefish volume is on your HOST.
mkdir -p build && cd build

cmake ..

make -j$(nproc)

# IMPORTANT. You have to run these two commands every time you run Docker container.
# 1.
sudo make install
# 2.
export LD_LIBRARY_PATH=/home/stonefish/build/libStonefish.so:$LD_LIBRARY_PATH

cd ../../sf_ws/

colcon build --packages-select stonefish_ros2
source install/setup.bash

colcon build --packages-select cola2_msgs
source install/setup.bash

colcon build
source install/setup.bash

```

---
Connect to running Docker container,

```bash
sudo docker exec -it ros2sim /bin/bash

```
---

## Start StoneFish with BlueROV2

 ```bash
ros2 launch cola2_stonefish bluerov_fls_simulation.launch.py
```

## Start StoneFish with BlueBoat

 ```bash
ros2 launch cola2_stonefish blueboat_launch.py
```

## Start StoneFish with BlueROV2 and ArduPilot

```bash
#terminal 1
ros2 run stonefish_bluerov2 ardusim_patch.py

#terminal 2
ros2 launch stonefish_bluerov2 bluerov2_sim.launch.py

#terminal 3
sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON --map  -l 55.99541530863445,-3.3010225004910683,0,0 -m --streamrate=-1

#terminal 4
cd /home/sf_ws/src/extras_rov
# run example script
# check for other scripts
python3 pos_req.py

```

Check BlueROV2 abilities using ```Mavlink```(fetch status information from ArduSub), <br>

```bash
cd /home/sf_ws/src/extras_rov
python3 print_abilities.py
```

## Thrust (PWM) Control Mode

Setting the ArduSub [RCPassThru](https://ardupilot.org/copter/docs/parameters.html#servo1-function-servo-output-function) parameter allows setting PWM directly for individual motors with values ranging from 1100 to 1900. <br>

BlueRobotics T200 thruster [specification](https://bluerobotics.com/store/thrusters/t100-t200-thrusters/t200-thruster-r2-rp/#:~:text=The%20T200%20propeller%20has%20a,and%203600%20RPM%2016%20V.).

Before use, Passthrough mode must be enabled via a service; the ROS 2 Interface for StonFish does this automatically.<br>


![image](https://github.com/user-attachments/assets/6e98db5a-1ded-4dca-89e7-a998c48100ae)


**Complete command pipeline**

```bash
#terminal 1
ros2 run stonefish_bluerov2 ardusim_patch.py

#terminal 2
ros2 launch stonefish_bluerov2 bluerov2_sim.launch.py

#terminal 3
sim_vehicle.py -v ArduSub -f vectored_6dof --model JSON --map  -l 55.99541530863445,-3.3010225004910683,0,0 -m --streamrate=-1

#terminal 4 
# Run ROS 2 interface for StonFish
cd /home/sf_ws/src/extras_interface

python3 ros2_simple_bluerov2_interface.py
```

The motor can be controlled using the topics. 

Example usage,

```bash
# motor 2
ros2 topic pub /bluerov2/thruster_2_pwm_set std_msgs/msg/Int32 "{data: 1600}"

# motor 7
ros2 topic pub /bluerov2/thruster_7_pwm_set std_msgs/msg/Int32 "{data: 1640}"
```


## Start StoneFish with BlueBoat and ArduPilot

```bash
#terminal 1
ros2 run stonefish_bluerov2 ardusim_patch_boat.py

#terminal 2
ros2 launch stonefish_bluerov2 blueboat_sim.launch.py

#terminal 3
sim_vehicle.py -v Rover -f boat --model JSON --map --console -l 55.99541530863445,-3.3010225004910683,0,0

#terminal 4
cd /home/sf_ws/src/extras_boat
# run example script
# check for other scripts
python3 python3 check.py

```
ROS 2 interface provides two common topics (ROS 2 is a function wrapper for Mavlink protocol):

```bash
cd /home/sf_ws/src/extras_boat
python3 python3 ros2_simple_blueboat_interface.py
```

```bash
/bluerov2/odometry
/bluerov2/cmd_vel
/bluerov2/servo_outputs
```

Run simple motion directly to ArduRover,

```bash
cd /home/sf_ws/src/extras_boat
python3 pwm_control.py
```


## Start StoneFish with BlueBoat, ArduPilot and simple ROS 2 interface

ROS 2 interface provides two common topics (ROS 2 is a function wrapper for Mavlink protocol):

```bash
/bluerov2/odometry
/bluerov2/cmd_vel
/bluerov2/servo_outputs
```

Run the motors individually.

Note. To be FIXED by fixing ```ardusim_patch``` (it works in GazeboSim),

```bash
ros2 topic pub /blueboat/send_port_motor_0_100_thrust std_msgs/msg/Float32 "{data: 50.0}"
ros2 topic pub /blueboat/send_stbd_motor_0_100_thrust std_msgs/msg/Float32 "{data: 75.0}"
```

```bash
#terminal 1
ros2 run stonefish_bluerov2 ardusim_patch_ros2_interface.py 

#terminal 2
ros2 launch stonefish_bluerov2 bluerov2_sim_ros2_interface.launch.py

#terminal 3
sim_vehicle.py -v Rover -f boat --model JSON --map --console -l 55.99541530863445,-3.3010225004910683,0,0

#terminal 4
cd /home/sf_ws/src/extras_interface
python3 ros2_simple_blueboat_interface
```

Move BlueROV2,

```bash
ros2 topic pub -r 10 /bluerov2/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 0.0
  y: 0.0
  z: -2.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" --qos-reliability reliable --qos-durability transient_local --qos-depth 10 -1
```


The PWM values:  ```/bluerov2/servo_outputs``` during robot motion can be simply saved,

```bash
cd /home/sf_ws/src/extras_interface
python3 ros2_pwm_topic_recorder.py
```



## Links

- [StoneFish github](https://github.com/patrykcieslak/stonefish)
- [ArduSub](https://www.ardusub.com/)
- [Pymavlink](https://www.ardusub.com/developers/pymavlink.html)
- [Stonefish_bluerov2](https://github.com/bvibhav/stonefish_bluerov2)
- [StoneFish paper](https://ieeexplore.ieee.org/document/8867434)








