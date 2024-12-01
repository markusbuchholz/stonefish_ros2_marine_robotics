#!/usr/bin/env bash

# X11 authorization
XAUTH=/tmp/.docker.xauth
if [ ! -f $XAUTH ]; then
    xauth_list=$(xauth nlist $DISPLAY)
    xauth_list=$(sed -e 's/^..../ffff/' <<< "$xauth_list")
    if [ ! -z "$xauth_list" ]; then
        echo "$xauth_list" | xauth -f $XAUTH nmerge -
    else
        touch $XAUTH
    fi
    chmod a+r $XAUTH
fi


local_workspace="/home/markus/underwater/ros2_marine_robotics_sim_env/src"
local_stonefish="/home/markus//underwater/ros2_marine_robotics_sim_env/stonefish"
local_sf_ws="/home/markus//underwater/ros2_marine_robotics_sim_env/sf_ws"
local_gz_ws="/home/markus//underwater/ros2_marine_robotics_sim_env/gz_ws"
local_SITL_Models="/home/markus//underwater/ros2_marine_robotics_sim_env/SITL_Models"


export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="$USER_HOME/ros2_jazzy/cyclonedds.xml"
export ROS_DOMAIN_ID=0 

#-e RMW_IMPLEMENTATION=$RMW_IMPLEMENTATION \

docker run -it \
    --rm \
    --name ros2sim \
    -e DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e XAUTHORITY=$XAUTH \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID=$ROS_DOMAIN_ID \
    -v "$XAUTH:$XAUTH" \
    -v "/tmp/.X11-unix:/tmp/.X11-unix" \
    -v "/etc/localtime:/etc/localtime:ro" \
    -v "/dev/input:/dev/input" \
    -v "$local_workspace:/home/colcon_ws/src" \
    -v "$local_sf_ws:/home/sf_ws" \
    -v "$local_stonefish:/home/stonefish" \
    -v "$local_gz_ws:/home/gz_ws" \
    -v "$local_SITL_Models:/home/SITL_Models" \
    --privileged \
    --security-opt seccomp=unconfined \
    --gpus all \
    ros2sim:latest
