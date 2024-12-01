# Markus Buchholz

FROM osrf/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get upgrade -y

RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    bash-completion \
    build-essential \
    git \
    glmark2 \
    gnupg \
    iputils-ping \
    lsb-release \
    plocate \
    software-properties-common \
    sudo \
    wget \
    vim \
    nano \
    libxkbcommon-x11-0 \
    xvfb \
    libncurses5-dev \
    libncursesw5-dev \
    cmake \
    libzmq3-dev \
    libdw-dev \
    qtbase5-dev \
    libqt5svg5-dev \
    libpcl-dev \
  && rm -rf /var/lib/apt/lists/*

# install NVIDIA software
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libglvnd0 \
    libgl1 \
    libglx0 \
    libegl1 \
    libxext6 \
    libx11-6 \
  && rm -rf /var/lib/apt/lists/*
ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=graphics,utility,compute
ENV QT_X11_NO_MITSHM=1

WORKDIR /home/$USERNAME/colcon_ws

ENV DIST=humble

# install ROS 2 control framework and related packages
RUN apt update \
  && apt install -y --no-install-recommends \
    ros-${DIST}-control-toolbox \
    ros-${DIST}-controller-interface \
    ros-${DIST}-controller-manager \
    ros-${DIST}-joint-state-publisher \
    ros-${DIST}-joint-trajectory-controller \
    ros-${DIST}-position-controllers \
    ros-${DIST}-effort-controllers \
    ros-${DIST}-velocity-controllers \
    ros-${DIST}-ros2-control \
    ros-${DIST}-ros2-controllers \
  && apt clean

# install Cyclone DDS
RUN apt update \
  && apt install -y --no-install-recommends \
    ros-${DIST}-rmw-cyclonedds-cpp \
  && apt clean

# Install additional libraries for StoneFish
RUN apt-get update \
 && apt-get -y --quiet --no-install-recommends install \
    libcgal-dev \
    libfftw3-dev \
    libglm-dev \
    libsdl2-dev \
    libfreetype6-dev \
 && rm -rf /var/lib/apt/lists/*

RUN sed -i 's/-lSDL2 /-lSDL2/' /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake

# install OpenCV
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    libopencv-dev \
    python3-opencv \
  && rm -rf /var/lib/apt/lists/*

RUN echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /root/.bashrc \
    && echo "export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" >> /root/.bashrc

# build the project
WORKDIR /home/$USERNAME/colcon_ws
RUN colcon build

# Install prerequisites for ArduPilot
RUN apt-get update && apt-get -y install \
    git \
    python3-dev \
    python3-pip \
    python3-setuptools \
    python3-wheel \
    libtool \
    g++ \
    gcc \
    gawk \
    make \
    pkg-config \
    ccache \
    libncurses5-dev \
    gdb \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a symbolic link for python -> python3
RUN ln -s /usr/bin/python3 /usr/bin/python

# Clone ArduPilot repository and checkout specific commit
WORKDIR /home/$USERNAME
RUN git clone https://github.com/ArduPilot/ardupilot.git
WORKDIR /home/$USERNAME/ardupilot
RUN git checkout aca9940534

# Initialize submodules
RUN git submodule update --init --recursive

# Install necessary Python dependencies, including pexpect
RUN pip3 install pymavlink MAVProxy dronekit pexpect

# Build ArduSub using waf-light
WORKDIR /home/$USERNAME/ardupilot
RUN modules/waf/waf-light configure --board sitl \
  && modules/waf/waf-light build --target bin/ardusub

# Update environment paths
RUN echo "export PATH=/home/$USERNAME/ardupilot/Tools/autotest:\$PATH" >> /root/.bashrc \
    && echo "export PATH=/home/$USERNAME/ardupilot/modules/waf:\$PATH" >> /root/.bashrc

RUN apt-get update && apt-get install -y \
ros-humble-tf-transformations \
&& rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy==1.23.5

RUN pip3 install pybullet

RUN apt update && apt install -y ros-humble-pinocchio

# Install FUSE
RUN apt-get update \
  && apt-get -y --quiet --no-install-recommends install \
    fuse \
  && rm -rf /var/lib/apt/lists/*

# Install BehaviorTree.CPP from the master branch with ZMQ support
RUN git clone https://github.com/BehaviorTree/BehaviorTree.CPP.git /opt/BehaviorTree.CPP \
    && cd /opt/BehaviorTree.CPP \
    && mkdir build \
    && cd build \
    && cmake -DCMAKE_INSTALL_PREFIX=/usr/local -DBTCPP_USE_ZMQ=true .. \
    && make \
    && make install

# Source ROS setup.bash and set environment variables for BehaviorTree.CPP
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
    && echo "export CMAKE_PREFIX_PATH=\$CMAKE_PREFIX_PATH:/usr/local" >> /root/.bashrc \
    && echo "export LD_LIBRARY_PATH=\$LD_LIBRARY_PATH:/usr/local/lib" >> /root/.bashrc \
    && echo "export PKG_CONFIG_PATH=\$PKG_CONFIG_PATH:/usr/local/lib/pkgconfig" >> /root/.bashrc \
    && echo "export PATH=\$PATH:/usr/local/bin" >> /root/.bashrc

RUN apt update \
    && apt install -y --no-install-recommends \
    ros-${DIST}-plotjuggler-ros \
    && apt clean

RUN apt update && apt install -y ros-humble-xacro

RUN apt update && apt install -y ros-humble-joint-state-publisher-gui

RUN apt-get update && apt-get install -y ros-humble-moveit && rm -rf /var/lib/apt/lists/*


# Entry point
CMD ["/bin/bash"]