FROM nvidia/cuda:11.4.2-devel-ubuntu20.04

CMD ["/bin/bash"]

# Args for setting up non-root users, example command to use your own user:
#   docker build -t <name: vtr3> \
#     --build-arg USERID=$(id -u) \
#     --build-arg GROUPID=$(id -g) \
#     --build-arg USERNAME=$(whoami) \
#     --build-arg HOMEDIR=${HOME} .
ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

# Default number of threads for make build
ARG NUMPROC=12

ENV DEBIAN_FRONTEND=noninteractive

## Switch to specified user to create directories
USER ${USERID}:${GROUPID}

ENV VTRROOT=${HOMEDIR}/ASRL
ENV VTRSRC=${VTRROOT}/vtr3 \
  VTRDEPS=${VTRROOT}/deps \
  VTRVENV=${VTRROOT}/venv \
  VTRDATA=${VTRROOT}/data \
  VTRTEMP=${VTRROOT}/temp
RUN mkdir -p ${VTRROOT} ${VTRSRC} ${VTRDEPS} ${VTRDATA} ${VTRVENV} ${VTRTEMP}

## Switch to root to install dependencies
USER 0:0

## Common packages
RUN apt update && apt install -q -y wget git

## Install Eigen
RUN apt update && apt install -q -y libeigen3-dev

## Install PROJ (8.0.0) (this is for graph_map_server in vtr_navigation)
RUN apt update && apt install -q -y cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev
RUN mkdir -p ${VTRDEPS}/proj && cd ${VTRDEPS}/proj \
  && git clone https://github.com/OSGeo/PROJ.git . && git checkout 8.0.0 \
  && mkdir -p ${VTRDEPS}/proj/build && cd ${VTRDEPS}/proj/build \
  && cmake .. && cmake --build . -j${NUMPROC} --target install
ENV LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

## Install Google Ceres (this is for teb local planner, to be removed)
RUN apt update && apt install -q -y libgoogle-glog-dev libgflags-dev libatlas-base-dev libeigen3-dev libsuitesparse-dev
RUN mkdir -p ${VTRDEPS}/ceres && cd ${VTRDEPS}/ceres \
  && git clone https://ceres-solver.googlesource.com/ceres-solver . && git checkout 2.0.0 \
  && mkdir -p ${VTRDEPS}/ceres/build && cd ${VTRDEPS}/ceres/build \
  && cmake .. && cmake --build . -j${NUMPROC} --target install

## Install g2o (this is for teb local planner, to be removed)
# for now use master branch, in case of failure, verified working commit: 4736df5ca8ef258caa47ae0de9b59bc22fb80d1b
RUN apt update && apt install -q -y libsuitesparse-dev qtdeclarative5-dev qt5-qmake libqglviewer-dev-qt5
RUN mkdir -p ${VTRDEPS}/g2o && cd ${VTRDEPS}/g2o \
  && git clone https://github.com/RainerKuemmerle/g2o.git . \
  && mkdir -p ${VTRDEPS}/g2o/build && cd ${VTRDEPS}/g2o/build \
  && cmake -DBUILD_WITH_MARCH_NATIVE=ON .. && cmake --build . -j${NUMPROC} --target install

## Install ROS2
# UTF-8
RUN apt install -q -y locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
# Add ROS2 key and install from Debian packages
RUN apt install -q -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt update && apt install -q -y ros-galactic-desktop

## Install misc dependencies
RUN apt update && apt install -q -y \
  tmux \
  doxygen \
  nodejs npm protobuf-compiler \
  libboost-all-dev libomp-dev \
  libpcl-dev \
  libcanberra-gtk-module libcanberra-gtk3-module \
  libdc1394-22 libdc1394-22-dev \
  libbluetooth-dev libcwiid-dev \
  python3-colcon-common-extensions \
  virtualenv

## Create a python virtual environment
RUN apt install -q -y python3-pip && pip3 install \
  tmuxp \
  pyyaml \
  pyproj \
  scipy \
  zmq \
  flask \
  flask_socketio \
  eventlet \
  python-socketio \
  python-socketio[client] \
  websocket-client

## Install VTR specific ROS2 dependencies
RUN apt update && apt install -q -y \
  ros-galactic-xacro \
  ros-galactic-vision-opencv \
  ros-galactic-perception-pcl ros-galactic-pcl-ros

## These are for teb local planner (to be removed)
RUN apt update && apt install -q -y \
  ros-galactic-nav2-costmap-2d \
  ros-galactic-libg2o \
  ros-galactic-dwb-critics \
  ros-galactic-nav2-core \
  ros-galactic-nav2-msgs \
  ros-galactic-nav2-util \
  ros-galactic-nav2-bringup

## Switch to specified user
USER ${USERID}:${GROUPID}