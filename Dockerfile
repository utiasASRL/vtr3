FROM nvidia/cuda:11.4.2-devel-ubuntu20.04

CMD ["/bin/bash"]

# Args for setting up non-root users, example command to use your own user:
#   docker build -t <name: vtr3> \
#     --build-arg USERID=$(id -u) \
#     --build-arg GROUPID=$(id -g) \
#     --build-arg USERNAME=$(whoami) \
#     --build-arg HOMEDIR=/home .
ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

# Default number of threads for make build
ARG NUMPROC=12

ENV DEBIAN_FRONTEND=noninteractive

ENV VTRROOT=${HOMEDIR}/${USERNAME}/ASRL
ENV VTRSRC=${VTRROOT}/vtr3 \
  VTRDEPS=${VTRROOT}/deps \
  VTRVENV=${VTRROOT}/venv \
  VTRDATA=${VTRROOT}/data \
  VTRTEMP=${VTRROOT}/temp
RUN mkdir -p ${VTRROOT} ${VTRSRC} ${VTRDEPS} ${VTRDATA} ${VTRVENV} ${VTRTEMP}

## Common packages
RUN apt update && apt install -q -y wget git

## Install Eigen
RUN apt update && apt install -q -y libeigen3-dev

## Install PROJ (8.0.0)
RUN apt update && apt install -q -y cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev
RUN mkdir -p ${VTRDEPS}/proj && cd ${VTRDEPS}/proj \
  && git clone https://github.com/OSGeo/PROJ.git . && git checkout 8.0.0 \
  && mkdir -p ${VTRDEPS}/proj/build && cd ${VTRDEPS}/proj/build \
  && cmake .. && cmake --build . -j${NUMPROC} --target install
ENV LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

## Install OpenCV (4.5.0)
RUN apt install -q -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
RUN cd ${VTRDEPS} \
  && git clone https://github.com/opencv/opencv.git \
  && git clone https://github.com/opencv/opencv_contrib.git \
  && cd ${VTRDEPS}/opencv && git checkout 4.5.0 \
  && cd ${VTRDEPS}/opencv_contrib && git checkout 4.5.0 \
  && mkdir -p ${VTRDEPS}/opencv/build && cd ${VTRDEPS}/opencv/build \
  && cmake -D CMAKE_BUILD_TYPE=RELEASE \
  -D CMAKE_INSTALL_PREFIX=/usr/local/opencv_cuda \
  -D OPENCV_EXTRA_MODULES_PATH=${VTRDEPS}/opencv_contrib/modules \
  -D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3.8 \
  -DBUILD_opencv_python2=OFF \
  -DBUILD_opencv_python3=ON \
  -DWITH_OPENMP=ON \
  -DWITH_CUDA=ON \
  -DOPENCV_ENABLE_NONFREE=ON \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -DWITH_TBB=ON \
  -DWITH_GTK=ON \
  -DWITH_OPENMP=ON \
  -DWITH_FFMPEG=ON \
  -DBUILD_opencv_cudacodec=OFF \
  -D BUILD_EXAMPLES=ON .. \
  && make -j${NUMPROC} && make install

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
  && apt update && apt install -q -y ros-galactic-desktop ros-galactic-test-msgs

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

## Switch to specified user
USER ${USERID}:${GROUPID}