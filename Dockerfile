FROM nvidia/cuda:11.3.1-devel-ubuntu20.04

CMD ["/bin/bash"]

# Default number of threads for make build
ARG NUMPROC=12

ENV DEBIAN_FRONTEND=noninteractive

ENV VTRROOT=/workspace
ENV VTRSRC=${VTRROOT}/vtr3
ENV VTRDEPS=${VTRROOT}/deps
ENV VTRDATA=${VTRROOT}/data
ENV VTRVENV=${VTRROOT}/venv
ENV VTRTEMP=${VTRROOT}/temp
RUN mkdir -p ${VTRROOT} ${VTRSRC} ${VTRDEPS} ${VTRDATA} ${VTRVENV} ${VTRTEMP}

RUN apt update

## Common packages
RUN apt install -q -y wget

## Change default python version to python3
RUN apt install -q -y python-is-python3

## Install Eigen
RUN apt install -q -y libeigen3-dev

## Install PROJ
RUN apt install -q -y cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev
RUN cd ${VTRDEPS} \
&& PROJ=proj-8.1.0 \
&& wget https://download.osgeo.org/proj/${PROJ}.tar.gz \
&& tar -xf ${PROJ}.tar.gz \
&& mkdir -p ${PROJ}/build && cd ${PROJ}/build \
&& cmake .. && cmake --build . -j${NUMPROC} --target install
ENV LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

## Install OpenCV
RUN apt install -q -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
RUN cd ${VTRDEPS} \
&& git clone https://github.com/opencv/opencv.git \
&& git clone https://github.com/opencv/opencv_contrib.git \
&& cd ${VTRDEPS}/opencv && git checkout 4.5.0 \
&& cd ${VTRDEPS}/opencv_contrib && git checkout 4.5.0 \
&& mkdir -p ${VTRDEPS}/opencv/build && cd ${VTRDEPS}/opencv/build \
&& cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
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

## Install ROS2 Foxy
# UTF-8
RUN apt install -q -y locales \
&& locale-gen en_US en_US.UTF-8 \
&& update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
# Add ROS2 key
RUN apt install -q -y curl gnupg2 lsb-release
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
&& echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
&& apt update
# ROS2 dependencies
RUN apt install -q -y \
  build-essential \
  cmake \
  git \
  wget \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
&& python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest \
&& apt install --no-install-recommends -q -y \
  libasio-dev \
  libtinyxml2-dev \
&& apt install --no-install-recommends -q -y \
  libcunit1-dev
# Get ROS2 source and install
RUN mkdir -p ${VTRDEPS}/ros_foxy && cd ${VTRDEPS}/ros_foxy \
&& wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos \
&& mkdir -p src \
&& vcs import --retry 100 src < ros2.repos \
&& rosdep init && rosdep update \
&& rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers python3-opencv libopencv-dev" \
&& colcon build --symlink-install --packages-skip ros1_bridge

## Install misc dependencies
RUN apt install -q -y \
  doxygen \
  libdc1394-22 libdc1394-22-dev \
  nodejs npm protobuf-compiler \
  libboost-all-dev \
  libomp-dev \
  libpcl-dev \
  python3-virtualenv

## Create a python virtual environment
RUN cd ${VTRVENV} \
&& virtualenv . --system-site-packages \
&& . ${VTRVENV}/bin/activate \
&& pip install \
  pyyaml \
  pyproj \
  scipy \
  zmq \
  flask \
  python-socketio \
  flask_socketio \
  eventlet \
  python-socketio \
  python-socketio[client]

## Install VTR specific ROS2 dependencies
RUN mkdir -p ${VTRDEPS}/vtr_ros2_deps && DEPSROOT=${VTRDEPS}/vtr_ros2_deps \
# vision opencv
&& mkdir -p ${DEPSROOT}/ros2_vision_opencv && cd ${DEPSROOT}/ros2_vision_opencv \
&& git clone https://github.com/ros-perception/vision_opencv.git . && git checkout ros2 \
# xacro
&& mkdir -p ${DEPSROOT}/ros2_xacro && cd ${DEPSROOT}/ros2_xacro \
&& git clone https://github.com/ros/xacro.git . && git checkout 2.0.3 \
# ros2_pcl_msgs (for lidar)
&& mkdir -p ${DEPSROOT}/ros2_pcl_msgs && cd ${DEPSROOT}/ros2_pcl_msgs \
&& git clone https://github.com/ros-perception/pcl_msgs.git . && git checkout ros2 \
# ros2_perception (for lidar)
&& mkdir -p ${DEPSROOT}/ros2_perception_pcl && cd ${DEPSROOT}/ros2_perception_pcl \
&& git clone https://github.com/ros-perception/perception_pcl.git . && git checkout 2.2.0 \
# joystick drivers (for grizzly control)
# && mkdir -p ${DEPSROOT}/joystick_drivers && cd ${DEPSROOT}/joystick_drivers \
# && git clone git@github.com:ros-drivers/joystick_drivers.git. && git checkout ros2 \
# && touch joy_linux/COLCON_IGNORE \
# && touch spacenav/COLCON_IGNORE \
# for robots at UTIAS
# && mkdir -p ${DEPSROOT}/robots && cd ${DEPSROOT}/robots \
# && git clone git@github.com:utiasASRL/robots.git . && git checkout ros2 \
# && touch asrl__lancaster/COLCON_IGNORE \
# && touch asrl__dji/COLCON_IGNORE \
# install all
&& cd ${DEPSROOT} \
&& . ${VTRVENV}/bin/activate \
&& . ${VTRDEPS}/ros_foxy/install/setup.sh \
&& colcon build --symlink-install

# Upgrade npm version for UI
RUN npm install -g npm@7.19.1