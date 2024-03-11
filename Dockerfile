FROM nvidia/cuda:11.7.1-cudnn8-devel-ubuntu22.04

CMD ["/bin/bash"]

# Args for setting up non-root users, example command to use your own user:
# docker build -t <name: vtr3> \
#   --build-arg USERID=$(id -u) \
#   --build-arg GROUPID=$(id -g) \
#   --build-arg USERNAME=$(whoami) \
#   --build-arg HOMEDIR=${HOME} \
#   --build-arg CUDA_ARCH=${nvidia-smi --query-gpu=compute_cap --format=csv,noheader} .
ARG GROUPID=0
ARG USERID=0
ARG USERNAME=root
ARG HOMEDIR=/root
ARG CUDA_ARCH="8.6"

RUN if [ ${GROUPID} -ne 0 ]; then addgroup --gid ${GROUPID} ${USERNAME}; fi \
  && if [ ${USERID} -ne 0 ]; then adduser --disabled-password --gecos '' --uid ${USERID} --gid ${GROUPID} ${USERNAME}; fi

# Default number of threads for make build
ARG NUMPROC=12

ENV DEBIAN_FRONTEND=noninteractive

## Switch to specified user to create directories
USER ${USERID}:${GROUPID}

ENV VTRROOT=${HOMEDIR}/ASRL/vtr3
ENV VTRSRC=${VTRROOT}/src \
  VTRDATA=${VTRROOT}/data \
  VTRTEMP=${VTRROOT}/temp \
  VTRMODELS=${VTRROOT}/models \
  GRIZZLY=${VTRROOT}/grizzly \
  WARTHOG=${VTRROOT}/warthog \
  VTRUI=${VTRSRC}/main/src/vtr_gui/vtr_gui/vtr-gui

RUN echo "alias build_ui='npm --prefix ${VTRUI} install ${VTRUI}; npm --prefix ${VTRUI} run build'" >> ~/.bashrc
RUN echo "alias build_vtr='source /opt/ros/humble/setup.bash; cd ${VTRSRC}/main; colcon build --symlink-install'" >> ~/.bashrc

## Switch to root to install dependencies
USER 0:0

## Dependencies
RUN apt update && apt upgrade -q -y
RUN apt update && apt install -q -y cmake git build-essential lsb-release curl gnupg2
RUN apt update && apt install -q -y libboost-all-dev libomp-dev
RUN apt update && apt install -q -y libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev
RUN apt update && apt install -q -y freeglut3-dev
RUN apt update && apt install -q -y python3 python3-distutils python3-pip
RUN apt update && apt install -q -y libeigen3-dev
RUN apt update && apt install -q -y libsqlite3-dev sqlite3
RUN apt install -q -y libc6-dbg gdb valgrind

## Install PROJ (8.2.0) (this is for graph_map_server in vtr_navigation)
RUN apt update && apt install -q -y cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev
RUN mkdir -p ${HOMEDIR}/proj && cd ${HOMEDIR}/proj \
  && git clone https://github.com/OSGeo/PROJ.git . && git checkout 8.2.0 \
  && mkdir -p ${HOMEDIR}/proj/build && cd ${HOMEDIR}/proj/build \
  && cmake .. && cmake --build . -j${NUMPROC} --target install
ENV LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}



## Install ROS2
# UTF-8
RUN apt install -q -y locales \
  && locale-gen en_US en_US.UTF-8 \
  && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8
# Add ROS2 key and install from Debian packages
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg \
  && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null \
  && apt update && apt install -q -y ros-humble-desktop

## Install VTR specific ROS2 dependencies
RUN apt update && apt install -q -y \
  ros-humble-xacro \
  ros-humble-vision-opencv \
  ros-humble-perception-pcl ros-humble-pcl-ros \
  ros-humble-rmw-cyclonedds-cpp

RUN apt install ros-humble-tf2-tools

## Install misc dependencies
RUN apt update && apt install -q -y \
  tmux \
  nodejs npm protobuf-compiler \
  libboost-all-dev libomp-dev \
  libpcl-dev \
  libcanberra-gtk-module libcanberra-gtk3-module \
  libbluetooth-dev libcwiid-dev \
  python3-colcon-common-extensions \
  virtualenv \
  texlive-latex-extra \
  clang-format

## Install python dependencies
RUN pip3 install \
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


#added by sherry

RUN apt install wget
RUN apt install nano


## install opencv 4.5.1

RUN apt install -q -y libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy


RUN mkdir -p ${HOMEDIR}/opencv && cd ${HOMEDIR}/opencv \
&& git clone https://github.com/opencv/opencv.git . 

RUN cd ${HOMEDIR}/opencv && git checkout 4.6.0
RUN mkdir -p ${HOMEDIR}/opencv_contrib && cd ${HOMEDIR}/opencv_contrib \
&& git clone https://github.com/opencv/opencv_contrib.git . 
RUN cd ${HOMEDIR}/opencv_contrib && git checkout 4.6.0 


RUN apt install -q -y build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
# # generate Makefiles (note that install prefix is customized to: /usr/local/opencv_cuda)

RUN mkdir -p ${HOMEDIR}/opencv/build && cd ${HOMEDIR}/opencv/build \
&& cmake -D CMAKE_BUILD_TYPE=RELEASE \
-D CMAKE_INSTALL_PREFIX=/usr/local/opencv_cuda \
-D OPENCV_EXTRA_MODULES_PATH=${HOMEDIR}/opencv_contrib/modules \
-D PYTHON_DEFAULT_EXECUTABLE=/usr/bin/python3.10 \
-DBUILD_opencv_python2=OFF \
-DBUILD_opencv_python3=ON \
-DWITH_OPENMP=ON \
-DWITH_CUDA=ON \
-D CUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-11.7 \
-DOPENCV_ENABLE_NONFREE=ON \
-D OPENCV_GENERATE_PKGCONFIG=ON \
-DWITH_TBB=ON \
-DWITH_GTK=ON \
-DWITH_OPENMP=ON \
-DWITH_FFMPEG=ON \
-DBUILD_opencv_cudacodec=OFF \
-D BUILD_EXAMPLES=OFF \
-D CUDA_ARCH_BIN=$CUDA_ARCH ..  && make -j16 && make install


ENV LD_LIBRARY_PATH=/usr/local/opencv_cuda/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}

RUN mkdir -p ${HOMEDIR}/.matplotcpp && cd ${HOMEDIR}/.matplotcpp \
  && git clone https://github.com/lava/matplotlib-cpp.git . \
  && mkdir -p ${HOMEDIR}/.matplotcpp/build && cd ${HOMEDIR}/.matplotcpp/build \
  && cmake .. && cmake --build . -j${NUMPROC} --target install
  
  
##Install LibTorch
RUN curl https://download.pytorch.org/libtorch/cu117/libtorch-cxx11-abi-shared-with-deps-2.0.0%2Bcu117.zip --output libtorch.zip
RUN unzip libtorch.zip -d /opt/torch
RUN rm libtorch.zip
ENV TORCH_LIB=/opt/torch/libtorch
ENV LD_LIBRARY_PATH=$TORCH_LIB/lib:${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
ENV CMAKE_PREFIX_PATH=$TORCH_LIB:$CMAKE_PREFIX_PATH

ENV NVIDIA_DRIVER_CAPABILITIES compute,utility,graphics

RUN apt install htop

# Install vim
RUN apt update && apt install -q -y vim

## Switch to specified user
USER ${USERID}:${GROUPID}
