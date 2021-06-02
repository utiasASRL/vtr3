# VT&amp;R3

Visual Teach &amp; Repeat 3

- [VT&amp;R3](#vtr3)
  - [Installation](#installation)
    - [Hardware and Software Requirements](#hardware-and-software-requirements)
    - [Install Ubuntu 20.04](#install-ubuntu-2004)
    - [Directory Structure Overview](#directory-structure-overview)
    - [Install CUDA (>=11.2)](#install-cuda-112)
    - [Change default python version to python3](#change-default-python-version-to-python3)
    - [Install Eigen (>=3.3.7)](#install-eigen-337)
    - [Install PROJ (>=8.0.0)](#install-proj-800)
    - [Install OpenCV (>=4.5.0)](#install-opencv-450)
    - [Install ROS2 Foxy](#install-ros2-foxy)
    - [Install python dependencies](#install-python-dependencies)
    - [Install javascript dependencies](#install-javascript-dependencies)
    - [Install VTR3](#install-vtr3)
  - [Launch VTR3](#launch-vtr3)
    - [Offline (Playback) Mode](#offline-playback-mode)
      - [Stereo SURF-Feature-Based T&R](#stereo-surf-feature-based-tr)
      - [LiDAR Point-Cloud-Based T&R](#lidar-point-cloud-based-tr)
    - [Online (Grizzly) Mode](#online-grizzly-mode)
    - [Testing & Development Mode](#testing--development-mode)
  - [VTR Datasets](#vtr-datasets)
  - [Documentation](#documentation)
    - [Conceptual design document](#conceptual-design-document)
    - [Mid-level documentation](#mid-level-documentation)
    - [In-source documentation](#in-source-documentation)
  - [Contributing](#contributing)
  - [License](#license)

## Installation

### Hardware and Software Requirements

Assume Lenovo P53 laptops, but technically any computer with an Nvidia GPU.

Software requirements listed below. No guarantee for VTR to work when there is any version mismatch.

### Install [Ubuntu 20.04](https://ubuntu.com/)

Install Ubuntu from [official website](https://ubuntu.com/).

- Note: For dual boot system, remember to DISABLE [device encryption](https://support.microsoft.com/en-ca/help/4028713/windows-10-turn-on-device-encryption) before start installing Ubuntu.

- Make sure your system packages are up to date:

  ```bash
  sudo apt-get update
  sudo apt-get upgrade
  ```

### Directory Structure Overview

The follow environment variables are assumed to be present so that files and data can be put into different locations on different computers. Values of these variables can be changed. Recommended to put them in bashrc.

```bash
export VTRROOT=~/ASRL  # root directory of VTR
export VTRSRC=${VTRROOT}/vtr3  # directory containing source code of VTR
export VTRDEPS=${VTRROOT}/workspace  # directory containing dependencies of VTR
export VTRDATA=${VTRROOT}/data  # datasets for VTR
export VTRVENV=${VTRROOT}/venv  # python virtual environment
```

If the values above are used, the final directory structure should look like this:

```text
|- ~/ASRL
  |- vtr3              VTR3 source code and installation
  |- venv              python virtual environment
  |- workspace         system dependencies source code and (maybe) installation
    |- opencv          opencv source code cloned from github, installed to /usr/local/[lib,bin]
    |- opencv_contrib  extra opencv source code cloned from github, installed together with opencv
    |- proj-<version>  the newest version of PROJ, which is required by VT&R
    |- ros_foxy        source code and installation of ROS2 on Ubuntu 20.04
    |- vtr_ros2_deps   VTR dependencies from public repositories without modification
  |- data              datasets for VTR
```

### Install [CUDA](https://developer.nvidia.com/cuda-toolkit) (>=11.2)

Install CUDA through Debian package manager (the network version) from its [official website](https://developer.nvidia.com/cuda-toolkit). Be sure to perform the necessary [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index).

You can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`. It is possible that these two commands report different CUDA version, which means that your CUDA driver and toolkit version do not match. This is OK as long as the driver and toolkit are compatible, which you can verify in the documentation.

### Change default python version to python3

Use python3 to install everything, but some packages default to use python from `/usr/bin/python` which is python2.

```bash
sudo apt install python-is-python3
```

### Install Eigen (>=3.3.7)

```bash
sudo apt install libeigen3-dev
```

### Install [PROJ](https://proj.org/) (>=8.0.0)

The instructions below follow the installation instructions [here](https://proj.org/install.html#compilation-and-installation-from-source-code). Download the [latest release](https://proj.org/download.html#current-release) first and extract it in to `${VTRDEPS}`

```bash
sudo apt install cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev # dependencies
mkdir ${VTRDEPS}/<extracted proj folder>/build && cd ${VTRDEPS}/<extracted proj folder>/build
cmake ..
sudo cmake --build . --target install  # will install to /usr/local/[lib,bin]
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}  # put this in bashrc
```

### Install [OpenCV](https://opencv.org/) (>=4.5.0)

Before installing OpenCV, make sure that it is not already installed in the system.

```bash
sudo apt list --installed | grep opencv*
```

Install OpenCV from source, and get code from its official Github repository as listed below. The following OpenCV install instructions refer to the instructions from [here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
```

Download OpenCV and OpenCV Contrib from GitHub to the following directory: `${VTRDEPS}`

```bash
cd ${VTRDEPS}
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Checkout the corresponding branch of the version you want to install

```bash
cd ${VTRDEPS}/opencv && git checkout <opencv-version>  # e.g. <opencv-version> = 4.4.0
cd ${VTRDEPS}/opencv_contrib && git checkout <opencv-version>  # e.g. <opencv-version> = 4.4.0
```

Build and install OpenCV

```bash
mkdir -p ${VTRDEPS}/opencv/build && cd ${VTRDEPS}/opencv/build  # create build directory
# generate Makefiles
cmake -D CMAKE_BUILD_TYPE=RELEASE \
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
      -D BUILD_EXAMPLES=ON ..
make -j<nproc>  # <nproc> is number of cores of your computer, 12 for Lenovo P53
sudo make install  # copy libraries to /usr/local/[lib, include]
# verify your opencv version
pkg-config --modversion opencv4
python3 -c "import cv2; print(cv2.__version__)"  # for python 3
```

### Install [ROS2 Foxy](https://www.ros.org/)

Before installing ROS2, install ROS1 if necessary in case the robot is not ROS2 enabled - instructions [here](./ros1/README.md).

Instructions below follow the installation tutorial [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/).

Install ROS2 dependencies

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
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
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
```

Get ROS2 code and install more dependencies using `rosdep`

```bash
mkdir -p ${VTRDEPS}/ros_foxy && cd ${VTRDEPS}/ros_foxy  # root dir for ROS2
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
mkdir src
vcs import src < ros2.repos

sudo rosdep init # Note: if you follow the instructions above to install ROS1, then no need to run this line
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers python3-opencv libopencv-dev"
colcon build --symlink-install --packages-skip ros1_bridge
```

- Note:
  1. must ignore dependencies on opencv packages because it is installed from source with GPU support.
  2. do not install `ros1_bridge` package at this moment since it usually requires other setups to use.

`source` the `setup.bash` script

```bash
source ${VTRDEPS}/ros_foxy/install/setup.bash  # Run this command everytime when you want to use ROS2.
```

- Note: DO NOT `source` the `setup.bash` script if you need to install `ros1_bridge`. `ros1_bridge` installation instruction [here](./ros2/README.md).

### Install python dependencies

Install all python dependencies inside a python virtualenv so they do not corrupt system python packages.

```bash
sudo apt install python3-virtualenv
cd ${VTRVENV} && virtualenv . --system-site-packages
source ${VTRVENV}/bin/activate  # Run this command everytime when you use this virtual environment.
pip install pyyaml pyproj scipy zmq socketIO_client flask
pip install "python-socketio<5" "flask_socketio<5"  # TODO (yuchen) upgrade the version
```

### Install javascript dependencies

Javascript is required to build the UI.

```bash
sudo apt install nodejs npm
```

### Install VTR3

Start a new terminal and source relevant resources.

```bash
source ${VTRVENV}/bin/activate
source ${VTRDEPS}/ros_foxy/install/setup.bash
```

Install ROS dependencies

```bash
mkdir ${VTRDEPS}/vtr_ros2_deps
# vision opencv
cd ${VTRDEPS}/vtr_ros2_deps
git clone https://github.com/ros-perception/vision_opencv.git ros2_vision_opencv
cd ros2_vision_opencv
git checkout ros2
# xacro
cd ${VTRDEPS}/vtr_ros2_deps
git clone https://github.com/ros/xacro.git ros2_xacro
cd ros2_xacro
git checkout 2.0.3
# ros2_pcl_msgs (for lidar)
cd ${VTRDEPS}/vtr_ros2_deps
git clone git@github.com:ros-perception/pcl_msgs.git ros2_pcl_msgs
cd ros2_pcl_msgs
git checkout ros2
# ros2_perception (for lidar)
cd ${VTRDEPS}/vtr_ros2_deps
git clone git@github.com:ros-perception/perception_pcl.git ros2_perception_pcl
cd ros2_perception_pcl
git checkout 2.2.0
# joystick drivers (for grizzly control)
cd ${VTRDEPS}/vtr_ros2_deps
git clone git@github.com:ros-drivers/joystick_drivers.git
cd joystick_drivers
git checkout ros2
touch joy_linux/COLCON_IGNORE
touch spacenav/COLCON_IGNORE
# for robots at UTIAS
cd ${VTRDEPS}/vtr_ros2_deps
git clone git@github.com:utiasASRL/robots.git
cd robots
git checkout ros2
touch asrl__lancaster/COLCON_IGNORE
touch asrl__dji/COLCON_IGNORE
# install all
cd ${VTRDEPS}/vtr_ros2_deps
colcon build --symlink-install
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
```

Change nvidia gpu compute capability in [gpusurf](./ros2/src/deps/gpusurf/gpusurf/CMakeLists.txt) line 16 based on your GPU, default to Lenovo P53 which is 75.

Install VTR3:

First download the source code from github including submodules

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

then install the ROS2 packages

- option 1: build for production

  ```bash
  cd ${VTRSRC}/ros2
  colcon build --symlink-install
  ```

- option 2: build for development

  ```bash
  cd ${VTRSRC}/ros2
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"  # Debug build with code coverage test
  colcon test --event-handlers console_cohesion+ # Will also run style check for c++, python, cmake and xml files.
  colcon test-result  # Summary: xx tests, 0 errors, 0 failures, 0 skipped
  lcov --capture --directory build/ --output-file vtr3_coverage_report.info
  genhtml vtr3_coverage_report.info --output-directory vtr3_coverage_report
  ```

  Open the html report at `vtr3_coverage_report/index.html` to see code coverage.

## Launch VTR3

### Offline (Playback) Mode

Download relevant datasets following instructions [below](#vtr-datasets).

#### Stereo SURF-Feature-Based T&R

Run the following command to launch the system

```bash
tmuxp load ${VTRSRC}/ros2/src/vtr_navigation/tmuxp/offline_vtr_stereo_launch.yaml
```

and then follow the video demo [here](https://youtu.be/g0Y9YlG9ZYY).

#### LiDAR Point-Cloud-Based T&R

### Online (Grizzly) Mode

### Testing & Development Mode

Run the following commands before running any executables from VTR packages.

```bash
source ${VTRVENV}/bin/activate
source ${VTRSRC}/ros2/install/setup.bash
```

Check the offline tool and playback tutorial in vtr_testing.

## VTR Datasets

Download datasets from [here](https://drive.google.com/drive/folders/1LSEgKyqqQp1aadNCILK6f2lWMdTHyU-m?usp=sharing) and store them into `${VTRDATA}`.

```text
|- ${VTRDATA}
  |- utias_20210412
  |- utias_2016_inthedark
```

## Documentation

### [Conceptual design document](https://www.overleaf.com/7219422566kdxtydzpbyfj)

Convey the idea of the algorithms, with architecture diagrams

- Note: old conceptual design documents and architecture diagrams found on document server in `asrl/notes/vtr`

### Mid-level documentation

Tutorials, quick reference, installation guide should be put in the README.md of vtr3 and each of its sub-packages. Check example [here](https://github.com/utiasASRL/vtr2/tree/develop/asrl__navigation).

### In-source documentation

Doxygen comments in-source. Compile the documentation for the version you are using.

## [Contributing](./CONTRIBUTING.md)

## [License](./LICENSE.md)
