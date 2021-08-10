# VT&amp;R3

Visual Teach &amp; Repeat 3

- [VT&amp;R3](#vtr3)
  - [Installation](#installation)
    - [Hardware and Software Requirements](#hardware-and-software-requirements)
    - [Install Ubuntu 20.04](#install-ubuntu-2004)
    - [Directory Structure Overview](#directory-structure-overview)
    - [Install CUDA (>=11.3)](#install-cuda-113)
    - [Install Eigen (>=3.3.7)](#install-eigen-337)
    - [Install PROJ (>=8.0.0)](#install-proj-800)
    - [Install OpenCV (>=4.5.0)](#install-opencv-450)
    - [Install ROS2 Foxy](#install-ros2-foxy)
    - [Install VTR3](#install-vtr3)
      - [Download VTR3 source code](#download-vtr3-source-code)
      - [Install necessary system libraries](#install-necessary-system-libraries)
      - [Install python dependencies](#install-python-dependencies)
      - [Install necessary ROS packages](#install-necessary-ros-packages)
      - [Build and install VTR3:](#build-and-install-vtr3)
      - [Build VTR3 user interface](#build-vtr3-user-interface)
  - [Launch VTR3](#launch-vtr3)
    - [Offline (Playback) Mode](#offline-playback-mode)
      - [Stereo SURF-Feature-Based T&R](#stereo-surf-feature-based-tr)
      - [LiDAR Point-Cloud-Based T&R](#lidar-point-cloud-based-tr)
    - [Online (Grizzly) Mode](#online-grizzly-mode)
      - [Stereo SURF-Feature-Based T&R](#stereo-surf-feature-based-tr-1)
      - [LiDAR Point-Cloud-Based T&R](#lidar-point-cloud-based-tr-1)
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
export VTRROOT=~/ASRL  # root directory of VTR (this variable only initializes the following variables and won't be used anywhere else)
export VTRSRC=${VTRROOT}/vtr3  # source code of VTR (this repo)
export VTRDEPS=${VTRROOT}/workspace  # system dependencies of VTR
export VTRVENV=${VTRROOT}/venv  # python dependencies of VTR
export VTRDATA=${VTRROOT}/data  # datasets for VTR
export VTRTEMP=${VTRROOT}/temp  # temporary data directory for testing
```

Remember to create the above directories

```bash
mkdir -p ${VTRROOT} ${VTRSRC} ${VTRDEPS} ${VTRVENV} ${VTRDATA} ${VTRTEMP}
```

If the values above are used, the final directory structure should look like this:

```text
|- ~/ASRL
  |- vtr3              VTR3 source code and installation
    |- main            main packages of VTR3, must be installed to get a working system
    |- extensions      sensor, robot, dataset specific add-ons
    |- drivers         sensor drivers
    |- launch          tmuxp launch files
  |- workspace         system dependencies source code and (maybe) installation
    |- opencv          opencv source code cloned from github, installed to /usr/local/[lib,bin]
    |- opencv_contrib  extra opencv source code cloned from github, installed together with opencv
    |- proj            the latest version of PROJ, installed to /usr/local/[lib,bin]
    |- ros_foxy        source code and installation of ROS2 on Ubuntu 20.04
    |- vtr_ros2_deps   VTR dependencies from public repositories without modification
  |- data              datasets for VTR
```

### Install [CUDA](https://developer.nvidia.com/cuda-toolkit) (>=11.3)

Install CUDA through Debian package manager (the network version) from its [official website](https://developer.nvidia.com/cuda-toolkit). Be sure to perform the necessary [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index).

Very importantly, put the following in bashrc:

```bash
export PATH=/usr/local/cuda-<your cuda version, e.g. 11.3>/bin${PATH:+:${PATH}}
```

You can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`. It is possible that these two commands report different CUDA version, which means that your CUDA driver and toolkit version do not match. This is OK as long as the driver and toolkit are compatible, which you can verify in the documentation.

### Install Eigen (>=3.3.7)

```bash
sudo apt install libeigen3-dev
```

### Install [PROJ](https://proj.org/) (>=8.0.0)

The instructions below follow the installation instructions [here](https://proj.org/install.html#compilation-and-installation-from-source-code) except that source code is cloned from GitHub.

Install dependencies

```bash
sudo apt install cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev
```

Download PROJ from GitHub to the following directory: `${VTRDEPS}` and check out the branch of the version you want to install

```bash
mkdir -p ${VTRDEPS}/proj && cd $_
git clone https://github.com/OSGeo/PROJ.git .
git checkout <proj-version>  # e.g. <proj-version> = 8.0.0
```

Build and install PROJ

```bash
mkdir -p ${VTRDEPS}/proj/build && cd $_
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

Check out the corresponding branch of the version you want to install

```bash
cd ${VTRDEPS}/opencv && git checkout <opencv-version>  # e.g. <opencv-version> = 4.5.0
cd ${VTRDEPS}/opencv_contrib && git checkout <opencv-version>  # e.g. <opencv-version> = 4.5.0
```

Build and install OpenCV

```bash
mkdir -p ${VTRDEPS}/opencv/build && cd $_  # create build directory
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
make -j<nproc>  # <nproc> is the number of cpu cores of your computer, 12 for Lenovo P53
sudo make install  # copy libraries to /usr/local/[lib, include]
# verify your opencv version
pkg-config --modversion opencv4
python3 -c "import cv2; print(cv2.__version__)"  # for python 3
```

### Install [ROS2 Foxy](https://www.ros.org/)

Before installing ROS2, install ROS1 if necessary in case the robot is not ROS2 enabled - instructions [here](./ros1_instructions.md).

Instructions below follow the installation tutorial [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/). If you already have ROS2 installed then you can skip this section and replace the `setup.bash` being `sourced` below to your ROS2 installation.

Install ROS2 dependencies

```bash
locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt update && sudo apt install curl gnupg2 lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key  -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

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
mkdir -p ${VTRDEPS}/ros_foxy && cd $_  # root dir for ROS2
wget https://raw.githubusercontent.com/ros2/ros2/foxy/ros2.repos
mkdir -p src
vcs import src < ros2.repos

sudo rosdep init # Note: if you follow the instructions above to install ROS1, then no need to run this line
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers python3-opencv libopencv-dev"
colcon build --symlink-install --packages-skip ros1_bridge
```

- Note:
  1. must ignore dependencies on opencv packages because it is installed from source with GPU support.
  2. do not install `ros1_bridge` package at this moment since it usually requires other setups to use, see [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and [here](./ros1_instructions.md).

`source` the `setup.bash` script

```bash
source ${VTRDEPS}/ros_foxy/install/setup.bash  # Run this command everytime when you want to use ROS2.
```

### Install VTR3

#### Download VTR3 source code

Download the source code of VTR3 from github

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

and then change nvidia gpu compute capability in [gpusurf](./main/src/deps/gpusurf/gpusurf/CMakeLists.txt) line 16 based on your GPU, default to Lenovo P53 which is 75.

#### Install necessary system libraries

```bash
sudo apt install -y tmux
sudo apt install -y doxygen  # for building the documentation
sudo apt install -y nodejs npm protobuf-compiler  # for building the interface
sudo apt install -y libdc1394-22 libdc1394-22-dev  # for BumbleBee stereo camera
sudo apt install -y libbluetooth-dev libcwiid-dev  # for joystick drivers
sudo apt install -y libboost-all-dev libomp-dev  # boost and openmp, needed by multiple packages
sudo apt install -y libpcl-dev  # point cloud library
```

#### Install python dependencies

```bash
cd ${VTRSRC} && pip3 install -r requirements.txt
```

#### Install necessary ROS packages

Start a new terminal and source ros2 installation

```bash
source ${VTRDEPS}/ros_foxy/install/setup.bash
```

and then download and install packages

```bash
mkdir -p ${VTRDEPS}/vtr_ros2_deps/src
# vision opencv
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/vision_opencv.git ros2_vision_opencv
cd ros2_vision_opencv
git checkout ros2
# xacro
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros/xacro.git ros2_xacro
cd ros2_xacro
git checkout 2.0.3
# ros2_pcl_msgs (for lidar)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/pcl_msgs.git ros2_pcl_msgs
cd ros2_pcl_msgs
git checkout ros2
# ros2_perception (for lidar)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/perception_pcl.git ros2_perception_pcl
cd ros2_perception_pcl
git checkout 2.2.0
# joystick drivers (for grizzly control)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-drivers/joystick_drivers.git
cd joystick_drivers
git checkout ros2
touch joy_linux/COLCON_IGNORE
touch spacenav/COLCON_IGNORE
# for robots at UTIAS
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/utiasASRL/robots.git
cd robots
git checkout ros2
touch asrl__lancaster/COLCON_IGNORE
touch asrl__dji/COLCON_IGNORE
# install all
cd ${VTRDEPS}/vtr_ros2_deps
colcon build --symlink-install
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
```

#### Build and install VTR3:

Finally, install the ROS2 packages

- option 1: build for production

  ```bash
  cd ${VTRSRC}/main
  colcon build --symlink-install
  ```

- option 2: build for development

  ```bash
  cd ${VTRSRC}/main
  colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"  # Debug build with code coverage test
  colcon test --event-handlers console_cohesion+ # Will also run style check for c++, python, cmake and xml files.
  colcon test-result  # Summary: xx tests, 0 errors, 0 failures, 0 skipped
  lcov --capture --directory build/ --output-file vtr3_coverage_report.info
  genhtml vtr3_coverage_report.info --output-directory vtr3_coverage_report
  ```

  Open the html report at `vtr3_coverage_report/index.html` to see code coverage. Note that this option is for development only and should not be used for benchmarking and/or real robot experiments.

  State estimation in VT&R (odometry, mapping and localization) can be built to run deterministically by adding the following flags to the [common cmake file](./main/src/vtr_common/vtr_include.cmake), given that data come at a sufficiently slow rate.

  ```bash
  add_definitions(-DDETERMINISTIC_VTR)  # disable multi-threading in VTR state estimation and force any GPU job to run deterministically
  add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1)  # disable multi-threading in STEAM
  ```

#### Build VTR3 user interface

```bash
VTRUI=${VTRSRC}/main/src/vtr_ui/vtr_ui/frontend/vtr-ui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build
```

## Launch VTR3

We use [tmux](https://github.com/tmux/tmux/wiki) and [tmuxp](https://github.com/tmux-python/tmuxp) to launch and run VTR3.

### Offline (Playback) Mode

Download relevant datasets following instructions [below](#vtr-datasets).

#### Stereo SURF-Feature-Based T&R

Run the following command to launch the system

```bash
tmuxp load ${VTRSRC}/launch/offline_vtr_stereo.launch.yaml
```

and then follow the video demo [here](https://youtu.be/g0Y9YlG9ZYY).

#### LiDAR Point-Cloud-Based T&R

TODO

### Online (Grizzly) Mode

#### Stereo SURF-Feature-Based T&R

TODO

#### LiDAR Point-Cloud-Based T&R

TODO

### Testing & Development Mode

Run the following commands before running any executables from VTR packages.

```bash
source ${VTRSRC}/main/install/setup.bash
```

Check the offline tool and playback tutorial in vtr_testing.

## VTR Datasets

Download datasets from [here](https://drive.google.com/drive/folders/1LSEgKyqqQp1aadNCILK6f2lWMdTHyU-m?usp=sharing), unzip and store them into `${VTRDATA}`, e.g.,

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
