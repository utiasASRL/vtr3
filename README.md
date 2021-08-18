# VT&amp;R3

Visual Teach &amp; Repeat 3

- [VT&amp;R3](#vtr3)
  - [Install VTR3](#install-vtr3)
    - [Hardware Requirement](#hardware-requirement)
    - [Install Ubuntu 20.04](#install-ubuntu-2004)
    - [Create VTR3 Directories](#create-vtr3-directories)
    - [Download VTR3 source code](#download-vtr3-source-code)
    - [Install CUDA (>=11.3)](#install-cuda-113)
    - [Install Eigen (>=3.3.7)](#install-eigen-337)
    - [Install PROJ (>=8.0.0)](#install-proj-800)
    - [Install OpenCV (>=4.5.0)](#install-opencv-450)
    - [Install ROS2 Foxy](#install-ros2-foxy)
    - [(OPTIONAL) Install ROS1 Noetic](#optional-install-ros1-noetic)
    - [Install miscellaneous system libraries](#install-miscellaneous-system-libraries)
    - [Install miscellaneous Python dependencies](#install-miscellaneous-python-dependencies)
    - [Install miscellaneous ROS2 dependencies](#install-miscellaneous-ros2-dependencies)
    - [Build and install VTR3](#build-and-install-vtr3)
    - [Install driver and robot description packages](#install-driver-and-robot-description-packages)
    - [Install VTR3 add-ons](#install-vtr3-add-ons)
  - [VTR3 Datasets](#vtr3-datasets)
  - [Launch VTR3](#launch-vtr3)
    - [Offline Mode](#offline-mode)
      - [Stereo SURF-Feature-Based T&R](#stereo-surf-feature-based-tr)
      - [LiDAR Point-Cloud-Based T&R](#lidar-point-cloud-based-tr)
    - [(INTERNAL) Online Mode - VTR3 on Grizzly](#internal-online-mode---vtr3-on-grizzly)
      - [Stereo SURF-Feature-Based T&R](#stereo-surf-feature-based-tr-1)
      - [LiDAR Point-Cloud-Based T&R](#lidar-point-cloud-based-tr-1)
  - [Documentation](#documentation)
    - [(INTERNAL) Conceptual design document](#internal-conceptual-design-document)
    - [In-source documentation](#in-source-documentation)
  - [Contributing](#contributing)
  - [License](#license)

## Install VTR3

The following instructions aim at being friendly to users with less experiences in using Linux, CUDA, ROS2, etc. Some sections can be skipped if the dependencies have been installed already. Anything marked `INTERNAL` is for ASRL students.

### Hardware Requirement

A high-powered, Nvidia GPU enabled machine.

<!-- At ASRL, we run VTR3 on Lenovo P53 laptop that has an Intel Core i7-9750H CPU, 32GB DDR4 RAM and an NVIDIA Quadro T2000 4GB GPU. -->

### Install [Ubuntu 20.04](https://ubuntu.com/)

Install Ubuntu from their [official website](https://ubuntu.com/).

- (INTERNAL) Note: for dual boot system, remember to DISABLE [device encryption](https://support.microsoft.com/en-ca/help/4028713/windows-10-turn-on-device-encryption) before start installing Ubuntu.

### Create VTR3 Directories

The follow environment variables are assumed present so that files and data can be put into different locations on different machines. It is recommended to put them in bashrc.

```bash
export VTRROOT=~/ASRL  # (INTERNAL default) root directory of VTR (this variable only initializes the following variables and won't be used anywhere else)
# you can change the following directories to anywhere appropriate
export VTRSRC=${VTRROOT}/vtr3        # source code of VTR (this repo)
export VTRDEPS=${VTRROOT}/workspace  # system dependencies of VTR
export VTRVENV=${VTRROOT}/venv       # python dependencies of VTR
export VTRDATA=${VTRROOT}/data       # datasets for VTR
export VTRTEMP=${VTRROOT}/temp       # temporary data directory for testing
```

Remember to create these directories

```bash
mkdir -p ${VTRSRC} ${VTRDEPS} ${VTRVENV} ${VTRDATA} ${VTRTEMP}
```

If the default values above are used, the final directory structure should look like

```text
|- ~/ASRL
  |- data              datasets for VTR3
  |- temp              temporary files
  |- venv              python virtual env for VTR3 (NOTE: currently not used, but will do eventually)
  |- vtr3              VTR3 source code and installation
    |- drivers         sensor, controller drivers not specific to VTR3
    |- extra           VTR3 specific sensor, robot, dataset specific add-ons
    |- launch          tmuxp launch files
    |- main            main packages of VTR3, must be installed to get a working system
    |- robots          robot description packages not specific to VTR3
  |- workspace         system dependencies source code and (maybe) installation
    |- opencv          opencv source code cloned from github, installed to /usr/local/[lib,bin]
    |- opencv_contrib  extra opencv source code cloned from github, installed together with opencv
    |- proj            the latest version of PROJ, installed to /usr/local/[lib,bin]
    |- ros1_bridge     (optional) ros1-ros2 message passing bridge package
    |- ros_foxy        source code and installation of ROS2 on Ubuntu 20.04
    |- ros_noetic      (optional) source code and installation of ROS1 on Ubuntu 20.04
    |- vtr_ros2_deps   VTR3 dependencies from public repositories without modification
```

### Download VTR3 source code

Download the source code from github

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

Machine specific settings

- change nvidia gpu compute capability in [gpusurf](./main/src/deps/gpusurf/gpusurf/CMakeLists.txt) line 16 based on your GPU, default is 75.

### Install [CUDA](https://developer.nvidia.com/cuda-toolkit) (>=11.3)

Install CUDA through Debian package manager (the network version) from its [official website](https://developer.nvidia.com/cuda-toolkit). Be sure to perform the necessary [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index).

Very importantly, put the following in bashrc:

```bash
export PATH=/usr/local/cuda-<your cuda version, e.g. 11.3>/bin${PATH:+:${PATH}}
```

You can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`.

### Install [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (>=3.3.7)

```bash
# using APT
sudo apt -q -y install libeigen3-dev

# OR from source if preferred
mkdir -p ${VTRDEPS}/eigen && cd $_
git clone https://gitlab.com/libeigen/eigen.git . && git checkout 3.3.7
mkdir build && cd $_
cmake .. && make install # default install location is /usr/local/
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

- Note: TODO add instructions on how to specify OpenCV paths so that we can have multiple versions installed.

Install OpenCV from source. The following instructions are copied from [here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
```

Download OpenCV and OpenCV Contrib from GitHub to `${VTRDEPS}`

```bash
cd ${VTRDEPS}
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Check out the version you want to install

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

Instructions below follow [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/). If you already have ROS2 installed then you can skip this section and replace the `setup.bash` `source`d below to your ROS2 installation.

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
  1. must ignore dependencies on opencv packages because it is installed from source with GPU support above.
  2. do not install `ros1_bridge` package at this moment since it usually requires other setups to use, see [here](https://github.com/ros2/ros1_bridge/blob/master/README.md) and the next section.

`source` the `setup.bash` script

```bash
source ${VTRDEPS}/ros_foxy/install/setup.bash  # Run this command everytime you want to use ROS2.
```

### (OPTIONAL) Install [ROS1 Noetic](https://www.ros.org/)

If you are working with robots and/or sensors that are not ROS2 enabled, you may need to install ROS1 Noetic and ros1_bridge to pass messages between ROS1 and ROS2. Follow the instructions [here](./ros1_instructions.md).

### Install miscellaneous system libraries

```bash
sudo apt install -y tmux  # for launching VTR3
sudo apt install -y doxygen  # for building the documentation
sudo apt install -y nodejs npm protobuf-compiler  # for building the interface
sudo apt install -y libdc1394-22 libdc1394-22-dev  # for BumbleBee stereo camera
sudo apt install -y libbluetooth-dev libcwiid-dev  # for joystick drivers
sudo apt install -y libboost-all-dev libomp-dev  # boost and openmp, needed by multiple packages
sudo apt install -y libpcl-dev  # point cloud library, for LiDAR VTR
```

### Install miscellaneous Python dependencies

```bash
cd ${VTRSRC} && pip3 install -r requirements.txt
```

### Install miscellaneous ROS2 dependencies

Source your ros2 installation

```bash
source ${VTRDEPS}/ros_foxy/install/setup.bash
```

- Note: if you installed ROS1 Noetic and ros1_bridge then `source` the workspace with ros1_bridge installed.

then download and install packages

```bash
mkdir -p ${VTRDEPS}/vtr_ros2_deps/src
# vision opencv
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/vision_opencv.git ros2_vision_opencv
cd ros2_vision_opencv
git checkout ros2
# xacro (for robot descriptions)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros/xacro.git ros2_xacro
cd ros2_xacro
git checkout 2.0.3
# ros2_pcl_msgs (for LiDAR VTR)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/pcl_msgs.git ros2_pcl_msgs
cd ros2_pcl_msgs
git checkout ros2
# ros2_perception (for LiDAR VTR)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/perception_pcl.git ros2_perception_pcl
cd ros2_perception_pcl
git checkout 2.2.0
# joystick drivers (for grizzly control) TODO: move this to drivers directory
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-drivers/joystick_drivers.git
cd joystick_drivers
git checkout ros2
touch joy_linux/COLCON_IGNORE
touch spacenav/COLCON_IGNORE
# install all
cd ${VTRDEPS}/vtr_ros2_deps
colcon build --symlink-install
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
```

### Build and install VTR3

Source the ROS2 workspace with dependencies installed

```bash
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash
```

Install VTR3 ROS2 packages

- option 1 (for users)

  ```bash
  cd ${VTRSRC}/main
  colcon build --symlink-install
  ```

- option 2 (for maintainers)

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
  add_definitions(-DVTR_DETERMINISTIC)  # disable multi-threading in VTR state estimation and force any GPU job to run deterministically
  add_definitions(-DSTEAM_DEFAULT_NUM_OPENMP_THREADS=1)  # disable multi-threading in STEAM
  ```

Build VTR3 user interface

```bash
VTRUI=${VTRSRC}/main/src/vtr_ui/vtr_ui/frontend/vtr-ui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build
```

### Install driver and robot description packages

Driver and robot description packages are not mandatary, depending on the sensors and robots to be used with VTR3.

- Note: (INTERNAL) ASRL students install them all.

Source the ROS2 workspace with VTR3 installed

```bash
source ${VTRSRC}/main/install/setup.bash
```

Install drivers

```bash
cd ${VTRSRC}/drivers/ros2
colcon build --symlink-install
source ${VTRSRC}/drivers/ros2/install/setup.bash
```

Install robot descriptions

```bash
cd ${VTRSRC}/robots/ros2
colcon build --symlink-install
source ${VTRSRC}/robots/ros2/install/setup.bash
```

### Install VTR3 add-ons

These packages are not mandatary.

- Note: (INTERNAL) ASRL students install vtr_bumblebee_xb3 to use the Bumblebee XB3 camera on Grizzly.

Source the ROS2 workspace with VTR3 plus drivers and robot packages installed

```bash
source ${VTRSRC}/robots/ros2/install/setup.bash
```

Install add-ons

```bash
cd ${VTRSRC}/extra
colcon build --symlink-install
source ${VTRSRC}/extra/install/setup.bash
```

## VTR3 Datasets

Some datasets can be downloaded from [here](https://drive.google.com/drive/folders/1LSEgKyqqQp1aadNCILK6f2lWMdTHyU-m?usp=sharing). Unzip and store them into `${VTRDATA}`, e.g.,

```text
|- ${VTRDATA}
  |- utias_20210412_camera
  |- utias_20210812_lidar_rosbag
```

## Launch VTR3

We use [tmux](https://github.com/tmux/tmux/wiki) and [tmuxp](https://github.com/tmux-python/tmuxp) to launch and run VTR3. They are useful tools that allow multiple terminal sessions to be launched and accessed simultaneously in a single window.

### Offline Mode

With the datasets above, VTR3 can be run without connecting to any sensor or robot.

#### Stereo SURF-Feature-Based T&R

Prerequisites

- Installed VTR3+UI inside `main` folder including all its dependencies.
- Installed Grizzly robot description packages inside `robots` folder.
- Installed `pgr_triclops` package inside `drivers/ros2` folder and `vtr_bumblebee_xb3` package inside `extra` folder.
- Downloaded the `utias_20210412_camera` dataset into `${VTRDATA}`.

The video demo [at this link](https://youtu.be/g0Y9YlG9ZYY) shows how to

- Launch VTR3 to use stereo camera images as input.
  ```bash
  tmuxp load ${VTRSRC}/launch/offline_vtr_camera.launch.yaml
  ```
- Use the UI to start teaching a path.
- Use the UI to perform loop closure, i.e., merge into existing path.
- Use the UI to align the graph with the underlying satellite map.
- Use the UI to place the robot on a different location in the graph.
- Use the UI to specify a repeat path and start repeating the path.
- Terminate VTR3 (`Ctrl-C` once in terminal)

#### LiDAR Point-Cloud-Based T&R

Prerequisites

- Installed VTR3+UI inside `main` folder including all its dependencies
- Installed Grizzly robot description packages inside `robots` folder
- Downloaded the `utias_20210812_lidar_rosbag` dataset into `${VTRDATA}`.

The video demo [at this link](https://youtu.be/KiJkTYet944) shows how to

- Launch VTR3 to use LiDAR point-clouds as input
  ```bash
  # launch command
  tmuxp load ${VTRSRC}/launch/offline_vtr_lidar.launch.yaml
  ```
- Use the UI to start teaching a path and replay point-clouds from the dataset
  ```bash
  # replay the first rosbag
  source ${VTRSRC}/main/install/setup.bash
  ros2 bag play ${VTRDATA}/utias_20210812_lidar_rosbag/rosbag2_2021_08_12-20_02_12
  ```
- Use the UI to align the graph with the underlying satellite map.
- Use the UI to place the robot on a different location in the graph.
- Use the UI to specify a repeat path and start repeating the path.
  ```bash
  # replay the second rosbag
  source ${VTRSRC}/main/install/setup.bash
  ros2 bag play ${VTRDATA}/utias_20210812_lidar_rosbag/rosbag2_2021_08_12-20_14_20
  ```
- Terminate VTR3 (`Ctrl-C` once in terminal)

### (INTERNAL) Online Mode - VTR3 on Grizzly

Connect Grizzly computer (ethernet) and XBox controller (usb) to your machine. Use `grizzly_control.launch.yaml` to start passing commands from the XBox controller to Grizzly.

```bash
tmuxp load ${VTRSRC}/launch/grizzly_control.launch.yaml
```

- Note: your machine receives commands from the XBox controller and publishes them as ROS2 messages. The Grizzly computer receives ROS2 messages and converts them to ROS1 messages, which are then received by the internal micro-controller.
- Note: ask a lab mate for passwords of the Grizzly computer and controller key mappings.

#### Stereo SURF-Feature-Based T&R

Connect the Bumblebee XB3 camera (usb-c) to your machine. Use `online_vtr_camera.launch.yaml` to launch VTR3.

```bash
tmuxp load ${VTRSRC}/launch/online_vtr_camera.launch.yaml
```

#### LiDAR Point-Cloud-Based T&R

Connect the Honeycomb LiDAR (ethernet-usb) to your machine. Use `honeycomb.launch.yaml` to turn on the sensor, which will then publish point-clouds as ROS1 and ROS2 messages (ros1_bridge is launched alongside).

```bash
tmuxp load ${VTRSRC}/launch/honeycomb.launch.yaml
```

Use `online_vtr_lidar.launch.yaml` to launch VTR3.

```bash
tmuxp load ${VTRSRC}/launch/online_vtr_lidar.launch.yaml
```

## Documentation

### [(INTERNAL) Conceptual design document](https://www.overleaf.com/7219422566kdxtydzpbyfj)

### [In-source documentation](./main/src/vtr_documentation/README.md)

After installing VTR3, the in-source documentation can be accessed by opening [index.html](./main/install/vtr_documentation/docs/html/index.html) in browser.

## [Contributing](./CONTRIBUTING.md)

## [License](./LICENSE)
