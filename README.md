# VT&amp;R3

Visual Teach &amp; Repeat

- [VT&amp;R3](#vtr3)
  - [Installation](#installation)
    - [Code Base Overview](#code-base-overview)
    - [Hardware and Software Requirements](#hardware-and-software-requirements)
    - [Install Ubuntu](#install-ubuntu)
    - [Install CUDA Driver and Toolkit](#install-cuda-driver-and-toolkit)
    - [Change default python version to python3](#change-default-python-version-to-python3)
    - [Install Eigen](#install-eigen)
    - [Install PROJ](#install-proj)
    - [Install OpenCV](#install-opencv)
    - [Install ROS2](#install-ros2)
    - [Install python dependencies](#install-python-dependencies)
    - [Install VTR3](#install-vtr3)
    - [Launch VTR3](#launch-vtr3)
  - [Documentation](#documentation)
    - [Conceptual design document](#conceptual-design-document)
    - [Mid-level documentation](#mid-level-documentation)
    - [In-source documentation](#in-source-documentation)
  - [Contributing & Code of Conduct](#contributing--code-of-conduct)
  - [License](#license)

## Installation

Note: The old install notes can be found [here](https://github.com/utiasASRL/vtr2), which are mainly aimed at older laptops with Ubuntu 14.04. Some changes for 16.04 installation are mentioned. Additional (older) notes can be found on the [lab wiki](http://192.168.42.2/mediawiki/index.php/ROS:Charlottetown_Installation).

### Code Base Overview

The instructions will create a final code base layout as follows:

```text
|- ~/charlottetown         All vtr2 stuff
    |- extras              Third party dependencies
    |- utiasASRL           ASRL vtr2 code base & all its required libraries
        |- robots          ASRL robot-specific code
|- ~/ASRL
    |- vtr3                VTR3 source code and installation
    |- workspace           System dependencies source code and (maybe) installation
        |- opencv          opencv source code cloned from github, installed to /usr/local/[lib,bin]
        |- opencv_contrib  extra opencv source code cloned from github, installed together with opencv
        |- ros_foxy        source code and installation of ROS2 on Ubuntu 20.04
        |- proj-<version>  the newest version of PROJ, which is required by VT&R
```

VT&R3 Package list (in this repository)

- [vtr_documentation](src/vtr_documentation) Generate VT&R3 documentation via Doxygen

### Hardware and Software Requirements

Assume Lenovo P53 laptops, but technically any computer with an Nvidia GPU.

Software requirements listed below. We make no guarantee that VTR works when there is any version mismatch.

### Install [Ubuntu](https://ubuntu.com/)

Required version: 20.04

Install Ubuntu from [official website](https://ubuntu.com/).

- Note: For dual boot system, remember to DISABLE [device encryption](https://support.microsoft.com/en-ca/help/4028713/windows-10-turn-on-device-encryption) before start installing Ubuntu.

Make sure your system packages are up to date:

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

### Install [CUDA Driver and Toolkit](https://developer.nvidia.com/cuda-toolkit)

Required version: >=11.2

Install CUDA through Debian package manager (the network version) from its [official website](https://developer.nvidia.com/cuda-toolkit). Be sure to perform the necessary [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index).

You can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`. It is possible that these two commands report different CUDA version, which means that your CUDA driver and toolkit version do not match. This is OK as long as the driver and toolkit are compatible, which you can verify in the documentation.

Optional: Install CuDNN through Debian package manager from its [official website](https://developer.nvidia.com/cudnn)

### Change default python version to python3

This is needed because we want to use python3 to install everything, but some packages default to use python from `/usr/bin/python` which is python2.

```bash
sudo apt install python-is-python3
```

### Install Eigen

Required version: >=3.3.7

```bash
sudo apt install libeigen3-dev
```

### Install [PROJ](https://proj.org/)

Required version: >=7.2.1

The instructions below follow the installation instructions [here](https://proj.org/install.html#compilation-and-installation-from-source-code). Download the [latest release](https://proj.org/download.html#current-release) first and extract it in to `~/ASRL/workspace`

```bash
sudo apt install cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev # dependencies
mkdir ~/ASRL/workspace/<extracted proj folder>/build && cd ~/ASRL/workspace/<extracted proj folder>/build
cmake ..
sudo cmake --build . --target install  # will install to /usr/local/[lib,bin]
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

### Install [OpenCV](https://opencv.org/)

Required version: >=4.5.0

Before installing OpenCV, make sure that it is not already installed in the system.

```bash
sudo apt list --installed | grep opencv*
```

Install OpenCV from source, and get code from its official Github repository as listed below. The following OpenCV install instructions refer to the instructions from [here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).

```bash
sudo apt-get install build-essential
sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
```

Download OpenCV and OpenCV Contrib from GitHub to the following directory: `~/ASRL/workspace`

```bash
mkdir -p ~/ASRL/workspace && cd ~/ASRL/workspace
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Checkout the corresponding branch of the version you want to install

```bash
cd ~/ASRL/workspace/opencv && git checkout <opencv-version>  # e.g. <opencv-version> = 4.4.0
cd ~/ASRL/workspace/opencv_contrib && git checkout <opencv-version>  # e.g. <opencv-version> = 4.4.0
```

- **TODO**: currently we need `xfeatures2d` library from opencv_contrib but this may change in the future, so keep an eye on the updates of OpenCV.

Build and install OpenCV

```bash
mkdir -p ~/ASRL/workspace/opencv/build && cd ~/ASRL/workspace/opencv/build  # create build directory
# generate Makefiles
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D OPENCV_EXTRA_MODULES_PATH=~/ASRL/workspace/opencv_contrib/modules \
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

### Install [ROS2](https://www.ros.org/)

Required version: Foxy

Before installing ROS2, install ROS1 if necessary. Instructions [here](./ros1/README.md).

Instructions follow the installation tutorial [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)

Install ROS2 dependencies

- Note: the following commands are copied from the tutorial link above. No change needed for our case.

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
mkdir -p ~/ASRL/workspace/ros_foxy && cd ~/ASRL/workspace/ros_foxy  # root dir for ROS2
wget https://raw.githubusercontent.com/ros2/ros2/master/ros2.repos
mkdir src
vcs import src < ros2.repos

sudo rosdep init # Note: if you follow the instructions above to install ROS1, then no need to run this line
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro foxy -y --skip-keys "console_bridge fastcdr fastrtps rti-connext-dds-5.3.1 urdfdom_headers python3-opencv libopencv-dev"
colcon build --symlink-install --packages-skip ros1_bridge
```

- Note:
  1. the commands above are also mostly copied from the online tutorial, but we also ignore dependencies on opencv packages since we have already installed them from source.
  2. we also do not install `ros1_bridge` package at this moment since it usually requires other setups to use.

`source` the `setup.bash` script

```bash
source ./install/setup.bash  # Run this command later (NOT now), and everytime after when you want to use ROS2.
```

DO NOT `source` the `setup.bash` script if you need to install `ros1_bridge`. `ros1_bridge` installation instruction [here](./ros2/README.md).

### Install python dependencies

We install all python dependencies inside a python virtual environment so that they do not corrupt system python packages. **Important**: activate the virtualenv whenever you install things starting from this stage and run vtr.

```bash
sudo apt install python3-virtualenv
cd ~/ASRL && virtualenv venv --system-site-packages
source ~/ASRL/venv/bin/activate
pip install pyyaml pyproj scipy zmq socketIO_client flask
pip install "python-socketio<5" "flask_socketio<5"  # TODO (yuchen) upgrade the version
```

### Install VTR3

Start a new terminal

```bash
source ~/ASRL/venv/bin/activate
source ~/ASRL/workspace/ros_foxy/install/setup.bash
```

```bash
mkdir ~/ASRL/workspace/vtr_ros2_deps
# vision opencv
cd ~/ASRL/workspace/vtr_ros2_deps
git clone https://github.com/ros-perception/vision_opencv.git ros2_vision_opencv
cd ros2_vision_opencv
git checkout ros2
# xacro
cd ~/ASRL/workspace/vtr_ros2_deps
git clone https://github.com/ros/xacro.git ros2_xacro
cd ros2_xacro
git checkout 2.0.3
# ros2_pcl_msgs
cd ~/ASRL/workspace/vtr_ros2_deps
git clone git@github.com:ros-perception/pcl_msgs.git ros2_pcl_msgs
cd ros2_pcl_msgs
git checkout ros2
# ros2_perception
cd ~/ASRL/workspace/vtr_ros2_deps
git clone git@github.com:ros-perception/perception_pcl.git ros2_perception_pcl
cd ros2_perception_pcl
git checkout 2.2.0
# install all
cd ~/ASRL/workspace/vtr_ros2_deps
colcon build --symlink-install
source ~/ASRL/workspace/vtr_ros2_deps/install/setup.bash
```

Change nvidia gpu compute capability in [gpusurf](./ros2/src/deps/gpusurf/gpusurf/CMakeLists.txt).

Option 1: Build vtr3 for production

```bash
cd ~/ASRL/vtr3/ros2
colcon build --symlink-install
```

Option 2: Build vtr3 for development

```bash
cd ~/ASRL/vtr3/ros2
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug -DCMAKE_CXX_FLAGS="--coverage"  # Debug build with code coverage test
colcon test --event-handlers console_cohesion+ # Will also run style check for c++, python, cmake and xml files.
colcon test-result  # Summary: xx tests, 0 errors, 0 failures, 0 skipped
lcov --capture --directory build/ --output-file vtr3_coverage_report.info
genhtml vtr3_coverage_report.info --output-directory vtr3_coverage_report
```

Open the html report at `vtr3_coverage_report/index.html` to see code coverage.

### Launch VTR3

```bash
source ~/ASRL/venv/bin/activate
source ~/ASRL/vtr3/ros2/install/setup.bash
```

Check the offline tool and playback tutorial in vtr_testing.

## Documentation

### [Conceptual design document](https://www.overleaf.com/7219422566kdxtydzpbyfj)

convey the idea of the algorithms, with architecture diagrams

- Note:
  - Old conceptual design documents and architecture diagrams found on our document server in `asrl/notes/vtr`

### Mid-level documentation

tutorials, quick reference, install guide should be put in the README.md of vtr3 and each of its sub-packages. Check example [here](https://github.com/utiasASRL/vtr2/tree/develop/asrl__navigation).

### In-source documentation

Doxygen comments in-source -- please compile the documentation for the specific commit you are using.

## Contributing & Code of Conduct

See [CONTRIBUTING.md](./CONTRIBUTING.md)

## License

TODO
