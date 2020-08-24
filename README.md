# vtr3

Make VT&amp;R Great Again

## Table of Contents

- [vtr3](#vtr3)
  - [Table of Contents](#table-of-contents)
  - [Installation](#installation)
    - [Code Base Overview](#code-base-overview)
    - [Hardware Requirement](#hardware-requirement)
    - [Install Ubuntu](#install-ubuntu)
    - [Install CUDA Driver and Toolkit](#install-cuda-driver-and-toolkit)
    - [Change default python version to python3](#change-default-python-version-to-python3)
    - [Install Eigen](#install-eigen)
    - [Install PROJ](#install-proj)
    - [Install OpenCV](#install-opencv)
      - [Install ROS2](#install-ros2)
      - [Install ros1_bridge](#install-ros1_bridge)
    - [Install catkin tools](#install-catkin-tools)
    - [Install Third-Party ROS1 Packages (VTR2 Build)](#install-third-party-ros1-packages-vtr2-build)
    - [Install utiasASRL robots library](#install-utiasasrl-robots-library)
    - [Install VTR](#install-vtr)
    - [Clean-Up (This section is not needed for now, because I don't know where the following env vars are used.)](#clean-up-this-section-is-not-needed-for-now-because-i-dont-know-where-the-following-env-vars-are-used)
    - [Install VTR3 (this repo)](#install-vtr3-this-repo)
    - [Prepare VTR3 for launching](#prepare-vtr3-for-launching)
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
        |- vtr2/deps       VTR2 dependencies
        |- vtr2            VTR2 source code
|- ~/ASRL
    |- vtr3                VTR3 source code and installation
    |- workspace           System dependencies source code and (maybe) installation
        |- opencv          opencv source code cloned from github, installed to /usr/local/[lib,bin]
        |- opencv_contrib  extra opencv source code cloned from github, installed together with opencv
        |- ros_noetic      source code and installation of ROS1 on Ubuntu 20.04
        |- ros_foxy        source code and installation of ROS2 on Ubuntu 20.04
        |- proj-<version>  the newest version of PROJ, which is required by VT&R
```

The directory structure will stay mostly the same for older Ubuntu versions, except for name changes, e.g. ros_noetic -> ros_melodic.

VT&R2 Package list (from the vtr2 repository)

- [asrl\_\_cmake](asrl__cmake) Build support such as custom CMake files.
- [asrl\_\_common](asrl__common) Generic tools, such as timing, etc.
- [asrl\_\_vision](asrl__vision) Vision-related tools, such as feature matching, RANSAC, etc.
- [asrl\_\_pose_graph](asrl__pose_graph) Implements the pose graph used throughout vtr2.
- [asrl\_\_steam_extensions](asrl__steam_extensions) Additions and enhancements to [steam](https://github.com/utiasASRL/steam) that are specific to VTR2.
- [asrl\_\_terrain_assessment](asrl__terrain_assessment) Stereo terrain assessment package written by Peter Berczi.
- [asrl\_\_navigation](asrl__navigation) High-level package that contains the Navigator: the primary binary used to run vtr2 live, its modules, launch files and params.
- [asrl\_\_offline_tools](asrl__offline_tools) High-level package that contains tools for local testing using pre-gathered data. These tools are a little less complicated than running the Navigator.
- Note: Not guaranteed to be complete, browse top-level directories for a complete package list.

VT&R3 Package list (in this repository)

- [vtr_documentation](src/vtr_documentation) Generate VT&R3 documentation via Doxygen

### Hardware Requirement

Currently we are only running VTR3 on Lenovo P53 laptops. But technically any computer with an Nvidia GPU and that can install Ubuntu 20.04 should work.

### Install [Ubuntu](https://ubuntu.com/)

Required version: 20.04

Install Ubuntu from its [official website](https://ubuntu.com/).

- Note: For dual boot system, remember to DISABLE [device encryption](https://support.microsoft.com/en-ca/help/4028713/windows-10-turn-on-device-encryption) before start installing Ubuntu.

Make sure your system packages are up to date:

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

### Install [CUDA Driver and Toolkit](https://developer.nvidia.com/cuda-toolkit)

Required version: 11.0

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

Required version: >=7.1.0

The instructions below follow the installation instructions [here](https://proj.org/install.html#compilation-and-installation-from-source-code). Download the [latest release](https://proj.org/download.html#current-release) first and extract it in to `~/ASRL/workspace`

```bash
sudo apt install cmake libsqlite3-dev sqlite3 libtiff-dev libcurl4-openssl-dev # dependencies
mkdir ~/ASRL/workspace/<extracted proj folder>/build && cd ~/ASRL/workspace/<extracted proj folder>/build
cmake ..
sudo cmake --build . --target install  # will install to /usr/local/[lib,bin]
```

### Install [OpenCV](https://opencv.org/)

Required version: >=4.4.0

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
# generate Makefile s
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

<!-- ```bash
cmake \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr \
  -DBUILD_PNG=OFF \
  -DBUILD_TIFF=OFF \
  -DBUILD_TBB=OFF \
  -DBUILD_JPEG=OFF \
  -DBUILD_JASPER=OFF \
  -DBUILD_ZLIB=OFF \
  -DBUILD_EXAMPLES=ON \
  -DBUILD_opencv_java=OFF \
  -DBUILD_opencv_python2=ON \
  -DBUILD_opencv_python3=OFF \
  -DENABLE_PRECOMPILED_HEADERS=OFF \
  -DWITH_OPENCL=OFF \
  -DWITH_OPENMP=ON \
  -DWITH_FFMPEG=ON \
  -DWITH_GSTREAMER=OFF \
  -DWITH_GSTREAMER_0_10=OFF \
  -DWITH_CUDA=ON \
  -DWITH_GTK=ON \
  -DWITH_VTK=OFF \
  -DWITH_TBB=ON \
  -DWITH_1394=OFF \
  -DWITH_OPENEXR=OFF \
  -DCUDA_TOOLKIT_ROOT_DIR=/usr/local/cuda-10.0 \
  -DCUDA_ARCH_BIN=7.5 \
  -DCUDA_ARCH_PTX="" \
  -DINSTALL_C_EXAMPLES=ON \
  -DINSTALL_TESTS=OFF \
  -DOPENCV_TEST_DATA_PATH=../../opencv_extra/testdata \
  -DOPENCV_ENABLE_NONFREE=ON \
  -DOPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules \
  ..
``` -->

### Install [ROS](https://www.ros.org/)

#### Install ROS1

Requied version: Noetic

We install ROS1 under `~/ASRL/workspace/ros_noetic`. Instructions follow the installation tutorial [here](http://wiki.ros.org/noetic/Installation/Source)

Add ROS1 repositories, 

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update
```

Install dependencies,

```bash
sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
```

Initialize rosdep,

```bash
sudo rosdep init
rosdep update
```

First download necessary ROS packages.

```bash
mkdir -p ~/ASRL/workspace/ros_noetic && cd ~/ASRL/workspace/ros_noetic  # root dir for ROS1
rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop-full.rosinstall
mkdir src
vcs import --input noetic-desktop-full.rosinstall ./src
rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="libopencv-dev python3-opencv" -y
```

- Note: we use `--skip-keys` option to skip installing opencv related tools, since we have already installed them from source.

Install ROS1 via `catkin_make_isolated`

```bash
./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space ~/ASRL/workspace/ros_noetic/install
```

**Important**: DO NOT run the following command. Normally you run the following command immediately after installing ROS1 to add path to its executables:

```bash
source ./install/setup.bash  # ROS 1 packages should always extend this workspace.
```

However, since now we also need to install ROS2, **DO NOT** run the above command after installation. If you have run it already, open a new terminal and continue.

#### Install ROS2

Required version: Foxy

VTR2 targets ROS1 while VTR3 targets ROS2. Currently, VTR3 is under active development, so we need to run both VTR2 and VTR3 on the same computer for testing purposes. Therefore, we install both ROS1 and ROS2, and use a ROS2 package called `ros1_bridge` to let ROS1 and ROS2 packages communicate with each other.

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
  2. we also do not install `ros1_bridge` package at this moment since it requires some other setup, as discussed below.

Important: same as for ROS1, DO NOT `source` the `setup.bash` script for now.

```bash
source ./install/setup.bash  # Run this command later (NOT now), and everytime after when you want to use ROS2.
```

#### Install ros1_bridge

Now we install the bridge between ROS1 and ROS2.

- The following instructions are based on [here](https://github.com/ros2/ros1_bridge).
- In fact, the instruction is put here only for completeness, you should install this package after you have installed VTR2 and VTR3. See the _important_ message below.

```bash
cd ~/ASRL/workspace/ros_foxy
# Source both ROS1 and ROS2 workspaces
source ~/ASRL/workspace/ros_noetic/install/setup.bash
source ~/ASRL/workspace/ros_foxy/install/setup.bash
# Now install ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

**Important**: currently `ros1_bridge` only works for the packages from the workspaces sourced by the time it is installed. Therefore, we have to reinstall this package every time we build a new workspace (either ROS1 or ROS2), which means that after we have installed VTR2 and VTR3, we need to rerun the above command again with the last workspace we have created:

```bash
cd ~/ASRL/workspace/ros_foxy
# Source both ROS1 and ROS2 workspaces
source <VTR2 workspace>/setup.bash
source <VTR3 workspace>/setup.bash
# Now install ros1_bridge
colcon build --symlink-install --packages-select ros1_bridge --cmake-force-configure
```

### Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)

This is a better version of `catkin_make` that is commonly used to build ROS1 packages. We use this tool to build VTR2 and its dependent packages. In ROS2, we use `colcon`, which is the default build tool for ROS2 packages.

```bash
sudo apt-get install python3-catkin-tools python3-osrf-pycommon
```

- Note: `python3-osrf-pycommon` is a dependency of catkin tools but not automatically installed due to a bug in catkin tools 0.5.0. TODO remove this once the bug is fixed.

### Install Third-Party ROS1 Packages (VTR2 Build)

Before continue, start a new terminal and source only the ROS1 workspace

```bash
source ~/ASRL/workspace/ros_noetic/install/setup.bash  # ROS 1 packages should always extend this workspace.
```

We install third-party ROS packages in the following directory

```bash
mkdir -p ~/charlottetown/extras/src
cd ~/charlottetown/extras/src
```

Now follow the instructions to download the repositories relating to your robot.

**The following sections for each robot are optional. You only need to install packages for a particular robot if you are going to use it, or use data that was gathered with that robot.** Don't have a specific robot you're working on? Follow the **grizzly** instructions.

- Grizzly

  Download source code.

  ```bash
  git clone https://github.com/utiasASRL/joystick_drivers.git
  git clone https://github.com/utiasASRL/grizzly.git -b vtr3_development  # tracking our development branch
  ```

  For the `joystick_driver` repository, you need to change the `package.xml` in its `wiimote` package to comment out the `python-cwiid` dependency and change `python-numpy` to `python3-numpy`. We cannot create new branches in this repo so the changes are not published.

  Install dependencies via rosdep for your ROS version

  ```bash
  rosdep install --from-paths ~/charlottetown/extras/src --ignore-src --rosdistro <your ROS distribution>
  ```

  - Note: The above command should not install any package that starts with `ros-` using `apt-get`.

If you downloaded any third party packages in the `extras/src` folder, then install them via `catkin build`

```bash
cd ~/charlottetown/extras
export UTIAS_ROS_DIR=~/charlottetown # not sure why this is needed
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/charlottetown/extras/devel/setup.bash
```

### Install utiasASRL robots library

We install the robots library in the following directory

```bash
mkdir -p ~/charlottetown/utiasASRL/robots
cd ~/charlottetown/utiasASRL/robots
```

Download the repo from github

```bash
git clone https://github.com/utiasASRL/robots.git src -b vtr3_development  # tracking our development branch
```

Install it via `catkin build`

```bash
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/charlottetown/utiasASRL/robots/devel/setup.bash
```

### Install VTR2.1

Install VTR system dependencies

```bash
sudo apt-get install cmake libprotobuf-dev protobuf-compiler libzmq3-dev \
  build-essential libdc1394-22 libdc1394-22-dev libpugixml1v5 libpugixml-dev \
  libgtest-dev
```

We install the vtr library in the following directory

```bash
mkdir -p ~/charlottetown/utiasASRL/vtr2
cd ~/charlottetown/utiasASRL/vtr2
```

Download vtr from github

```bash
git clone https://github.com/utiasASRL/vtr2.git src -b vtr3_development  # tracking our development branch
cd ~/charlottetown/utiasASRL/vtr2/src
# Submodule update the UI and deps.
git submodule update --init --remote  # add remote flag so that git automatically track the latest updates of the submodules.
```

**Important** `vtr3_development` branch has the following changes to support ubuntu >=16.04:

- Added the following two lines to the CMakeLists.txt of all vtr2 packages that contain the line `find_package(Eigen3 3.x.x REQUIRED)` (there are 12 total - 10 in asrl\_\_\* packages as well as LGmath and STEAM). We are unsure yet if this is needed on 20.04.

  ```bash
  add_definitions(-DEIGEN_DONT_VECTORIZE=1)
  add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)
  ```

  This degrades performance but prevents [Eigen alignment issues](http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html). Why this is an issue on some systems but not others is unknown.

- For the submodule gpusurf library, set the correct compute capability for Lenovo P53 GPU. You can look for it [here](https://developer.nvidia.com/cuda-gpus). `gpusurf/CMakeLists.txt` line 53, change _7.5_ to the version of CUDA you are using (e.g. _10.2_). On line 55, change the _compute_30_ and _sm_30_ values to the value on the nvidia webpage (minus the '.') (e.g. 7.5 becomes _compute_75_ and _sm_75_) and remove "_sm_50_".
- For the submodule robochunk, changed robochunk build type to debug in this file `~/charlottetown/utiasASRL/vtr2/src/deps/catkin/.catkin_tools/profiles/default/config.yaml`
- A lots small changes needed to get VTR2 compile on Ubuntu 18.04 and 20.04, which has a newer gcc/g++ version.

Now go to `deps` dir and install vtr dependencies (including robochunk)

```bash
cd ~/charlottetown/utiasASRL/vtr2/src/deps/catkin
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/deps/setup.bash
```

Install python dependencies

We install all python dependencies inside a python virtual environment so that they do not corrupt system python packages. **Important**: activate the virtualenv whenever you install things starting from this stage and run vtr.

```bash
sudo apt install python3-virtualenv
cd ~/ASRL && virtualenv venv --system-site-packages
source ~/ASRL/venv/bin/activate
pip install pyyaml pyproj scipy zmq socketIO_client flask flask_socketio
```

Install protobuf python packages via `pip`.

There are 3 in total.

```python
cd ~/charlottetown/utiasASRL/vtr2/devel/deps/.private/robochunk_msgs/robochunk_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/robo_sensors/setup/babelfish_non_primitive_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/src/robo_sensors/rtp/babel_rtp_robochunk_protopy
pip install -e .
```

Build robochunk translator (currently this has to be built after building the libs in `deps`)

```bash
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/
catkin init
catkin config --no-cmake-args
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Debug  # Note: A bug in robochunk, there is a confusing runtime error when setting build type to Release.
catkin build
source ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/setup.bash
```

- Note: it is weird that the original installation instruction did not source the setup.bash file of the extended translator workspace. Not sure why.

Same as above, install protobuf python packages.

```python
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/.private/babelfish_non_primitive/babelfish_non_primitive_protopy
pip install -e .
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/.private/babel_rtp_robochunk/babel_rtp_robochunk_protopy
pip install -e .
```

Build VTR

```bash
cd ~/charlottetown/utiasASRL/vtr2/src/
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/repo/setup.bash
```

**Important**: Currently there's a [bug](https://github.com/protocolbuffers/protobuf/issues/1491) with the generated python protobuf package (relative/absolute import). The current workaround is to manually change absolute import to relative import used in each python file generated from protobuf, through the `sed` command `sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py`. This is a ugly hack but should be enough for now since we won't be using robochunk in vtr3.

```bash
cd ~/charlottetown/utiasASRL/vtr2/devel/deps/.private/robochunk_msgs/robochunk_protopy/robochunk/proto
sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py
cd ~/charlottetown/utiasASRL/vtr2/src/asrl__messages/src/asrl__messages/proto
sed -i -r 's/^import (.*_pb2)/from . import \1/g' *_pb2*.py
sed -i -r 's/as .*_pb2 as/as/g' *_pb2*.py
```

- Note: There should be more protobuf generated scripts that need the above change, but I haven't found them all. If you encounter any python error that looks like: `Cannot import ...`, it should be the above problem.

### Clean-Up (This section is not needed for now, because I don't know where the following env vars are used.)

We are going to set up a more permanent source for the 1 workspace we have set up (ROS).

Open the bashrc file

```bash
gedit ~/.bashrc
```

Add the following to your bashrc file

```bash
###ASRL-ROS
#Set the ASRL Code directory:
export ASRL_CODE_DIR=~/asrl-misc-code

#Set the charlottetown root folder, it is assumed that all workspaces (ros_osrf, extras, utias, YOUR_LAB) and the scripts/rosinstalls/logs reside in this folder
export UTIAS_ROS_DIR=~/charlottetown

#Set the name of your lab workspace
export UTIAS_LAB_WS=asrl

#Add the helper scripts to the PATH:
export PATH="${UTIAS_ROS_DIR}"/scripts/:$PATH

#Set the data logging directory:
export ASRL_DATA_DIR="${UTIAS_ROS_DIR}"/logs

#Source the vtr2 workspace (which includes all parents)
. ~/charlottetown/utiasASRL/vtr2/devel/repo/setup.bash
#Announce the currently sourced version
echo "Sourced: ${UTIAS_ROS_DIR}"

# Catkin recursive cmd in src
function catkin_src_cmd () {
    test $# -lt 1 && echo "Must provide a command to run!" && return 1
    test -d src || echo "There is no 'src' subdirectory!"
    test -d src || return 1
    local command="cd {} && echo \"--- In: {} ---\" && $@"
    find ./src -mindepth 1 -maxdepth 1 -type d -exec sh -c "$command" \;
}
```

You are finished installing VTR2! You should now take a look at **asrl\_\_navigation** and **asrl\_\_offline_tools** and their top-level READMEs. To verify your installation is working and to get started with running VTR2, follow the follow the [First Run Tutorial](https://github.com/utiasASRL/vtr2/blob/develop/asrl__offline_tools/FirstRunTutorial.md) in [asrl\_\_offline_tools](https://github.com/utiasASRL/vtr2/tree/develop/asrl__offline_tools).

### Install VTR2.2

**Note**

1. VTR2.2 is not supposed to be usable. It only contains the ros1 version of vtr packages that will soon be ported to ros2.
2. For now, you must install VTR2.1 before installing VTR2.2.

Install VTR2.2 dependencies.

```
sudo apt install doxygen  # For building the documentations.
sudo apt install nodejs npm  # For building the user interface.
```

Clone this repo and then build it with Catkin.

```bash
cd ~/ASRL
git clone https://github.com/utiasASRL/vtr3.git
cd vtr3/ros1
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin build --catkin-make-args run_tests  # Build and run tests, note that you must run "catkin build" before this command.
catkin_test_results build  # Ensure that all unit tests pass.
source ~/ASRL/vtr3/ros1/devel/setup.bash
```

### Prepare VTR2.2 for launching

After installing VTR2.2, either add the following commands to `bashrc` or run them everytime before launching VTR3.

```bash
source ~/ASRL/venv/bin/activate  # Enter the venv we created for VTR3.
source ~/ASRL/vtr3/ros1/devel/setup.bash  # source the ros setup.
source $(rospack find vtr_navigation)/setup.bash  # define environment variables for VTR3.
```

You are finished installing VTR2.2. Now take a look at the documentations and tutorials below on how to use it.

### Install VTR3

**Note**: Before you install VTR3, make sure that you are in a new terminal **without** sourcing anything mentioned above!

Source ros2 installation

```bash
source ~/ASRL/workspace/ros_foxy/install/setup.bash
```

Build vtr3

```bash
cd vtr3/ros2
colcon build
colcon test --event-handlers console_cohesion+ # Will also run style check for c++, python, cmake and xml files.
colcon test-result  # Summary: 14 tests, 0 errors, 0 failures, 0 skipped
source ~/ASRL/vtr3/ros2/install/setup.bash
```

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
