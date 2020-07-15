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
    - [Change the default gcc/g++ version to 8.0](#change-the-default-gccg-version-to-80)
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

The following installation instruction is a more general version of vtr2 installation instruction. This version mainly considers installing VT&R2 on systems with newer software and hardware, e.g. Ubuntu 20.04, CUDA 10.0+ and OpenCV 4.0+.

The following instructions should be kept as reference while we upgrade VT&R2 and port it to this new repo. Remember to add installation instructions for upgraded & ported code to the end of this section.

- Note:
  - VTR2 Install Notes: The old install notes can be found [here](https://github.com/utiasASRL/vtr2), which are mainly aimed at older laptops with Ubuntu 14.04. Some changes for 16.04 installation are mentioned. Additional (older) notes can be found on the [lab wiki](http://192.168.42.2/mediawiki/index.php/ROS:Charlottetown_Installation).

### Code Base Overview

The instructions will create a final code base layout as follows in Ubuntu 20.04:

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
        |- proj-7.0.1      the newest version of PROJ, which is required by VTR2&3
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
- Note:
  - TODO: I named the repo vtr\_\* instead of asrl\_\_\* to differentiate the old and new packages. Fix this later.

### Hardware Requirement

Currently we are only running VTR3 on Lenovo P53 laptops. But technically any computer with an Nvidia GPU and that can install Ubuntu 20.04 should work.

### Install [Ubuntu](https://ubuntu.com/)

VTR2 targets Ubuntu 14.04 and Ubuntu 16.04, while VTR3 targets Ubuntu 20.04.

Install Ubuntu from its [official website](https://ubuntu.com/).

- Note: For dual boot system, remember to DISABLE [device encryption](https://support.microsoft.com/en-ca/help/4028713/windows-10-turn-on-device-encryption) before start installing Ubuntu.

Make sure your system packages are up to date:

```bash
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

### Install [CUDA Driver and Toolkit](https://developer.nvidia.com/cuda-toolkit)

VTR2 targets CUDA 7.5 (for Ubuntu 14.04) and CUDA 8.0 (for Ubuntu 16.04), while VTR3 targets CUDA 10.0 and newer.

Install CUDA through Debian package manager from its [official website](https://developer.nvidia.com/cuda-toolkit)

- Note:
  - At time of writing, the newest CUDA version is 10.2.
  - **Ubuntu 20.04**: CUDA 10.2 does not officially support this version of Ubuntu, but you can just follow the [installation guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html) for ubuntu 18.04. Be sure to perform the necessary [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions). **TODO**: this will change when newer version of CUDA comes out. Update this accordingly.
  - you can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`. It is possible that these two commands report different CUDA version, which means that your CUDA driver and toolkit version do not match. This is OK as long as the driver and toolkit are compatible, which you can verify in their documentation.

Optional: Install CuDNN 7.6.5 through Debian package manager from its [official website](https://developer.nvidia.com/cudnn)

### Change the default gcc/g++ version to 8.0

- **Ubuntu 20.04**
  - Ubuntu 20.04 comes with gcc/g++ 9, but CUDA 10.2 does not work with gcc/g++ 9, so we need to change the system default gcc/g++ version. Follow the tutorial [here](https://linuxconfig.org/how-to-switch-between-multiple-gcc-and-g-compiler-versions-on-ubuntu-20-04-lts-focal-fossa) and switch the default gcc/g++ version to 8.
  - **TODO**: this may change in the future, keep an eye on the updates of CUDA.

### Change default python version to python3

- **Ubuntu 20.04**

  - We do not need python 2 on ubuntu 20.04, so change the default python version to python 3.

    ```bash
    sudo apt install python-is-python3
    ```

    This is needed because we want to use python3 to install everything, but some packages default to use python from `/usr/bin/python` which is python2.

### Install Eigen

- **Ubuntu 20.04 and 18.04**

  - Install Eigen 3.3.7 from package manager

  ```bash
  sudo apt install libeigen3-dev
  ```

  - Note: Eigen is an optional library for OpenCV, but is required by VTR3.

- **Ubuntu 16.04**
  - The instructions note a conflict with Eigen and OpenCV requiring specific versions of the Eigen library for different laptops.
  - (We installed Eigen 3.3.4 from source. It passes the OpenCV core tests but still causes issues we address later on.)

### Install [PROJ](https://proj.org/)

This package is needed by some of the VTR2 packages.

- **Ubuntu 20.04 and 18.04**

  - Install PROJ 7.0.1 from source because `libproj0` is not available in these Ubuntu versions.
  - The instructions below follow the installation instructions [here](https://proj.org/install.html#compilation-and-installation-from-source-code)
  - Download the [latest release](https://proj.org/download.html#current-release) (7.0.1 at time of writing) first and extract it in to `~/ASRL/workspace`

  ```bash
  sudo apt install sqlite3  # dependency of PROJ
  mkdir ~/ASRL/workspace/proj-7.0.1/build && cd ~/ASRL/workspace/proj-7.0.1/build  # 7.0.1 is the version name
  cmake ..
  sudo cmake --build . --target install  # will instal to /usr/local/[lib,bin]
  ```

  - Note:
    1. `libproj0` can also be found [here](https://packages.debian.org/jessie/libproj0). But if the above one works, this is not needed.
    2. If later you want to uninstall PROJ, just remove files recorded in `build/install_manifest.txt`.

- **Ubuntu 16.04-**
  - Install `lbproj0` using apt should be fine.

### Install [OpenCV](https://opencv.org/)

Install OpenCV from source, and get code from its official Github repository as listed below.

The following OpenCV install instructions refer to the instructions from [here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).

The following instructions only consider Ubuntu 18.04 and 20.04. Instructions for older Ubuntu versions can be found in the original VTR2 repository.

Before installing OpenCV, make sure that it is not already installed in the system.

```bash
sudo apt list --installed | grep opencv*
```

Remove any OpenCV related packages and packages dependent on them.

Install OpenCV dependencies based on [here](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html).

- **Ubuntu 20.04**

  ```bash
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
  ```

  - Note: there are some more optional packages to be installed but not all of them are available in 20.04. We should check what we actually need.

- **Ubuntu 18.04**

  ```bash
  sudo apt install build-essential
  sudo apt install cmake git libgtk-3-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
  sudo apt install libv4l-dev libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev gfortran openexr libatlas-base-dev libtbb2 libtbb-dev libdc1394-22-dev
  ```

Download OpenCV and OpenCV Contrib from GitHub to the following directory: `~/ASRL/workspace`

```bash
mkdir -p ~/ASRL/workspace && cd ~/ASRL/workspace
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Checkout the corresponding branch of the version you want to install

```bash
cd ~/ASRL/workspace/opencv && git checkout <opencv-version>  # <opencv-version> = 4.3.0 at time of writing
cd ~/ASRL/workspace/opencv_contrib && git checkout <opencv-version>  # <opencv-version> = 4.3.0 at time of writing
```

- **TODO**: currently we need `xfeatures2d` library from opencv_contrib but this may change in the future, so keep an eye on the updates of OpenCV.

Build and install OpenCV

```bash
mkdir -p ~/ASRL/workspace/opencv/build && cd ~/ASRL/workspace/opencv/build  # create build directory
```

- **Ubuntu 20.04**

  ```bash
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
  ```

- **Ubuntu 18.04 and older**

  ```bash
  # generate Makefile s
  cmake -D CMAKE_BUILD_TYPE=RELEASE \
        -D CMAKE_INSTALL_PREFIX=/usr/local \
        -DBUILD_opencv_python2=ON \
        -DBUILD_opencv_python3=ON \
        -DWITH_OPENMP=ON \
        -DWITH_CUDA=ON \
        -DOPENCV_ENABLE_NONFREE=ON \
        -D OPENCV_GENERATE_PKGCONFIG=ON \
        -DWITH_TBB=ON \
        -DWITH_GTK=ON \
        -DWITH_OPENMP=ON \
        -DWITH_FFMPEG=ON \
        -D OPENCV_EXTRA_MODULES_PATH=~/ASRL/workspace/opencv_contrib/modules \
        -DBUILD_opencv_cudacodec=OFF \ # for cuda 10 and opencv 3.4.9, this module only work with cuda 7.5-
        -D BUILD_EXAMPLES=ON ..
  ```

- Note: the differences between 20.04 and 18.04- are: 1. 20.04 uses python3.8 for build; 2. 20.04 does not build opencv for python2. These are specified in flags.

```bash
make -j<nproc>  # <nproc> is number of cores of your computer, 12 for Lenovo P53
sudo make install  # copy libraries to /usr/local/[lib, include]
# verify your opencv version
pkg-config --modversion opencv4
python -c "import cv2; print(cv2.__version__)"  # for python 2, only for Ubuntu 18.04 and older
python3 -c "import cv2; print(cv2.__version__)"  # for python 3
```

- Note: it looks like we used to use the following flags when compiling OpenCV. We do not know whether these flags are still needed for OpenCV 3+.

  ```bash
  cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX=/usr/local \
    -DBUILD_PNG=OFF \
    -DBUILD_TIFF=OFF \
    -DBUILD_TBB=OFF \
    -DBUILD_JPEG=OFF \
    -DBUILD_JASPER=OFF \
    -DBUILD_ZLIB=OFF \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_opencv_java=OFF \
    -DBUILD_opencv_python2=OFF  # no longer build in Ubuntu 20.4 \
    -DBUILD_opencv_python3=ON   # build python3 instead \
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
    -DINSTALL_C_EXAMPLES=ON \
    -DINSTALL_PYTHON_EXAMPLES=ON \
    -DOPENCV_ENABLE_NONFREE=ON \
    -DOPENCV_EXTRA_MODULES_PATH=~/ASRL/workspace/opencv_contrib/modules \
    ..
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

We install ROS1 under `~/ASRL/workspace/ros_noetic`, we just use the name `noetic` here because it is the version being installed to Ubuntu 20.04. If you use a different version of Ubuntu, feel free to change that to the corresponding name of the ROS distribution.

- Note: at time of writing, the latest ROS1 version is noetic. vtr2 targets kinectic/jade.

The following instructions follow the installation tutorial [here](http://wiki.ros.org/noetic/Installation/Source)

```bash
mkdir -p ~/ASRL/workspace/ros_noetic && cd ~/ASRL/workspace/ros_noetic  # root dir for ROS1
```

Install ROS dependencies and rosdep

- **Ubuntu 20.04**

  ```bash
  sudo apt-get install python3-rosdep python3-rosinstall-generator python3-vcstool build-essential
  ```

- **Ubuntu 18.04 and older**

  ```bash
  sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
  ```

If apt cannot find one or more of these packages, you may need to run the following before trying the command above again.

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://pool.sks-keyservers.net --recv-key 0xAB17C654
sudo apt-get update
```

After installing the dependencies above,

```bash
sudo rosdep init
rosdep update
```

- **Ubuntu 20.04** -> ROS Noetic

  First download necessary ROS packages.

  ```bash
  rosinstall_generator desktop_full --rosdistro noetic --deps --tar > noetic-desktop-full.rosinstall
  mkdir ./src
  vcs import --input noetic-desktop-full.rosinstall ./src
  rosdep install --from-paths src --ignore-src --rosdistro noetic --skip-keys="libopencv-dev python3-opencv" -y
  ```

- **Ubuntu 18.04** -> ROS Melodic

  ```bash
  rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall
  wstool init -j8 src melodic-desktop-full.rosinstall
  rosdep install --from-paths src --ignore-src --rosdistro melodic --skip-keys="libopencv-dev python-opencv" -y
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

VTR2 targets ROS1 while VTR3 targets ROS2. Currently, VTR3 is under active development, so we need to run both VTR2 and VTR3 on the same computer for testing purposes. Therefore, we install both ROS1 and ROS2, and use a ROS2 package called `ros1_bridge` to let ROS1 and ROS2 packages communicate with each other.

- Note: at time of writing, the latest ROS2 version is foxy.

The instructions follow the installation tutorial [here](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/)

```bash
mkdir -p ~/ASRL/workspace/ros_foxy && cd ~/ASRL/workspace/ros_foxy  # root dir for ROS2
```

Install ROS2 dependencies

- Note: the following commands are copied from the tutorial link above. No change needed for our case.

```bash
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt update && sudo apt install curl gnupg2 lsb-release
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'
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

- **Ubuntu 20.04**:

  - We have to install this from source because the current latest release still has some dependencies on python2 packages. We roughly followed the instructions [here](https://catkin-tools.readthedocs.io/en/latest/installing.html).

    ```bash
    sudo apt install python3-catkin-pkg python3-catkin-pkg-modules  # should already been installed
    pip3 install sphinxcontrib-programoutput osrf-pycommon # install using pip since it is not available in apt
    pip3 install git+https://github.com/catkin/catkin_tools.git
    ```

    - Note: once we can install this package from apt, make sure that you uninstall the package and the two dependencies using pip first.

  - **TODO**: Once catkin-tools can be installed from package manager, we must uninstall this via pip first then reinstall it through package manager.

- **Ubuntu 18.04 and older**

  ```bash
  sudo apt-get install python-catkin-tools
  ```

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

  Download source code

  ```bash
  git clone https://github.com/utiasASRL/joystick_drivers.git  # Not working on Ubuntu 20.04, so ask catkin to ignore this directory for now.
  git clone https://github.com/utiasASRL/grizzly.git -b vtr3_development  # tracking our development branch
  ```

  Install dependencies via rosdep for your ROS version

  ```bash
  rosdep install --from-paths ~/charlottetown/extras/src --ignore-src --rosdistro <your ROS distribution>
  ```

- Note:
  - If rosdep asks you to confirm installing a library, please accept (press "y").
  - If rosdep says that a library cannot be authenticated, please accept (press "y").
  - MAKE SURE YOU DON'T INSTALL ANY PACKAGES THAT BEGIN WITH ros- USING APT-GET. If this is occuring, you will need to install the package from source first, as installing such a package with apt-get will likely install all the _other_ dependencies that should already be available in the `ros_osrf` folder. A suitable interim measure is to install these dependent packages from source in the `extras` folder before trying to install dependencies again.

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

### Install VTR

Install VTR system dependencies

- **Ubuntu 20.04 and 18.04**

  ```bash
  sudo apt-get install cmake libprotobuf-dev protobuf-compiler libzmq3-dev \
    build-essential libdc1394-22 libdc1394-22-dev libpugixml1v5 libpugixml-dev \
    libgtest-dev
  # Gtest post apt-get install configuration
  cd /usr/src/googletest/googletest
  sudo mkdir build && cd build
  sudo cmake ..
  sudo make
  sudo cp libgtest* /usr/lib/
  cd .. && sudo rm -rf build
  ```

- **Ubuntu 16.04 and older**
  - Check the guide for vtr2.

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

**Important** Ubuntu 16.06, 18.04 and probably 20.04: you will likely need to add the following two lines to the CMakeLists.txt of all vtr2 packages that contain the line `find_package(Eigen3 3.x.x REQUIRED)` (there are 12 total - 10 in asrl\_\_\* packages as well as LGmath and STEAM). We are unsure yet if this is needed on 20.04.

```bash
add_definitions(-DEIGEN_DONT_VECTORIZE=1)
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)
```

This degrades performance but prevents [Eigen alignment issues](http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html). Why this is an issue on some systems but not others is unknown.

Besides the above change, there are still a lots small changes needed to get VTR2 compile on Ubuntu 18.04 and 20.04. Check the diff files in [vtr2_diffs](./vtr2_diffs) to see all the changes. Note: these changes should not have to be done manually if you are using the `vtr3_development` branch of VTR2.

Now go to `deps` dir and install vtr dependencies (including robochunk)

```bash
cd ~/charlottetown/utiasASRL/vtr2/src/deps/catkin
```

- Note:
  1. For gpusurf library, you need to set the correct compute capability for your GPU. Look for it [here](https://developer.nvidia.com/cuda-gpus). Open `gpusurf/CMakeLists.txt`. On line 53, change _7.5_ to the version of CUDA you are using (e.g. _10.2_). On line 55, change the _compute\_30_ and _sm\_30_ values to the value on the nvidia webpage (minus the '.') (e.g. 7.5 becomes _compute\_75_ and _sm\_75_) and remove "_sm\_50_". This ensures that gpuSURF is compiled to be compatible with your GPU.
  2. **Ubuntu 20.04**: change robochunk build type to debug in this file `~/charlottetown/utiasASRL/vtr2/src/deps/catkin/.catkin_tools/profiles/default/config.yaml`

```bash
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/deps/setup.bash
```

- Note:
  - Depends on your c++ compiler version (which determines your c++ standard and is determined by your Ubuntu version), you may encounter compiler errors such as certain functions/members not defined under `std` namespace. Those should be easy to fix. Just google the function and find which header file should be included in the source code. E.g. Googling _mt19937_ tells you that _\<random\>_ should be included in the file causing the associated error.

Install python dependencies

We install all python dependencies inside a python virtual environment so that they do not corrupt system python packages. **Important**: activate the virtualenv whenever you install things starting from this stage and run vtr.

```bash
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

- Note:
  - Again, depends on your c++ compiler version (which determines your c++ standard and is determined by your Ubuntu version), you may encounter compiler errors such as certain functions/members not defined under `std` namespace. Those should be easy to fix. Just google the function and find which header file should be included in the source code.
  - Currently, the asrl\_\_terrain_assessment package may fail on Ubuntu 18.04 due to a weird `make` parse error, which we are still investigating. This will also cause `cakin build` to skip installing any package depending on asrl\_\_terrain_assessment.

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

### Install VTR3 (this repo)

- Note: before we finishing upgrading VT&R2 and porting it to this repo, you may want to install VT&R2 first so that you can use functions from the old packages for testing purposes.

Clone this repo and then build it with Catkin.

```bash
cd ~/ASRL
git clone https://github.com/utiasASRL/vtr3.git
cd vtr3
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
catkin build --catkin-make-args run_tests  # Build and run tests, note that you must run "catkin build" before this command.
catkin_test_results build  # Ensure that all unit tests pass.
source ~/ASRL/vtr3/devel/setup.bash
```

### Prepare VTR3 for launching

After installing VTR3, either add the following commands to `bashrc` or run them everytime before launching VTR3.

```bash
source ~/ASRL/venv/bin/activate  # Enter the venv we created for VTR3.
source ~/ASRL/vtr3/devel/setup.bash  # source the ros setup.
source $(rospack find vtr_navigation)/setup.bash  # define environment variables for VTR3.
```

You are finished installing VTR3. Now take a look at the documentations and tutorials below on how to use it.

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
