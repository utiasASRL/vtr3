# vtr3
Make VT&amp;R Great Again

**Contents**
- [Installation](#installation)
- [Documentation](#documentation)
- [Contributing & Code of Conduct](#contributing-&-code-of-conduct)
- [License](#license)


## Installation

The following installation instruction is a more general version of vtr2 intallaltion instruction. This version mainly considers installing VT&R2 on systems with newer software and hardware, e.g. Ubuntu 18.04, CUDA 10+ and OpenCV 3.4/4.3+.

The following instructions should be kept as reference while we upgrade VT&R2 and port it to this new repo. Remember to add installation instructions for upgraded & ported code to the end of this section.

- Note:
  - VTR2 Install Notes: The old install notes can be found [here](https://github.com/utiasASRL/vtr2), which are mainly aimed at older laptops with Ubuntu 14.04. Some changes for 16.04 installation are mentioned. Additional (older) notes can be found on the [lab wiki](http://192.168.42.2/mediawiki/index.php/ROS:Charlottetown_Installation).

### Code Base Overview

The instructions will create a final code base layout as follows :

```
|- ~/charlottetown
    |- ros_osrf         ROS Jade Source Code
    |- extras           Third party dependencies
    |- utiasASRL        ASRL vtr2 code base & all its required libraries
        |- robots       ASRL robot-specific code
        |- vtr2/deps    VTR2 dependencies
        |- vtr2         VTR2 source code
```

VT&R2 Package list (from the vtr2 repository)
- [asrl__cmake](asrl__cmake) Build support such as custom CMake files.
- [asrl__common](asrl__common) Generic tools, such as timing, etc.
- [asrl__vision](asrl__vision) Vision-related tools, such as feature matching, RANSAC, etc.
- [asrl__pose_graph](asrl__pose_graph) Implements the pose graph used throughout vtr2.
- [asrl__steam_extensions](asrl__steam_extensions) Additions and enhancements to [steam](https://github.com/utiasASRL/steam) that are specific to VTR2.
- [asrl__terrain_assessment](asrl__terrain_assessment) Stereo terrain assessment package written by Peter Berczi.
- [asrl__navigation](asrl__navigation) High-level package that contains the Navigator: the primary binary used to run vtr2 live, its modules, launch files and params.
- [asrl__offline_tools](asrl__offline_tools) High-level package that contains tools for local testing using pre-gathered data. These tools are a little less complicated than running the Navigator.
- Note: Not guaranteed to be complete, browse top-level directories for a complete package list.

VT&R3 Package list (in this repository)
- [vtr_documentation](src/vtr_documentation) Generate VT&R3 documentation via Doxygen
- Note:
  - TODO: I named the repo vtr_* instead of asrl__* to differentiate the old and new packages. Fix this later.


### Hardware Requirement

TODO - minimum + recommended hardware requirements including CPU, GPU, etc

### Install [Ubuntu](https://ubuntu.com/)

VTR2 targets Ubuntu 14.04 and Ubuntu 16.04, while VTR3 targets Ubuntu 18.04 and newer.

- Note: at time of writing, Ubuntu 18.04 is the newest possible version due to the system requirement of [ROS Melodic](https://ros.org/reps/rep-0003.html).

Install Ubuntu from its [official website](https://ubuntu.com/).

Make sure your system packages are up to date:

```
sudo apt-get update
sudo apt-get upgrade
sudo apt-get dist-upgrade
```

### Install [CUDA Driver and Toolkit](https://developer.nvidia.com/cuda-toolkit)

VTR2 targets CUDA 7.5 (for Ubuntu 14.04) and CUDA 8.0 (for Ubuntu 16.04), while VTR3 targets CUDA 10.0 and newer.

Install CUDA from its [official website](https://developer.nvidia.com/cuda-toolkit)

- Note: you can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`. It is possible that these two commands report different CUDA version, which means that your CUDA driver and toolkit version do not match. This is OK as long as the driver and toolkit are compatible, which you can verify in their documentation.
- Note: at time of writing, the newest CUDA version is 10.2.

### Install [OpenCV](https://opencv.org/)

- Note: I followed the instructions from [here](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/).

Before installing OpenCV, make sure that it is not already installed in the system.

```
sudo apt list --installed | grep opencv*
```
Remove any OpenCV related packages and packages dependent on them.

Also make sure that you have python2+numpy installed in your system
```
python --version  # check python version
pip list  # make sure numpy is there, otherwise install with pip
```

Install OpenCV dependencies

- Ubuntu 18.04
  ```
  sudo apt install build-essential cmake git pkg-config libgtk-3-dev \
      libavcodec-dev libavformat-dev libswscale-dev libv4l-dev \
      libxvidcore-dev libx264-dev libjpeg-dev libpng-dev libtiff-dev \
      gfortran openexr libatlas-base-dev python3-dev python3-numpy \
      libtbb2 libtbb-dev libdc1394-22-dev
  ```

Download OpenCV and OpenCV Contrib from GitHub to the following directory: `~/workspace`

```
mkdir ~/workspace && cd ~/workspace
git clone https://github.com/opencv/opencv.git
git clone https://github.com/opencv/opencv_contrib.git
```

Checkout the corresponding branch of the version you want to install
```
cd ~/workspace/opencv && git checkout <opencv-version>  # 3.4.9 at time of writing
cd ~/workspace/opencv_contrib && git checkout <opencv-version>  # 3.4.9 at time of writing
```

- Note: we need `xfeatures2d` library from opencv_contrib.

Build and install OpenCV
```
mkdir -p ~/workspace/build && cd ~/workspace/build  # create build directory
# generate Makefile s
cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr/local \
      -D INSTALL_C_EXAMPLES=ON \
      -D INSTALL_PYTHON_EXAMPLES=ON \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D OPENCV_EXTRA_MODULES_PATH=~/workspace/opencv_contrib/modules \
      -D BUILD_EXAMPLES=ON ../opencv
make -j<nproc>  # <nproc> is number of cores of your computer
sudo make install  # copy libraries to /usr/local/[lib, include]
# verify your opencv version
pkg-config --modversion opencv
python -c "import cv2; print(cv2.__version__)"  # for python 2
python3 -c "import cv2; print(cv2.__version__)"  # for python 3
```

### Install [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)

This is just a better version of `catkin_make`. We use it to install ROS, vtr and its dependencies.
```
sudo apt-get install python-catkin-tools
```
- TODO: may need to add keys to apt

### Install [ROS](https://www.ros.org/)

We also install ROS from source, following its official [tutorial/documentation](http://wiki.ros.org/melodic/Installation/Source).

- Note: at time of writing, the latest ROS version is melodic. vtr2 targets kinectic/jade.

We install ROS, vtr3 and its dependencies under `~/charlottetown`

```
mkdir ~/charlottetown && cd ~/charlottetown  # root dir for ROS, vtr3 and its dependencies
mkdir ros_osrf && cd ros_osrf  # root dir for ROS
```

Install ROS dependencies and rosdep
```
sudo apt-get install python-rosdep python-rosinstall-generator python-wstool python-rosinstall build-essential
sudo rosdep init
rosdep update
```
- Melodic
  ```
  rosinstall_generator desktop_full --rosdistro melodic --deps --tar > melodic-desktop-full.rosinstall
  wstool init -j8 src melodic-desktop-full.rosinstall
  rosdep install --from-paths src --ignore-src --rosdistro kinetic --skip-keys=libopencv-dev --skip-keys=python-opencv -y
  ```
  - Note: we use `--skip-keys` option to skip installing opencv related tools, since we have already installed them from source.

Install ROS via `catkin build` and use it as the base catkin workspace
```
# Install
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --install
catkin build
# Use the installed workspace as base
source ./install/setup.bash
```
- Note: this is a standard building flow of `catkin build`, and we will use this flow multiple times to install vtr stuff. It is important to remember that you need to `source` the newly installed/extended workspace everytime after you install some libraries using `catkin build`, to make sure that the newly installed packages are visible.

### Install Third-Party ROS Packages

We install third-party ROS packages in the following directory
```
mkdir -p ~/charlottetown/extras/src
cd ~/charlottetown/extras/src
```

Now follow the instructions to download the repositories relating to your robot.

**The following sections for each robot are optional. You only need to install packages for a particular robot if you are going to use it, or use data that was gathered with that robot.** Don't have a specific robot you're working on? Follow the **grizzly** instructions.

- Grizzly

  Download source code
  ```
  git clone https://github.com/utiasASRL/joystick_drivers.git
  git clone https://github.com/utiasASRL/grizzly.git
  ```
  Install dependencies via rosdep for your ROS version
  ```
  rosdep install --from-paths ~/charlottetown/extras/src --ignore-src --rosdistro melodic
  ```

- Note:
  - If rosdep asks you to confirm installing a library, please accept (press "y").
  - If rosdep says that a library cannot be authenticated, please accept (press "y").
  - MAKE SURE YOU DON'T INSTALL ANY PACKAGES THAT BEGIN WITH ros-jade OR ros-kinetic USING APT-GET. If this is occuring, you will need to install the package from source first, as installing such a package with apt-get will likely install all the _other_ dependencies that should already be available in the ```ros_osrf``` folder. A suitable interim measure is to install these dependent packages from source in the ```extras``` folder before trying to install dependencies again.

If you downloaded any third party packages in the `extras/src` folder, then install them via `catkin build`

```
cd ~/charlottetown/extras
export UTIAS_ROS_DIR=~/charlottetown # not sure why this is needed
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

### Install utiasASRL robots library

We install the robots library in the following directory
```
mkdir -p ~/charlottetown/robots
cd ~/charlottetown/robots
```

Download the repo from github
```
git clone https://github.com/utiasASRL/robots.git src
```

Install it via `catkin build`
```
catkin init
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash
```

### Install VTR

Install VTR system dependencies

- Ubuntu 18.04
  ```
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

We install the vtr library in the following directory
```
mkdir -p ~/charlottetown/vtr2
cd ~/charlottetown/vtr2
```

Download vtr from github
```
git clone https://github.com/utiasASRL/vtr2.git src
cd ~/charlottetown/utiasASRL/vtr2/src
# Checkout the development branch
git checkout develop
# Submodule update the UI and deps.
git submodule update --init
```

Go to `deps` dir and install vtr dependencies (including robochunk)
```
cd ~/charlottetown/utiasASRL/vtr2/src/deps/catkin
```
- Note:
  1. For gpusurf library, you need to set the correct compute capability for your GPU. Look for it [here](). Open `gpusurf/CMakeLists.txt` line 55 and change the _compute_30_ and _sm_30_ values to the value on the nvidia webpage (minus the '.') (e.g. 5.2 becomes _compute_52_ and _sm_52_). This ensures that gpuSURF is compiled to be compatible with your GPU.

```
catkin build
source ~/charlottetown/utiasASRL/vtr2/devel/deps/setup.bash
```

- Note:
  - Depends on your c++ compiler version (which determines your c++ standard and is determined by your Ubuntu version), you may encounter compiler errors such as certain functions/members not defined under `std` namespace. Those should be easy to fix. Just google the function and find which header file should be included in the source code.

Build robochunk translator (currently this has to be built after building the libs in `deps`)

```
cd ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/
catkin config --no-cmake-args
catkin config -a --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source ~/charlottetown/utiasASRL/vtr2/build/deps/robochunk_babelfish_generator/translator/robochunk/devel/setup.bash
```
- Note: it is weird that the original installation instruction did not source the setup.bash file of the extended translator workspace. Not sure why.

Build VTR
```
cd ~/charlottetown/utiasASRL/vtr2/src/
catkin build
source ../devel/repo/setup.bash
```
- Note:
  - Again, depends on your c++ compiler version (which determines your c++ standard and is determined by your Ubuntu version), you may encounter compiler errors such as certain functions/members not defined under `std` namespace. Those should be easy to fix. Just google the function and find which header file should be included in the source code.
  - Currently, the asrl__terrain_assessment package may fail on Ubuntu 18.04 due to a weird `make` parse error, which we are still investigating. This will also cause `cakin build` to skip installing any package depending on asrl__terrain_assessment.

### Clean-Up

We are going to set up a more permanent source for the 1 workspace we have set up (ROS)

Add the following to your bashrc file

```
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

You are finished installing VTR2! You should now take a look at **asrl__navigation** and **asrl__offline_tools** and their top-level READMEs. To verify your installation is working and to get started with running VTR2, follow the [First Run Tutorial](asrl__offline_tools/FirstRunTutorial.md) in [asrl__offline_tools](asrl__offline_tools)

### Install VTR3 (this repo)

- Note: before we finishing upgrading VT&R2 and porting it to this repo, you may want to install VT&R2 first so that you can use functions from the old packages for testing purposes.

Clone this repo and then
```
cd <root folder of this repo>
catkin init
catkin build
source devel/setup.bash
```

- Note: if you want to build and install documentation, then use run the following command to install the packages.
  ```
  cd <root folder of this repo>
  catkin init
  catkin config --install
  catkin build
  source devel/setup.bash
  ```
### Install VTR2 on Lenovo P53 with Ubuntu 16.04

- Note: These instructions are only for getting a partially working version of VTR2 working on new laptops for testing/developing. They should be ignored by the vast majority of users.

In general, follow the instructions on [this branch](https://github.com/utiasASRL/vtr2/blob/install_on_ubuntu1604_x86/README.md).
There are a few differences related to the newer hardware and GPU we highlight here. 
First, we require CUDA 10.0+; we used CUDA 10.0. See [above](https://github.com/utiasASRL/vtr3#install-cuda-driver-and-toolkit).
The instructions note a conflict with Eigen and OpenCV requiring specific versions of the Eigen library for different laptops. 
We installed Eigen 3.3.4 from source. It passes the OpenCV core tests but still causes issues we address later on.
Next install OpenCV 3.4.10 from source. We used the following CMake flags adapted from the lab wiki instructions:
```
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
```
Continue with the ROS Kinetic installation, noting the step to disable building the ROS-packaged opencv3 so it finds our version.
Continue with the rest of the steps. Make sure to assign the correct compute capability before building GPUsurf.
You will also need to add the following two lines to the CMakeLists.txt of all vtr2 packages that contain the line `find_package(Eigen3 3.2.2 REQUIRED)` (there are 12 total - 10 in asrl__* packages as well as LGmath and STEAM):
```
add_definitions(-DEIGEN_DONT_VECTORIZE=1) 		
add_definitions(-DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT=1)
```
This degrades performance but prevents [Eigen alignment issues](http://eigen.tuxfamily.org/dox-devel/group__TopicUnalignedArrayAssert.html).
Why this is an issue on some systems but not others is unknown. Once, you have successfully installed VTR2, try the [First Run Tutorial](https://github.com/utiasASRL/vtr2/blob/install_on_ubuntu1604_x86/asrl__offline_tools/FirstRunTutorial.md)!

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

## Resources

TODO

## License

TODO

