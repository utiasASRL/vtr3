## Hardware Recommendation

### Computation

VT&amp;R3 requires a high-powered, NVIDIA GPU-enabled machine that is able to run [Ubuntu 20.04](https://ubuntu.com/). Example:

- [Lenovo Notebook ThinkPad P53](https://www.lenovo.com/ca/en/laptops/thinkpad/thinkpad-p/P53/p/22WS2WPWP53): Intel Core i7-9750H Processor, NVIDIA Quadro T2000 4GB GPU, 32GB DDR4 RAM

### Sensor

VT&amp;R3 requires either a stereo camera or a spinning LiDAR. Examples:

- stereo camera: [FLIR Bumblebee XB3 FireWire](https://www.flir.ca/support/products/bumblebee-xb3-firewire/#Overview)
- LiDAR: Waymo Honeycomb, [Velodyne Alpha Prime](https://velodynelidar.com/products/alpha-prime/)

See [this page](./new_sensor_robot.md) for how to use VT&amp;R3 with your own camera/LiDAR sensors.

### Robot

VT&amp;R3 communicates with the underlying platform only through the kinematic controller inputs of the system and can be run on robots that take ROS geometry_msgs/twist message as control command.

## Set up VT&amp;R3 directories

The following environment variables are assumed present so that files and data can be put into different locations on different machines. It is recommended to put them in `.bashrc`.

```bash
export VTRROOT=~/ASRL  # (INTERNAL default) root directory of VTR3 (this variable only initializes the following variables and will not be used anywhere else)
# you can change the following directories to anywhere appropriate
export VTRSRC=${VTRROOT}/vtr3        # source code of VTR3 (this repo)
export VTRDEPS=${VTRROOT}/workspace  # system dependencies of VTR3
export VTRVENV=${VTRROOT}/venv       # python dependencies of VTR3 (not used at the moment)
export VTRDATA=${VTRROOT}/data       # datasets for VTR3
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
    |- drivers           sensor, controller drivers not specific to VTR3
    |- extra             VTR3 specific sensor, robot, dataset specific add-ons
    |- launch            tmuxp launch files
    |- main              main packages of VTR3, must be installed to get a working system
    |- robots            robot description packages not specific to VTR3
  |- workspace         system dependencies source code and (maybe) installation
    |- opencv            opencv source code cloned from github, installed to /usr/local/[lib,bin]
    |- opencv_contrib    extra opencv source code cloned from github, installed together with opencv
    |- proj              the latest version of PROJ, installed to /usr/local/[lib,bin]
    |- vtr_ros2_deps     VTR3 dependencies from public repositories without modification
```

## Download VT&amp;R3 source code

```
cd ${VTRSRC}
git clone git@github.com:utiasASRL/vtr3.git .
git submodule update --init --remote
```

Machine specific settings

- Change [Nvidia GPU compute capability](https://developer.nvidia.com/cuda-gpus) in [gpusurf](../main/src/deps/gpusurf/gpusurf/CMakeLists.txt) line 16 based on your GPU model (default to 7.5).
- Change `OpenCV_DIR` in [gpusurf](../main/src/deps/gpusurf/gpusurf/CMakeLists.txt) line 21 and [vtr_common](../main/src/vtr_common/vtr_include.cmake) line 48 to point to your OpenCV+CUDA installation (default to `/usr/local/opencv_cuda/lib/cmake/opencv4`). If you do not have an OpenCV+CUDA installation, keep the default value for now, continue to the next section and we will eventually install OpenCV to this location [here](#install-opencv-450).

## Install system dependencies

```bash
# Dependencies from Debian packages
sudo apt install -y tmux  # for launching VTR3
sudo apt install -y doxygen  # for building the documentation
sudo apt install -y nodejs npm protobuf-compiler  # for building the VTR web-based graphical user interface
sudo apt install -y libboost-all-dev libomp-dev  # boost and openmp, needed by multiple packages
sudo apt install -y libpcl-dev  # point cloud library, for LiDAR VTR
sudo apt install -y libcanberra-gtk-module libcanberra-gtk3-module # for camera VTR image playback (see: https://github.com/utiasASRL/vtr3/issues/107)
sudo apt install -y libdc1394-22 libdc1394-22-dev  # (INTERNAL) for BumbleBee stereo camera
sudo apt install -y libbluetooth-dev libcwiid-dev  # (INTERNAL) for joystick drivers

# Dependencies from Python packages
cd ${VTRSRC} && pip3 install -r requirements.txt
```

### Install [CUDA](https://developer.nvidia.com/cuda-toolkit) (>=11.3)

We recommend install CUDA from Debian packages following instructions [here](https://developer.nvidia.com/cuda-toolkit). Be sure to perform [post-installation actions](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index). Do not forget to put the following line in `.bashrc`:

```bash
export PATH=/usr/local/cuda-<your cuda version, e.g. 11.3>/bin${PATH:+:${PATH}}
```

You can check the CUDA driver version using `nvidia-smi` and CUDA toolkit version using `nvcc --version`.

### Install [Eigen](https://eigen.tuxfamily.org/index.php?title=Main_Page) (>=3.3.7)

```bash
# Debian package
sudo apt -y install libeigen3-dev

# OR from source if preferred
mkdir -p ${VTRDEPS}/eigen && cd $_
git clone https://gitlab.com/libeigen/eigen.git . && git checkout 3.3.7
mkdir build && cd $_
cmake .. && make install # default install location is /usr/local/
```

### Install [PROJ](https://proj.org/) (>=8.0.0)

The instruction below is mostly copied from [this page](https://proj.org/install.html#compilation-and-installation-from-source-code), except that the source code is cloned from GitHub.

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

Install OpenCV with CUDA from source to a customized location so that it is not conflicted with OpenCV installed from Debian packages. The instruction below is copied from [this page](https://docs.opencv.org/trunk/d7/d9f/tutorial_linux_install.html) with install location changed to `/usr/local/opencv_cuda` to be different from the default `/usr/local`.

```bash
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev python3-dev python3-numpy
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
# generate Makefiles (note that install prefix is customized to: /usr/local/opencv_cuda)
cmake -D CMAKE_BUILD_TYPE=RELEASE \
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
      -D BUILD_EXAMPLES=ON ..
make -j<nproc>  # <nproc> is the number of cpu cores of your computer, 12 for Lenovo P53
sudo make install  # copy libraries to /usr/local/[lib, include]

export LD_LIBRARY_PATH=/usr/local/opencv_cuda/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}  # put this in bashrc, note that the path should match CMAKE_INSTALL_PREFIX
```

### Install [ROS2 Foxy](https://www.ros.org/) (and optionally ROS1 Noetic+ros1_bridge)

Follow [this page](https://docs.ros.org/en/foxy/Installation.html) to install ROS2 Foxy binary packages or build from source. For Debian package installation, the `test_msgs` package needs to be installed manually

```bash
sudo apt install -y ros-foxy-test-msgs  # given that ROS2 Foxy is also installed from Debian packages
```

If you are working with robots or sensors that are ROS1 but not ROS2 enabled, also install ROS1 Noetic following instructions [here](http://wiki.ros.org/noetic/Installation) (install binary packages or build from source). [ros1_bridge](https://github.com/ros2/ros1_bridge/tree/foxy) is required to pass messages between ROS1 and ROS2, which can be [built from source](https://github.com/ros2/ros1_bridge/tree/foxy) or installed from Debian packages

```bash
sudo apt install -y ros-foxy-ros1-bridge  # given that ROS2 Foxy is also installed from Debian packages
```

### Install dependencies from ROS2 packages

```bash
mkdir -p ${VTRDEPS}/vtr_ros2_deps/src
# xacro (for robot descriptions)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros/xacro.git ros2_xacro
cd ros2_xacro
git checkout 2.0.3
# vision opencv (for camera T&R)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/vision_opencv.git ros2_vision_opencv
cd ros2_vision_opencv
git checkout ros2
# ros2_pcl_msgs (for LiDAR T&R)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/pcl_msgs.git ros2_pcl_msgs
cd ros2_pcl_msgs
git checkout ros2
# ros2_perception (for LiDAR T&R)
cd ${VTRDEPS}/vtr_ros2_deps/src
git clone https://github.com/ros-perception/perception_pcl.git ros2_perception_pcl
cd ros2_perception_pcl
git checkout 2.2.0
# install all
cd ${VTRDEPS}/vtr_ros2_deps
source /opt/ros/foxy/setup.bash  # source ros2 workspace first, e.g. for Debian package install
colcon build --symlink-install
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash  # source the overlayed workspace
```

## Build and install driver and robot description packages

For external users, we assume that the necessary sensor drivers and robot description packages have been installed already, so that data from sensors can be published to a ROS2 topic, and robot frames and transformations are published via the [robot_state_publisher](http://wiki.ros.org/robot_state_publisher).

- See [this page](./new_sensor_robot.md) for how to use VT&amp;R3 with your own camera/LiDAR sensors and robots.
- See ROS2 packges in [this directory](../robots/ros2) for some example robot description packages.

ASRL students please follow instructions on [this page](./internal_driver_installation.md) to install drivers for our sensors.

Our [sample dataset](./datasets.md) for VT&amp;R3 are collected using the UTIAS Grizzly platform equipped with a Waymo Honeycomb LiDAR and a FLIR Bumblebee XB3 stereo camera. To use these datasets, it is necessary to install the UTIAS Grizzly description packages in [this directory](../robots/ros2).

```bash
# source the ROS2 workspace with necessary dependencies installed
source ${VTRDEPS}/vtr_ros2_deps/install/setup.bash  # this is an example command, not necessarily the one you would use

# install UTIAS Grizzly description packages
cd ${VTRSRC}/robots/ros2
colcon build --symlink-install
source ${VTRSRC}/robots/ros2/install/setup.bash
```

## Build and install VT&amp;R3

Source the ROS2 workspace with all dependencies installed

```bash
# source the ROS2 workspace with all VTR3 dependencies installed
source ${VTRSRC}/vtr_ros2_deps/ros2/install/setup.bash  # this is an example command, not necessarily the one you would use

# build and install VTR3 packages
cd ${VTRSRC}/main
colcon build --symlink-install

# build VTR3 web-based GUI
VTRUI=${VTRSRC}/main/src/vtr_ui/vtr_ui/frontend/vtr-ui
npm --prefix ${VTRUI} install ${VTRUI}
npm --prefix ${VTRUI} run build
```

### Build and install VT&amp;R3 add-ons

These additional ROS2 packages are mainly used for testing and running VT&amp;R3 offline with datasets. The [vtr_bumblebee_xb3](../extra/src/vtr_bumblebee_xb3) package converts stereo image output from the pgr_triclop library to VT&amp;R3 expected input format.

Source the ROS2 workspace with VT&amp;R3 plus drivers and robot packages installed

```bash
# source the ROS2 workspaces with VTR3, drivers and robot description packages installed
source ${VTRSRC}/main/install/setup.bash

# build and install add-ons
cd ${VTRSRC}/extra
colcon build --symlink-install
source ${VTRSRC}/extra/install/setup.bash
```

After installing VT&amp;R3, the in-source documentation can be accessed by opening [index.html](../main/install/vtr_documentation/docs/html/index.html) at `main/install/vtr_documentation/docs/html/index.html` in browser.
